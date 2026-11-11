"""Legge di controllo del wall following — Python puro, nessuna dipendenza da ROS.

Il controllore comanda la velocita' angolare per mantenere la distanza di
riferimento dalla parete. Il termine in `angle` non e' un secondo obiettivo:
e' il termine derivativo, perche' dalla cinematica dell'uniciclo

    de/dt = -v * psi        (psi = -angle, orientamento rispetto alla parete)

l'orientamento E' la derivata dell'errore di distanza. Misurarlo invece di
derivare la distanza evita di amplificare il rumore.

Dinamica dell'anello chiuso:

    e'' + K_psi * e' + v*K_d * e = 0

cioe' un secondo ordine con omega_n = sqrt(v*K_d) e zeta = K_psi/(2*sqrt(v*K_d)).
I guadagni si ricavano invertendo: si sceglie il comportamento, non il guadagno.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from limo_wf.wall_geometry_detection import SIDE_LEFT, SIDE_RIGHT


class Phase:
    """Fasi del controllore. Finiscono in Command.reason, quindi il nodo puo'
    loggare le transizioni e si vede in chiaro quale ramo e' stato preso."""
    FOLLOW = 'segue'
    ADVANCE = 'avanza'
    TURN = 'ruota'
    LOST = 'fermo'


@dataclass
class ControlParams:
    """Parametri del controllore. I default servono ai test: sul robot
    arrivano dal YAML tramite il nodo."""

    side: int = SIDE_LEFT              # deve combaciare con quello di wall_detection
    d_ref: float = 0.30                # [m] distanza di riferimento dalla parete
    v_nominal: float = 0.40            # [m/s] velocita' lineare, costante nel punto 2

    # Comportamento voluto dell'anello chiuso (da qui si ricavano i guadagni)
    omega_n: float = 1.0               # [rad/s] banda: a 8 Hz di campionamento e'
                                       # un fattore 50, ampiamente al sicuro
    zeta: float = 1.0                  # smorzamento: 1.0 = nessun sorpasso

    max_omega: float = 1.5             # [rad/s] saturazione del comando
    v_min: float = 0.05                # [m/s] K_d = omega_n^2/v diverge per v->0

    # --- anello di velocita': profilo a decelerazione costante.
    # Si azzera esattamente su front_stop, cosi' la macchina a stati prende il
    # comando a robot fermo invece che in frenata.
    d_slow: float = 1.00               # [m] da qui inizia a rallentare
    v_floor: float = 0.05              # [m/s] velocita' minima: sotto questa il
                                       # profilo la tiene costante fino a
                                       # front_stop, dove va a zero di colpo
    accel_max: float = 0.30            # [m/s^2] aumento massimo della velocita'
    decel_max: float = 1.00            # [m/s^2] diminuzione massima della velocita'

    # --- ricerca della parete dopo uno spigolo
    v_search: float = 0.12             # [m/s] piu' lenta: si procede quasi alla cieca
    advance_distance: float = 0.20     # [m] quanto proseguire dritto prima di ruotare,
                                       # per superare lo spigolo e vedere la svolta
    omega_turn: float = 0.6            # [rad/s] rotazione sul posto (spigolo concavo)
    front_release: float = 0.20        # [m] isteresi in uscita dalla virata: si
                                       # smette di girare solo oltre
                                       # front_stop + front_release, altrimenti si
                                       # esce quando il muro vecchio e' ancora li'
                                       # e si rientra subito dopo
    front_stop: float = 0.40           # [m] sotto: c'e' un muro davanti. E' anche
                                       # il punto in cui cruise_speed va a zero
    max_search_yaw: float = 2.62       # [rad] ~150 gradi: oltre, rinuncia e si ferma
    front_turn: float = 0.75           # [m] sotto questa distanza frontale parte la virata ad arco
    v_turn: float = 0.25               # [m/s] velocita' lungo l'arco
    turn_radius_min: float = 0.22      # [m] raggio minimo dell'arco
    v_orbit: float = 0.30              # [m/s] velocita' massima attorno allo spigolo


@dataclass
class Command:
    """Cosa il nodo mette nel Twist."""
    v: float
    omega: float
    active: bool                       # False = fermo, parete non rilevata
    reason: str = ''


class WallFollowController:

    def __init__(self, params: ControlParams):
        self.params = params
        self.phase = Phase.FOLLOW
        self.advanced = 0.0            # [m] percorsi dritti da quando ha perso
        self.turned = 0.0              # [rad] ruotati da quando cerca
        self.turn_sign = 1.0           # verso della rotazione in corso
        self.v_prev = 0.0
        self.arc_active = False

    def gains(self):
        """Guadagni calcolati sulla velocita' NOMINALE, non su quella corrente.

        K_d = omega_n^2/v manterrebbe omega_n costante solo se v fosse un ingresso
        esterno. Qui non lo e': v dipende da front_distance, che dipende da dove
        punta il robot, che dipende da omega. Programmare il guadagno su una
        variabile interna all'anello ne chiude un secondo, e quello e' positivo:
        il robot sterza verso il muro, il muro entra nel corridoio frontale, v
        scende, K_d sale, sterza di piu'. Misurato su muro dritto in 40 s: 3.47 m
        percorsi e 121 transizioni di fase, contro 7.52 m e 1 transizione.
        """
        p = self.params
        v = max(p.v_nominal, p.v_min)      # v_min protegge da v_nominal -> 0,
        return p.omega_n ** 2 / v, 2.0 * p.zeta * p.omega_n

    def decel(self) -> float:
        """Decelerazione ricavata dagli estremi del profilo, cosi' nel YAML si
        mettono distanze (leggibili) invece di un'accelerazione."""
        p = self.params
        span = max(1e-3, p.d_slow - p.front_stop)
        return p.v_nominal ** 2 / (2.0 * span)

    def cruise_speed(self, front: float) -> float:
        """Profilo a decelerazione costante: v = sqrt(2*a*(d - front_stop)).

        Una rampa lineare in distanza da' un esponenziale nel tempo (v' = -k v),
        quindi il robot striscia verso la soglia senza arrivarci: misurato, 16 s
        contro 7. La radice arriva a zero in tempo finito ed esattamente dove
        serve. v_floor evita la coda e la tangente verticale in front_stop.
        """
        p = self.params
        if front <= p.front_stop:
            return 0.0
        v = math.sqrt(2.0 * self.decel() * (front - p.front_stop))
        return max(p.v_floor, min(p.v_nominal, v))

    def update(self, wall_detected, distance, angle, front_distance, dt, corner=False):
        cmd = self._compute(wall_detected, distance, angle, front_distance, dt, corner)
        if cmd.v <= 0.0:
            v = 0.0                      # fermate e rotazioni sul posto: subito
        else:
            p = self.params
            v = min(self.v_prev + p.accel_max * dt,
                    max(self.v_prev - p.decel_max * dt, cmd.v))
        self.v_prev = v
        if self.arc_active and cmd.v > 0.0:
            omega = cmd.omega * v / cmd.v
        else:
            omega = cmd.omega
        return Command(v, omega, cmd.active, cmd.reason)

    def _compute(self, wall_detected: bool, distance: float, angle: float,
                 front_distance: float, dt: float, corner=False) -> Command:
        self.arc_active = False
        p = self.params
        s = 1.0 if p.side == SIDE_LEFT else -1.0
        front = front_distance if math.isfinite(front_distance) else math.inf
        ok = wall_detected and math.isfinite(distance) and math.isfinite(angle)

        turning_away = self.phase == Phase.TURN and self.turn_sign == -s
        arc_exit = p.front_turn + (p.front_release if turning_away else 0.0)
        if self.phase != Phase.LOST and p.front_stop <= front < arc_exit:
            if self.phase != Phase.TURN or self.turn_sign != -s:
                self.phase, self.turn_sign, self.turned = Phase.TURN, -s, 0.0
            R = max(p.turn_radius_min, front - p.d_ref)
            v = p.v_turn
            omega = self.turn_sign * v / R
            self.turned += abs(omega) * dt
            self.arc_active = True
            return Command(v, omega, True, Phase.TURN)

        # --- un muro davanti prevale su tutto: si gira via dal lato seguito,
        #     anche se la parete laterale e' ancora agganciata. Altrimenti il
        #     robot continua a seguire il fianco fino a sbatterci contro.
        # Isteresi in uscita: una volta che sta girando via dal muro frontale, non
        # smette appena front risale sopra front_stop — a quel punto il muro vecchio
        # e' ancora li' e rientrerebbe subito, oscillando. Serve superare
        # front_stop + front_release.
        if front < p.front_stop or (turning_away and front < p.front_stop + p.front_release):
            if self.phase == Phase.LOST:
                return Command(0.0, 0.0, False, Phase.LOST)   # LOST si aggancia
            if self.phase != Phase.TURN or self.turn_sign != -s:
                self.phase, self.turn_sign, self.turned = Phase.TURN, -s, 0.0
            if self.turned < p.max_search_yaw:
                omega = self.turn_sign * p.omega_turn
                self.turned += abs(omega) * dt
                return Command(0.0, omega, True, Phase.TURN)
            self.phase = Phase.LOST
            return Command(0.0, 0.0, False, Phase.LOST)

        # --- parete rilevata: si segue
        if ok:
            self.phase = Phase.FOLLOW
            self.advanced = 0.0
            self.turned = 0.0
            v = self.cruise_speed(front)
            if corner:
                v = min(v, p.v_orbit)
            k_d, k_psi = self.gains()
            omega = s * k_d * (distance - p.d_ref) + k_psi * angle
            if corner:
                omega += s * v / distance        # feedforward di curvatura dell'orbita
            omega = max(-p.max_omega, min(p.max_omega, omega))
            return Command(v, omega, True, Phase.FOLLOW)

        # --- parete persa e davanti libero: prima dritto, poi arco
        if self.phase == Phase.FOLLOW:
            self.phase = Phase.ADVANCE
            self.advanced = 0.0

        if self.phase == Phase.ADVANCE:
            v = min(p.v_search, self.cruise_speed(front))
            self.advanced += v * dt
            if self.advanced < p.advance_distance:
                return Command(v, 0.0, True, Phase.ADVANCE)
            self.phase, self.turn_sign = Phase.TURN, s     # spigolo superato
            self.turned = 0.0

        if self.phase == Phase.TURN:
            if self.turned < p.max_search_yaw:
                v = min(p.v_search, self.cruise_speed(front))
                omega = (self.turn_sign * v / p.d_ref if v > 0.0
                         else self.turn_sign * p.omega_turn)
                self.turned += abs(omega) * dt
                return Command(v, omega, True, Phase.TURN)
            self.phase = Phase.LOST

        return Command(0.0, 0.0, False, Phase.LOST)