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
    v_nominal: float = 0.20            # [m/s] velocita' lineare, costante nel punto 2

    # Comportamento voluto dell'anello chiuso (da qui si ricavano i guadagni)
    omega_n: float = 1.0               # [rad/s] banda: a 8 Hz di campionamento e'
                                       # un fattore 50, ampiamente al sicuro
    zeta: float = 1.0                  # smorzamento: 1.0 = nessun sorpasso

    max_omega: float = 1.5             # [rad/s] saturazione del comando
    v_min: float = 0.05                # [m/s] K_d = omega_n^2/v diverge per v->0

    # --- ricerca della parete dopo uno spigolo
    v_search: float = 0.12             # [m/s] piu' lenta: si procede quasi alla cieca
    advance_distance: float = 0.35     # [m] quanto proseguire dritto prima di ruotare,
                                       # per superare lo spigolo e vedere la svolta
    omega_turn: float = 0.6            # [rad/s] rotazione sul posto (spigolo concavo)
    front_stop: float = 0.45           # [m] sotto: c'e' un muro davanti
    max_search_yaw: float = 2.62       # [rad] ~150 gradi: oltre, rinuncia e si ferma


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

    def gains(self, v: float) -> tuple[float, float]:
        """K_d dipende dalla velocita'. Nel punto 2 v e' costante, ma quando il
        punto 3 iniziera' a modularla il guadagno va ricalcolato, altrimenti la
        dinamica cambia sotto i piedi."""
        p = self.params
        v = max(v, p.v_min)
        return p.omega_n ** 2 / v, 2.0 * p.zeta * p.omega_n

    def update(self, wall_detected: bool, distance: float, angle: float,
               front_distance: float, dt: float) -> Command:
        p = self.params
        s = 1.0 if p.side == SIDE_LEFT else -1.0
        front = front_distance if math.isfinite(front_distance) else math.inf
        ok = wall_detected and math.isfinite(distance) and math.isfinite(angle)

        # --- un muro davanti prevale su tutto: si gira via dal lato seguito,
        #     anche se la parete laterale e' ancora agganciata. Altrimenti il
        #     robot continua a seguire il fianco fino a sbatterci contro.
        if front < p.front_stop:
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
            v = p.v_nominal
            k_d, k_psi = self.gains(v)
            omega = s * k_d * (distance - p.d_ref) + k_psi * angle
            omega = max(-p.max_omega, min(p.max_omega, omega))
            return Command(v, omega, True, Phase.FOLLOW)

        # --- parete persa e davanti libero: prima dritto, poi arco
        if self.phase == Phase.FOLLOW:
            self.phase = Phase.ADVANCE
            self.advanced = 0.0

        if self.phase == Phase.ADVANCE:
            self.advanced += p.v_search * dt
            if self.advanced < p.advance_distance:
                return Command(p.v_search, 0.0, True, Phase.ADVANCE)
            self.phase, self.turn_sign = Phase.TURN, s     # spigolo superato
            self.turned = 0.0

        if self.phase == Phase.TURN:
            if self.turned < p.max_search_yaw:
                v = p.v_search                             # arco di raggio d_ref
                omega = self.turn_sign * v / p.d_ref
                self.turned += abs(omega) * dt
                return Command(v, omega, True, Phase.TURN)
            self.phase = Phase.LOST

        return Command(0.0, 0.0, False, Phase.LOST)