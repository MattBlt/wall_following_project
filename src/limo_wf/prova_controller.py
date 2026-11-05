import math
import numpy as np
from limo_wf.wall_follow_controller_logic import ControlParams, WallFollowController
from limo_wf.wall_geometry_detection import SIDE_LEFT, SIDE_RIGHT

DT = 0.125
INF = float('inf')

def simula(side, d0, psi0_deg=0.0, T=25.0, seed=0, front=INF):
    p = ControlParams(side=side)
    ctrl = WallFollowController(p)
    s = 1.0 if side == SIDE_LEFT else -1.0
    rng = np.random.default_rng(seed)
    d, psi = d0, math.radians(psi0_deg)
    storia = []
    for _ in range(int(T / DT)):
        cmd = ctrl.update(True, d + rng.normal(0, 0.0008),
                          -psi + rng.normal(0, math.radians(0.04)), front, DT)
        storia.append((d, math.degrees(psi), cmd.v, cmd.omega))
        d += -s * cmd.v * math.sin(psi) * DT
        psi += cmd.omega * DT
    return np.array(storia)

print('--- inseguimento della parete (davanti libero) ---')
print(f"{'caso':<34}{'d finale':>10}{'psi finale':>12}{'d min':>9}{'d max':>9}")
for side, nome in ((SIDE_LEFT, 'sinistra'), (SIDE_RIGHT, 'destra')):
    for d0, psi0 in ((0.50, 0), (0.15, 0), (0.30, 20), (0.30, -20)):
        h = simula(side, d0, psi0)
        print(f'{nome + f"  d0={d0:.2f} psi0={psi0:+}°":<34}'
              f'{h[-1,0]:>10.3f}{h[-1,1]:>11.2f}°{h[:,0].min():>9.3f}{h[:,0].max():>9.3f}')

print('\n--- profilo di velocita\' ---')
c = WallFollowController(ControlParams())
print(f'decelerazione ricavata: {c.decel():.3f} m/s^2')
print(f"{'front [m]':>10}{'v [m/s]':>10}")
for f in (2.00, 1.20, 1.00, 0.90, 0.80, 0.70, 0.60, 0.56, 0.55, 0.40):
    print(f'{f:>10.2f}{c.cruise_speed(f):>10.3f}')

print('\n--- avvicinamento a un muro frontale ---')
p = ControlParams(); ctrl = WallFollowController(p)
d_front, t = 1.60, 0.0
while t < 30 and d_front > p.front_stop:
    cmd = ctrl.update(True, 0.30, 0.0, d_front, DT)
    d_front -= cmd.v * DT
    t += DT
print(f'front_stop raggiunto a t={t:.1f} s, front={d_front:.3f} m, fase={cmd.reason}')
cmd = ctrl.update(True, 0.30, 0.0, d_front, DT)
print(f'scansione successiva: v={cmd.v:.3f}  omega={cmd.omega:+.3f}  fase={cmd.reason}')