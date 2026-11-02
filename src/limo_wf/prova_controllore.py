"""Collaudo offline della legge di controllo. Nessun ROS, nessun Gazebo.
Integra la cinematica dell'uniciclo e verifica che il robot converga su d_ref."""
import math
import numpy as np
from limo_wf.wall_follow_controller_logic import ControlParams, WallFollowController
from limo_wf.wall_geometry_detection import SIDE_LEFT, SIDE_RIGHT

DT = 0.125                                  # periodo vero dell'anello (LiDAR a 8 Hz)

def simula(side, d0, psi0_deg=0.0, T=25.0, seed=0):
    p = ControlParams(side=side)
    ctrl = WallFollowController(p)
    s = 1.0 if side == SIDE_LEFT else -1.0
    rng = np.random.default_rng(seed)
    d, psi = d0, math.radians(psi0_deg)
    storia = []
    for _ in range(int(T / DT)):
        dm = d + rng.normal(0, 0.0008)                      # rumore misurato
        am = -psi + rng.normal(0, math.radians(0.04))       # angle = -psi
        cmd = ctrl.update(True, dm, am)
        storia.append((d, math.degrees(psi), cmd.omega))
        d += -s * cmd.v * math.sin(psi) * DT
        psi += cmd.omega * DT
    return np.array(storia)

print(f"{'caso':<34}{'d finale':>10}{'psi finale':>12}{'d min':>9}{'d max':>9}")
for side, nome in ((SIDE_LEFT, 'sinistra'), (SIDE_RIGHT, 'destra')):
    for d0, psi0 in ((0.50, 0), (0.15, 0), (0.30, 20), (0.30, -20)):
        h = simula(side, d0, psi0)
        print(f'{nome + f"  d0={d0:.2f} psi0={psi0:+}°":<34}'
              f'{h[-1,0]:>10.3f}{h[-1,1]:>11.2f}°{h[:,0].min():>9.3f}{h[:,0].max():>9.3f}')

print('\nparete non rilevata:',
      WallFollowController(ControlParams()).update(False, float('nan'), float('nan')))