#!/usr/bin/env python3
"""
Criterio di uscita della fase 0: deriva dell'odometria grezza.

Meta' di analisi della pipeline: legge i CSV prodotti da export_bag.py e
produce la figura di confronto fra odometria grezza e ground truth.
Nessuna dipendenza da ROS.

    cd analisi
    uv run plot_deriva.py dati/quadrato_01
"""
import os
import sys

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

GT_COLOR, ODO_COLOR = '#1a1a1a', '#d1495b'


def load(path):
    a = np.loadtxt(path, delimiter=',', skiprows=1, ndmin=2)
    a = a[np.argsort(a[:, 0], kind='stable')]   # np.interp vuole t crescente
    return a.T          # t, x, y, yaw


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    bag = sys.argv[1].rstrip('/')
    base = os.path.join(bag, os.path.basename(bag))

    try:
        gt = load(base + '_odom.csv')
        odo = load(base + '_odom_noisy.csv')
    except OSError as exc:
        sys.exit(f"{exc}\nLanciare prima export_bag.py nell'ambiente ROS.")

    # l'odometria grezza viene riportata sulla base dei tempi del ground truth
    t = gt[0]
    ox = np.interp(t, odo[0], odo[1])
    oy = np.interp(t, odo[0], odo[2])
    oth = np.interp(t, odo[0], np.unwrap(odo[3]))
    gth = np.unwrap(gt[3])

    err_pos = np.hypot(ox - gt[1], oy - gt[2])
    err_yaw = np.degrees(np.arctan2(np.sin(oth - gth), np.cos(oth - gth)))

    fig = plt.figure(figsize=(11, 4.6))
    grid = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], hspace=0.45, wspace=0.28)

    ax = fig.add_subplot(grid[:, 0])
    ax.plot(gt[1], gt[2], color=GT_COLOR, lw=1.6, label='ground truth', zorder=3)
    ax.plot(ox, oy, color=ODO_COLOR, lw=1.6, label='odometria grezza', zorder=2)
    ax.plot(gt[1][0], gt[2][0], 'o', ms=6, mfc='white', mec=GT_COLOR, mew=1.5,
            zorder=4, label='partenza')
    ax.plot(gt[1][-1], gt[2][-1], 's', ms=5, color=GT_COLOR, zorder=4)
    ax.plot(ox[-1], oy[-1], 's', ms=5, color=ODO_COLOR, zorder=4)
    ax.set_aspect('equal', adjustable='datalim')
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    ax.set_title('Traiettoria', loc='left', fontsize=11)
    ax.legend(frameon=False, fontsize=9, loc='best')

    a1 = fig.add_subplot(grid[0, 1])
    a1.plot(t, err_pos, color=ODO_COLOR, lw=1.4)
    a1.set_ylabel('[m]')
    a1.set_title('Errore di posizione', loc='left', fontsize=11)

    a2 = fig.add_subplot(grid[1, 1], sharex=a1)
    a2.plot(t, err_yaw, color=ODO_COLOR, lw=1.4)
    a2.axhline(0, color='#999999', lw=0.7)
    a2.set_ylabel('[deg]')
    a2.set_xlabel('tempo [s]')
    a2.set_title('Errore di orientamento', loc='left', fontsize=11)

    for a in (ax, a1, a2):
        a.grid(True, color='#e6e6e6', lw=0.6)
        a.set_axisbelow(True)
        for side in ('top', 'right'):
            a.spines[side].set_visible(False)

    png = base + '_deriva.png'
    fig.savefig(png, dpi=200, bbox_inches='tight', facecolor='white')

    path_len = np.sum(np.hypot(np.diff(gt[1]), np.diff(gt[2])))
    print(f'durata              {t[-1]:.1f} s')
    print(f'percorso            {path_len:.2f} m')
    rel = f'   ({err_pos[-1] / path_len * 100:.2f}% del percorso)' if path_len > 0 else ''
    print(f'errore finale pos.  {err_pos[-1]:.3f} m{rel}')
    print(f'errore finale yaw   {err_yaw[-1]:+.2f} deg')
    print(f'errore pos. massimo {err_pos.max():.3f} m')
    print()
    print(f'figura  {png}')


if __name__ == '__main__':
    main()