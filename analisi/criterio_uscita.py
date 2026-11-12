"""Criterio di uscita della fase 1: wall following su una o più bag di Gazebo.

Per ogni prova (una cartella con dentro la bag):
  - distanza VERA dalla parete, dalla posa di /odom e dal poligono della stanza
    (non da quello che crede il rilevamento);
  - errore sui tratti dritti (lontano dagli spigoli) e deviazione agli spigoli,
    separata per tipo di spigolo (concavo, smusso, convesso);
  - giri completati e controllo "senza intervento".

Uso (dalla cartella analisi/):
    env -u PYTHONPATH uv run criterio_uscita.py dati/wf_v040_01
    env -u PYTHONPATH uv run criterio_uscita.py dati/wf_*            # più prove + riepilogo
Opzioni utili:
    --stanza stanza_gazebo.json   poligono della stanza (default: accanto allo script)
    --giri 3                      giri attesi: meno di così → prova fallita
    --msg ../src/limo_wf_msgs/msg/WallInfo.msg   per leggere anche /wall_info
    --da 0 --a 1e9                finestra di tempo da analizzare [s dall'inizio bag]
Dipendenze nel progetto uv: numpy, matplotlib, rosbags  (uv add rosbags)
"""
import argparse
import json
import math
from pathlib import Path

import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_typestore, get_types_from_msg

QUI = Path(__file__).resolve().parent


# --------------------------------------------------------------------------- stanza
def carica_stanza(path):
    s = json.loads(Path(path).read_text())
    V = np.array(s['vertici'], float)
    if signed_area(V) < 0:                       # sempre antiorario
        V = V[::-1]
    return V, s


def signed_area(V):
    x, y = V[:, 0], V[:, 1]
    return 0.5 * float(np.sum(x * np.roll(y, -1) - np.roll(x, -1) * y))


def tipi_spigoli(V, lato):
    """Tipo di ogni vertice visto dal robot. Con il poligono antiorario un vertice con
    angolo interno < 180° è un muro che si chiude davanti al robot (concavo, o smusso
    se l'angolo interno è ~135°); > 180° è uno spigolo convesso. Non dipende dal lato:
    il robot gira dentro la stanza comunque."""
    out = []
    n = len(V)
    for i in range(n):
        a, b, c = V[i - 1], V[i], V[(i + 1) % n]
        e1, e2 = b - a, c - b
        cross = e1[0] * e2[1] - e1[1] * e2[0]
        svolta = math.degrees(math.atan2(cross, float(e1 @ e2)))   # >0 sinistra (CCW)
        interno = 180.0 - svolta
        if interno > 180.5:
            out.append('convesso')
        elif interno > 112.5:
            out.append('smusso')
        else:
            out.append('concavo')
    return out


def distanze_pareti(P, V):
    """Distanza di ogni punto P (N,2) da ogni parete (lati del poligono) → (N, n_lati)."""
    A, B = V, np.roll(V, -1, axis=0)
    E = B - A
    L2 = (E * E).sum(1)
    u = np.clip(((P[:, None, :] - A[None]) * E[None]).sum(2) / L2[None], 0.0, 1.0)
    Q = A[None] + u[..., None] * E[None]
    return np.linalg.norm(P[:, None, :] - Q, axis=2)


# --------------------------------------------------------------------------- bag
def leggi_bag(cartella, msg_path=None):
    ts = get_typestore(Stores.ROS2_HUMBLE)
    if msg_path:
        ts.register(get_types_from_msg(Path(msg_path).read_text(), 'limo_wf_msgs/msg/WallInfo'))
    bags = [p.parent for p in Path(cartella).rglob('metadata.yaml')]
    if not bags:
        raise SystemExit(f'{cartella}: nessuna bag (metadata.yaml) trovata')
    odom, cmd, scan, wi = [], [], [], []
    with AnyReader(bags, default_typestore=ts) as r:
        for c, t, raw in r.messages():
            tt = t * 1e-9
            if c.topic == '/odom':
                m = r.deserialize(raw, c.msgtype)
                p, q = m.pose.pose.position, m.pose.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                odom.append((tt, p.x, p.y, yaw, m.twist.twist.linear.x))
            elif c.topic == '/cmd_vel':
                m = r.deserialize(raw, c.msgtype)
                cmd.append((tt, m.linear.x, m.angular.z))
            elif c.topic == '/scan' and len(scan) < 400:
                m = r.deserialize(raw, c.msgtype)
                scan.append((tt, m.angle_min, m.angle_increment, m.range_min, m.range_max,
                             np.asarray(m.ranges, float)))
            elif c.topic == '/wall_info' and msg_path:
                m = r.deserialize(raw, c.msgtype)
                wi.append((tt, getattr(m, 'ref_corner', False), getattr(m, 'wall_detected', False)))
    if not odom:
        raise SystemExit(f'{cartella}: /odom assente nella bag')
    return (np.array(odom), np.array(cmd) if cmd else np.zeros((0, 3)), scan,
            np.array(wi, float) if wi else None)


def controllo_allineamento(scan, odom, V, laser_x):
    """Proietta un campione di scansioni con /odom: se la stanza è nel frame giusto i
    punti cadono sulle pareti (pochi mm). Altrimenti la distanza 'vera' non vale nulla."""
    if not scan:
        return None
    pts = []
    for (tt, amin, inc, rmin, rmax, rg) in scan[::10]:
        i = int(np.argmin(np.abs(odom[:, 0] - tt)))
        x, y, yaw = odom[i, 1:4]
        a = amin + inc * np.arange(len(rg))
        ok = np.isfinite(rg) & (rg > rmin) & (rg < min(rmax, 3.0))
        lx, ly = laser_x + rg[ok] * np.cos(a[ok]), rg[ok] * np.sin(a[ok])
        pts.append(np.column_stack((x + lx * math.cos(yaw) - ly * math.sin(yaw),
                                    y + lx * math.sin(yaw) + ly * math.cos(yaw))))
    P = np.vstack(pts)
    d = distanze_pareti(P[:: max(1, len(P) // 4000)], V).min(1)
    return float(np.median(d))


# --------------------------------------------------------------------------- analisi
def analizza(cartella, args, V, tipi):
    odom, cmd, scan, wi = leggi_bag(cartella, args.msg)
    t0 = odom[0, 0]
    m = (odom[:, 0] - t0 >= args.da) & (odom[:, 0] - t0 <= args.a)
    # si parte dal primo movimento: prima il controllore può non essere ancora abilitato
    mossa = np.where(m & (np.abs(odom[:, 4]) > 0.05))[0]
    if len(mossa):
        m &= np.arange(len(odom)) >= mossa[0]
    odom = odom[m]
    t = odom[:, 0] - t0
    P = odom[:, 1:3]
    D = distanze_pareti(P, V)
    d = D.min(1)                                         # distanza vera dalla parete più vicina
    e = d - args.d_ref

    # zona: spigolo se entro raggio_spigolo da un vertice, altrimenti tratto dritto
    dv = np.linalg.norm(P[:, None, :] - V[None], axis=2)
    iv = dv.argmin(1)
    vicino = dv.min(1) < args.raggio_spigolo
    dritto = ~vicino

    # giri: rotazione totale / 2π (per un poligono semplice un giro vale ±2π)
    yaw_u = np.unwrap(odom[:, 3])
    giri = abs(yaw_u[-1] - yaw_u[0]) / (2 * math.pi)
    percorso = float(np.sum(np.linalg.norm(np.diff(P, axis=0), axis=1)))
    durata = float(t[-1] - t[0])

    # velocità nominale: mediana del comando sui tratti dritti
    v_nom = float('nan')
    if len(cmd):
        tc = cmd[:, 0] - t0
        zc = np.interp(tc, t, dritto.astype(float)) > 0.5
        mv = zc & (cmd[:, 1] > 0.01)
        if mv.any():
            v_nom = float(np.median(cmd[mv, 1]))

    # senza intervento
    fermo = np.abs(odom[:, 4]) < 0.02
    fermo_max = 0.0
    run = 0.0
    for k in range(1, len(t)):
        run = run + (t[k] - t[k - 1]) if fermo[k] else 0.0
        fermo_max = max(fermo_max, run)
    problemi = []
    if d.min() < args.d_sicurezza:
        problemi.append(f'distanza minima {d.min():.3f} m < {args.d_sicurezza}')
    if fermo_max > args.fermo_max:
        problemi.append(f'fermo per {fermo_max:.1f} s')
    if args.giri and giri < args.giri - 0.05:
        problemi.append(f'giri {giri:.2f} < {args.giri}')

    allin = controllo_allineamento(scan, odom, V, args.laser_x)
    if allin is not None and allin > 0.03:
        problemi.append(f'stanza non allineata al frame di /odom (mediana {allin*1000:.0f} mm): '
                        'distanze vere non affidabili')

    # spigoli: per ogni passaggio, min e max distanza dentro la zona del vertice
    passaggi = []
    k = 0
    while k < len(t):
        if vicino[k]:
            j = k
            while j + 1 < len(t) and vicino[j + 1] and iv[j + 1] == iv[k]:
                j += 1
            seg = slice(k, j + 1)
            passaggi.append(dict(vertice=int(iv[k]), tipo=tipi[iv[k]], t=float(t[k]),
                                 dmin=float(d[seg].min()), dmax=float(d[seg].max())))
            k = j + 1
        else:
            k += 1

    orbite = None
    if wi is not None and len(wi):
        wi = wi[(wi[:, 0] >= odom[0, 0]) & (wi[:, 0] <= odom[-1, 0])]
        rc = wi[:, 1] > 0.5
        orbite = int(np.sum(rc[1:] & ~rc[:-1]))

    res = dict(prova=Path(cartella).name, v_nom=v_nom, durata=durata, percorso=percorso, giri=giri,
               rms_dritti=float(np.sqrt(np.mean(e[dritto] ** 2))) if dritto.any() else float('nan'),
               bias_dritti=float(np.mean(e[dritto])) if dritto.any() else float('nan'),
               p95_dritti=float(np.percentile(np.abs(e[dritto]), 95)) if dritto.any() else float('nan'),
               max_dritti=float(np.abs(e[dritto]).max()) if dritto.any() else float('nan'),
               dmin=float(d.min()), fermo_max=fermo_max, allineamento=allin,
               orbite=orbite, problemi=problemi, passaggi=passaggi)
    for tipo in ('concavo', 'smusso', 'convesso'):
        pp = [p for p in passaggi if p['tipo'] == tipo]
        res[f'{tipo}_n'] = len(pp)
        res[f'{tipo}_dmin'] = min((p['dmin'] for p in pp), default=float('nan'))
        res[f'{tipo}_dmax'] = max((p['dmax'] for p in pp), default=float('nan'))

    # CSV campione per campione
    np.savetxt(Path(cartella) / 'distanza_vera.csv',
               np.column_stack((t, P, d, e, dritto.astype(int))),
               delimiter=',', header='t,x,y,d_vera,errore,dritto', comments='', fmt='%.4f')
    figura(cartella, res, t, P, d, e, dritto, V, tipi, args)
    return res


def figura(cartella, res, t, P, d, e, dritto, V, tipi, args):
    fig = plt.figure(figsize=(12, 8.5))
    gs = fig.add_gridspec(2, 2, height_ratios=[1.35, 1], width_ratios=[1.25, 1])
    ax = fig.add_subplot(gs[0, 0])
    Vc = np.vstack((V, V[:1]))
    ax.plot(Vc[:, 0], Vc[:, 1], color='#3a4652', lw=2)
    lim = 0.10
    sc = ax.scatter(P[:, 0], P[:, 1], c=np.clip(e, -lim, lim), cmap='RdBu_r', s=3, vmin=-lim, vmax=lim)
    for i, (v, tp) in enumerate(zip(V, tipi)):
        ax.add_patch(plt.Circle(v, args.raggio_spigolo, fill=False, ls=':', color='#8a96a3', lw=0.8))
        ax.annotate(tp, v, xytext=(4, 4), textcoords='offset points', fontsize=7, color='#5b6876')
    ax.set_aspect('equal'); ax.set_xlabel('x [m]'); ax.set_ylabel('y [m]')
    ax.set_title('Percorso, colore = errore di distanza (rosso: lontano, blu: vicino)', fontsize=10)
    fig.colorbar(sc, ax=ax, shrink=0.75, label='d − d_ref [m]')

    ax2 = fig.add_subplot(gs[0, 1])
    ax2.hist(e[dritto] * 1000, bins=60, color='#1b7f8c', alpha=0.85)
    ax2.axvline(0, color='#3a4652', lw=0.8)
    ax2.set_xlabel('errore sui tratti dritti [mm]'); ax2.set_ylabel('campioni')
    ax2.set_title(f"dritti: RMS {res['rms_dritti']*1000:.1f} mm, bias {res['bias_dritti']*1000:+.1f} mm, "
                  f"p95 {res['p95_dritti']*1000:.1f} mm", fontsize=10)

    ax3 = fig.add_subplot(gs[1, :])
    ax3.plot(t, d, color='#3a4652', lw=0.8)
    ax3.axhline(args.d_ref, color='#1b7f8c', lw=0.8, ls='--')
    ax3.fill_between(t, 0, 1, where=~dritto, color='#c77700', alpha=0.12, transform=ax3.get_xaxis_transform(),
                     label='zona spigolo')
    ax3.set_ylim(max(0.0, d.min() - 0.05), max(args.d_ref + 0.25, d.max() + 0.02))
    ax3.set_xlabel('t [s]'); ax3.set_ylabel('distanza vera [m]'); ax3.legend(loc='upper right', fontsize=8)
    esito = 'OK' if not res['problemi'] else 'FALLITA: ' + '; '.join(res['problemi'])
    fig.suptitle(f"{res['prova']} · v_nom {res['v_nom']:.2f} m/s · {res['giri']:.2f} giri · {esito}", fontsize=11)
    fig.tight_layout()
    fig.savefig(Path(cartella) / 'criterio_uscita.png', dpi=130)
    plt.close(fig)


def stampa(res):
    mm = lambda x: f'{x*1000:6.1f}' if np.isfinite(x) else '     —'
    print(f"\n== {res['prova']} ==")
    print(f"  v_nominal {res['v_nom']:.2f} m/s · durata {res['durata']:.0f} s · percorso {res['percorso']:.1f} m · giri {res['giri']:.2f}")
    if res['allineamento'] is not None:
        print(f"  allineamento stanza/odom: {res['allineamento']*1000:.1f} mm (mediana punti LiDAR → pareti)")
    print(f"  dritti [mm]: RMS {mm(res['rms_dritti'])}  bias {mm(res['bias_dritti'])}  p95 {mm(res['p95_dritti'])}  max {mm(res['max_dritti'])}")
    for tipo in ('concavo', 'smusso', 'convesso'):
        if res[f'{tipo}_n']:
            print(f"  {tipo:<9} {res[f'{tipo}_n']:3d} passaggi · d min {res[f'{tipo}_dmin']:.3f} m · d max {res[f'{tipo}_dmax']:.3f} m")
    print(f"  distanza minima assoluta {res['dmin']:.3f} m · fermo più lungo {res['fermo_max']:.1f} s")
    if res['orbite'] is not None:
        nc = res['convesso_n']
        print(f"  orbite iniziate (ref_corner): {res['orbite']} su {nc} passaggi da spigoli convessi"
              f"  ({'giusto: una per passaggio' if res['orbite'] == nc else 'ATTENZIONE: dovrebbe essere una per passaggio'})")
    print('  ESITO:', 'OK, senza intervento' if not res['problemi'] else 'FALLITA → ' + '; '.join(res['problemi']))


def riepilogo(tutti):
    print('\n== Riepilogo per velocità ==')
    print(f"{'v_nom':>6} {'prove':>6} {'giri':>6} {'RMS dritti [mm]':>17} {'bias [mm]':>10} "
          f"{'concavo min/max [m]':>21} {'smusso min/max [m]':>20} {'convesso min/max [m]':>22} {'OK':>4}")
    gruppi = {}
    for r in tutti:
        gruppi.setdefault(round(r['v_nom'], 1), []).append(r)
    for v in sorted(gruppi):
        g = gruppi[v]
        rms = np.array([r['rms_dritti'] for r in g]) * 1000
        bias = np.array([r['bias_dritti'] for r in g]) * 1000
        def mm(tipo):
            a = min(r[f'{tipo}_dmin'] for r in g); b = max(r[f'{tipo}_dmax'] for r in g)
            return f'{a:.3f} / {b:.3f}' if np.isfinite(a) else '—'
        sd = f' ± {rms.std(ddof=1):.1f}' if len(g) > 1 else ''
        print(f"{v:6.1f} {len(g):6d} {sum(r['giri'] for r in g):6.1f} {rms.mean():11.1f}{sd:>6} {bias.mean():10.1f} "
              f"{mm('concavo'):>21} {mm('smusso'):>20} {mm('convesso'):>22} {sum(not r['problemi'] for r in g):>2}/{len(g)}")


def figura_riepilogo(tutti, cartella):
    """Una figura per tutto il batch: errore a regime e distanze agli spigoli contro la velocità."""
    v = np.array([r['v_nom'] for r in tutti])
    fig, (a1, a2) = plt.subplots(1, 2, figsize=(11, 4.2))
    a1.scatter(v, [r['rms_dritti'] * 1000 for r in tutti], color='#1b7f8c', label='RMS', zorder=3)
    a1.scatter(v, [r['p95_dritti'] * 1000 for r in tutti], color='#8a96a3', marker='s', label='p95', zorder=3)
    a1.set_xlabel('v_nominal [m/s]'); a1.set_ylabel('errore sui tratti dritti [mm]')
    a1.set_title('Rettilinei', fontsize=10); a1.grid(alpha=0.3); a1.legend(fontsize=8)
    colori = {'concavo': '#c77700', 'smusso': '#6b7785', 'convesso': '#a3359b'}
    off = {'concavo': -0.012, 'smusso': 0.0, 'convesso': 0.012}
    for tipo, c in colori.items():
        lo = [r[f'{tipo}_dmin'] for r in tutti]; hi = [r[f'{tipo}_dmax'] for r in tutti]
        if not np.isfinite(lo).any():
            continue
        x = v + off[tipo]
        a2.vlines(x, lo, hi, color=c, lw=2, label=tipo)
        a2.scatter(x, lo, color=c, s=14); a2.scatter(x, hi, color=c, s=14)
    a2.axhline(tutti[0].get('d_ref', 0.30), color='#1b7f8c', ls='--', lw=0.8, label='d_ref')
    a2.set_xlabel('v_nominal [m/s]'); a2.set_ylabel('distanza vera nella zona di spigolo [m]')
    a2.set_title('Spigoli: intervallo min–max per prova', fontsize=10); a2.grid(alpha=0.3); a2.legend(fontsize=8)
    ok = sum(not r['problemi'] for r in tutti)
    fig.suptitle(f'Criterio di uscita: {ok}/{len(tutti)} prove senza intervento', fontsize=11)
    fig.tight_layout()
    out = Path(cartella) / 'criterio_uscita_riepilogo.png'
    fig.savefig(out, dpi=130); plt.close(fig)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('prove', nargs='+', help='cartelle delle prove (con la bag dentro)')
    ap.add_argument('--stanza', default=str(QUI / 'stanza_gazebo.json'))
    ap.add_argument('--d-ref', type=float, default=0.30)
    ap.add_argument('--raggio-spigolo', type=float, default=1.0,
                    help='entro questa distanza da un vertice il campione conta come spigolo [m]')
    ap.add_argument('--giri', type=float, default=0.0, help='giri attesi (0 = non controllare)')
    ap.add_argument('--d-sicurezza', type=float, default=0.15)
    ap.add_argument('--fermo-max', type=float, default=3.0, help='fermo più lungo ammesso [s]')
    ap.add_argument('--laser-x', type=float, default=0.103, help='posizione del laser in base_link [m]')
    ap.add_argument('--msg', default=None, help='WallInfo.msg, per leggere anche /wall_info')
    ap.add_argument('--da', type=float, default=0.0)
    ap.add_argument('--a', type=float, default=1e9)
    args = ap.parse_args()

    V, _ = carica_stanza(args.stanza)
    tipi = tipi_spigoli(V, None)
    tutti = []
    prove = [c for c in args.prove if Path(c).is_dir()]
    for c in args.prove:
        if not Path(c).is_dir():
            print(f'(ignoro {c}: non è una cartella)')
    if not prove:
        raise SystemExit('nessuna cartella di prova fra gli argomenti')
    for c in prove:
        r = analizza(c, args, V, tipi)
        stampa(r)
        tutti.append(r)
    if len(tutti) > 1:
        riepilogo(tutti)
        print(f"\nfigura del riepilogo: {figura_riepilogo(tutti, Path(prove[0]).parent)}")
    print('figure delle singole prove: <cartella prova>/criterio_uscita.png')
    Path(prove[0]).parent.joinpath('criterio_uscita_riepilogo.json').write_text(
        json.dumps([{k: v for k, v in r.items() if k != 'passaggi'} for r in tutti], indent=2))


if __name__ == '__main__':
    main()
