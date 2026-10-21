#!/usr/bin/env python3
"""
Esporta un bag di prova in CSV.

Metà ROS della pipeline di analisi: legge /odom (ground truth) e
/odom_noisy (odometria grezza) e li scrive in due CSV, che la cartella
analisi/ legge senza sapere nulla di ROS.

Gira con il Python di sistema nell'ambiente ROS, perche' rosbag2_py e'
un pacchetto compilato e non esiste su PyPI:

    cd ~/ros2_ws/wall_following_project
    source install/setup.zsh
    ros2 run limo_wf_sim export_bag.py analisi/dati/quadrato_01
"""
import math
import os
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from nav_msgs.msg import Odometry

TOPICS = {'/odom': 'odom', '/odom_noisy': 'odom_noisy'}


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def open_reader(path):
    errors = []
    for storage_id in ('sqlite3', 'mcap'):
        try:
            r = rosbag2_py.SequentialReader()
            r.open(rosbag2_py.StorageOptions(uri=path, storage_id=storage_id),
                   rosbag2_py.ConverterOptions(input_serialization_format='cdr',
                                               output_serialization_format='cdr'))
            return r
        except Exception as exc:
            errors.append(f'  {storage_id}: {exc}')
    sys.exit(f"impossibile aprire il bag '{path}':\n" + '\n'.join(errors))


def main():
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    bag = sys.argv[1].rstrip('/')
    if not os.path.isdir(bag):
        sys.exit(f"cartella del bag non trovata: {bag}")

    rows = {t: [] for t in TOPICS}
    reader = open_reader(bag)
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic not in rows:
            continue
        m = deserialize_message(data, Odometry)
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose
        rows[topic].append((t, p.position.x, p.position.y, yaw_of(p.orientation)))

    for topic, r in rows.items():
        if not r:
            sys.exit(f"nessun messaggio su {topic}: il bag ha registrato quel topic?")

    t0 = min(r[0][0] for r in rows.values())
    base = os.path.join(bag, os.path.basename(bag))

    for topic, suffix in TOPICS.items():
        out = f'{base}_{suffix}.csv'
        with open(out, 'w') as f:
            f.write('t,x,y,yaw\n')
            for t, x, y, th in rows[topic]:
                f.write(f'{t - t0:.4f},{x:.6f},{y:.6f},{th:.6f}\n')
        print(f'{len(rows[topic]):6d} campioni  ->  {out}')


if __name__ == '__main__':
    main()