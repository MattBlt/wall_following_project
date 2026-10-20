#!/usr/bin/env python3
"""
Test isolato della cinematica di rotazione.

Comanda una rotazione pura su /cmd_vel per alcuni secondi e misura la
velocita' angolare reale su /odom (ground truth dal plugin Gazebo).
Stampa il rapporto fra misurato e comandato.

Non dipende dal workspace: si lancia con `python3 spin_test.py` dopo aver
sorgiato ROS. Nessun colcon build, nessun percorso di install.
"""
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

CMD = -0.4          # rad/s comandati
DUR = 6.0           # s di comando
WINDOW = 2.0        # s finali su cui mediare


class SpinTest(Node):
    def __init__(self):
        super().__init__('spin_test')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10)

        self.samples = []
        self.t0 = None
        self.done = False
        self.create_timer(0.05, self.tick)

        print(f'comando {CMD:+.2f} rad/s per {DUR:.0f} s...')

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_odom(self, msg):
        if self.t0 is not None:
            self.samples.append((self.now() - self.t0, msg.twist.twist.angular.z))

    def tick(self):
        if self.t0 is None:
            self.t0 = self.now()
            return

        t = self.now() - self.t0
        msg = Twist()

        if t < DUR:
            msg.angular.z = CMD
            self.pub.publish(msg)
            return

        self.pub.publish(Twist())
        self.report()
        self.done = True

    def report(self):
        if not self.samples:
            print('\nNessun campione da /odom.')
            print('La simulazione non e\' in esecuzione, oppure /odom non viene pubblicato.')
            return

        tail = [w for (t, w) in self.samples if t > DUR - WINDOW]
        if not tail:
            tail = [w for (_, w) in self.samples]

        avg = sum(tail) / len(tail)
        peak = min(tail) if CMD < 0 else max(tail)

        print()
        print(f'  comandato : {CMD:+.3f} rad/s')
        print(f'  misurato  : {avg:+.3f} rad/s   (media ultimi {WINDOW:.0f} s, {len(tail)} campioni)')
        print(f'  picco     : {peak:+.3f} rad/s')
        print(f'  resa      : {abs(avg / CMD) * 100:.0f} %')
        print()
        if abs(avg) < 0.02:
            print('  -> robot fermo: il comando non arriva, o il robot e\' bloccato.')
        elif abs(avg / CMD) < 0.7:
            print('  -> rotazione insufficiente: attrito laterale ancora troppo alto.')
        else:
            print('  -> cinematica di rotazione corretta.')


def main():
    rclpy.init()
    node = SpinTest()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.pub.publish(Twist())
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
