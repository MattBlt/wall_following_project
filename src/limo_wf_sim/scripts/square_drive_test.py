#!/usr/bin/env python3
"""
Percorso di prova per la caratterizzazione della deriva odometrica.

Quadrato percorso in catena chiusa sul ground truth (/odom del plugin
Gazebo): ogni lato termina quando la distanza percorsa raggiunge il
valore nominale, ogni rotazione quando lo yaw e' variato di 90 gradi.

La catena chiusa e' necessaria perche' il LIMO differenziale ha quattro
ruote rigide: ruotando sul posto striscia lateralmente, e la velocita'
angolare reale risulta circa meta' di quella comandata. Un percorso a
tempo produrrebbe rotazioni molto minori di 90 gradi.

Chiudere sul ground truth non falsa la misura: l'errore caratterizzato
e' quello di /odom_noisy rispetto a /odom, e il metodo UMBmark richiede
che il percorso vero sia un quadrato il piu' esatto possibile.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

STRAIGHT, TURN, PAUSE = 'straight', 'turn', 'pause'


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


class SquareDrive(Node):
    def __init__(self):
        super().__init__('square_drive')
        self.set_parameters([Parameter('use_sim_time', Parameter.Type.BOOL, True)])

        self.declare_parameter('side_length', 2.0)       # m
        self.declare_parameter('linear_speed', 0.20)     # m/s
        self.declare_parameter('angular_speed', 0.40)    # rad/s, modulo
        self.declare_parameter('turn_direction', -1)     # -1 destra, +1 sinistra
        self.declare_parameter('laps', 1)
        self.declare_parameter('pause', 1.0)             # s fra i segmenti
        self.declare_parameter('dist_tol', 0.01)         # m
        self.declare_parameter('yaw_tol', 0.01)          # rad

        self.L = self.get_parameter('side_length').value
        self.v = self.get_parameter('linear_speed').value
        self.w = self.get_parameter('angular_speed').value
        self.d = self.get_parameter('turn_direction').value
        laps = self.get_parameter('laps').value
        pause = self.get_parameter('pause').value
        self.dist_tol = self.get_parameter('dist_tol').value
        self.yaw_tol = self.get_parameter('yaw_tol').value

        self.steps = []
        for _ in range(4 * laps):
            self.steps.append((STRAIGHT, self.L))
            self.steps.append((PAUSE, pause))
            self.steps.append((TURN, self.d * math.pi / 2))
            self.steps.append((PAUSE, pause))

        self.pose = None          # (x, y, yaw) corrente
        self.ref = None           # riferimento all'ingresso nello step
        self.acc_yaw = 0.0        # rotazione accumulata nello step
        self.idx = 0
        self.t_entry = None

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = self.create_timer(0.05, self.tick)

        self.get_logger().info(
            f'square_drive: lato {self.L} m, {laps} giri, '
            f'verso {"orario" if self.d < 0 else "antiorario"}')

    def on_odom(self, msg):
        p = msg.pose.pose
        self.pose = (p.position.x, p.position.y, yaw_from_quat(p.orientation))

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def stop(self):
        self.pub.publish(Twist())

    def tick(self):
        if self.pose is None:
            return

        if self.idx >= len(self.steps):
            self.stop()
            self.get_logger().info('percorso completato')
            self.timer.cancel()
            return

        kind, target = self.steps[self.idx]
        x, y, yaw = self.pose

        # ingresso in un nuovo step: fissa il riferimento
        if self.ref is None:
            self.ref = (x, y, yaw)
            self.acc_yaw = 0.0
            self.t_entry = self.now()

        rx, ry, ryaw = self.ref
        msg = Twist()

        if kind == STRAIGHT:
            done = math.hypot(x - rx, y - ry)
            if done >= target - self.dist_tol:
                self.next_step()
                return
            msg.linear.x = self.v

        elif kind == TURN:
            # accumula l'incremento di yaw, cosi' il wrap a +-pi non rompe il conto
            self.acc_yaw += math.atan2(math.sin(yaw - ryaw), math.cos(yaw - ryaw))
            self.ref = (rx, ry, yaw)
            residuo = target - self.acc_yaw
            if abs(residuo) <= self.yaw_tol:
                self.next_step()
                return
            # rallenta in prossimita' del bersaglio, per limitare la sovraelongazione
            cmd = max(0.08, min(self.w, 2.0 * abs(residuo)))
            msg.angular.z = math.copysign(cmd, residuo)

        else:  # PAUSE
            if self.now() - self.t_entry >= target:
                self.next_step()
                return

        self.pub.publish(msg)

    def next_step(self):
        self.stop()
        self.idx += 1
        self.ref = None


def main():
    rclpy.init()
    node = SquareDrive()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()