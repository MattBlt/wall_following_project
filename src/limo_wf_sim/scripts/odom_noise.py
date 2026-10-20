#!/usr/bin/env python3
"""
Inietta un errore realistico sull'odometria simulata.

Due meccanismi distinti, parametrizzati separatamente:

1. Rumore di lettura dell'encoder (errore casuale, limitato).
   Encoder ottico a n tacche/giro, risoluzione 2*pi/n rad.  La LETTURA
   del conteggio cumulativo e' perturbata da una pmf triangolare:
       P(0) = 2/5, P(+-1) = 1/5, P(+-2) = 1/10
   a media nulla e varianza 1.2 tacche^2, approssimata con una gaussiana
   N(0, 1.2) tacche come da indicazione del relatore.

   Il rumore si applica al CONTEGGIO CUMULATIVO, non all'incremento:
   l'incremento usato per l'integrazione e' la differenza fra due letture
   consecutive, quindi gli errori di lettura in gran parte si elidono e
   l'errore di posizione che ne deriva resta limitato.  Perturbare
   direttamente gli incrementi produrrebbe un random walk, cioe' deriva
   illimitata anche a robot fermo: non e' il comportamento di un encoder.

2. Errore sistematico di calibrazione (bias, non si media).
   Le due ruote hanno raggio effettivo diverso da quello nominale:
       R_sx = R*(1+eps),  R_dx = R*(1-eps)
   Il software integra con R nominale, quindi su traiettoria rettilinea
   accumula una rotazione spuria.  E' l'errore sistematico classico
   dell'odometria differenziale (UMBmark, Borenstein & Feng 1996), ed e'
   la componente che domina la deriva su percorsi lunghi.

Ingresso:  /odom        ground truth dal plugin Gazebo
Uscita:    /odom_noisy  odometria come la misurerebbe il robot
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


def yaw_from_quat(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y ** 2 + q.z ** 2))


def quat_from_yaw(yaw):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)


class OdomNoise(Node):
    def __init__(self):
        super().__init__('odom_noise')

        self.declare_parameter('ticks_per_rev', 1024)
        self.declare_parameter('tick_sigma', 1.0954)   # sqrt(1.2)
        self.declare_parameter('radius_bias', 0.005)   # eps, 0.5%
        self.declare_parameter('wheel_radius', 0.045)
        self.declare_parameter('wheel_separation', 0.172)
        self.declare_parameter('seed', 0)

        self.n = self.get_parameter('ticks_per_rev').value
        self.tick_sigma = self.get_parameter('tick_sigma').value
        self.eps = self.get_parameter('radius_bias').value
        self.R = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('wheel_separation').value

        self.tick = 2.0 * math.pi / self.n          # rad/tacca
        self.arc_per_tick = self.R * self.tick      # m/tacca
        self.rng = np.random.default_rng(self.get_parameter('seed').value)

        # posa stimata, parte dalla stessa origine del ground truth
        self.x = self.y = self.th = 0.0
        self.prev = None
        self.cum = [0.0, 0.0]       # archi veri cumulati, sinistra e destra
        self.last_read = [0, 0]     # ultime letture dell'encoder, in tacche

        self.create_subscription(Odometry, 'odom', self.on_odom, 10)
        self.pub = self.create_publisher(Odometry, 'odom_noisy', 10)

        self.get_logger().info(
            f'odom_noise: n={self.n} tacche/giro ({math.degrees(self.tick):.3f} deg), '
            f'sigma={self.tick_sigma:.3f} tacche sulla lettura, '
            f'bias eps={self.eps * 100:.2f}%')

    def read_encoder(self, cumulative_arc):
        """Lettura dell'encoder: conteggio cumulativo quantizzato e perturbato."""
        ticks = cumulative_arc / self.arc_per_tick
        return int(round(ticks + self.rng.normal(0.0, self.tick_sigma)))

    def on_odom(self, msg: Odometry):
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        th = yaw_from_quat(msg.pose.pose.orientation)

        if self.prev is None:
            self.x, self.y, self.th = x, y, th
            self.prev = (x, y, th)
            return

        px, py, pth = self.prev
        self.prev = (x, y, th)

        # incremento vero, nel frame del robot
        ds = math.hypot(x - px, y - py)
        if (x - px) * math.cos(pth) + (y - py) * math.sin(pth) < 0:
            ds = -ds
        dth = math.atan2(math.sin(th - pth), math.cos(th - pth))

        # cinematica differenziale inversa: archi veri delle due ruote
        d_left = ds - dth * self.L / 2.0
        d_right = ds + dth * self.L / 2.0

        # --- 1. bias: il raggio effettivo non e' quello nominale ---
        self.cum[0] += d_left * (1.0 + self.eps)
        self.cum[1] += d_right * (1.0 - self.eps)

        # --- 2. lettura degli encoder, e incremento come differenza ---
        read_l = self.read_encoder(self.cum[0])
        read_r = self.read_encoder(self.cum[1])
        d_left = (read_l - self.last_read[0]) * self.arc_per_tick
        d_right = (read_r - self.last_read[1]) * self.arc_per_tick
        self.last_read = [read_l, read_r]

        # integrazione con i parametri NOMINALI: e' quello che fa il robot
        ds_m = (d_right + d_left) / 2.0
        dth_m = (d_right - d_left) / self.L

        self.th = math.atan2(math.sin(self.th + dth_m), math.cos(self.th + dth_m))
        self.x += ds_m * math.cos(self.th)
        self.y += ds_m * math.sin(self.th)

        self.publish(msg.header)

    def publish(self, header):
        out = Odometry()
        out.header = header
        out.child_frame_id = 'base_link'
        out.pose.pose.position.x = self.x
        out.pose.pose.position.y = self.y
        qz, qw = quat_from_yaw(self.th)
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        self.pub.publish(out)


def main():
    rclpy.init()
    node = OdomNoise()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()