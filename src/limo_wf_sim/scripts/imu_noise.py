#!/usr/bin/env python3
"""
Inietta un bias costante sul giroscopio simulato.

Il plugin IMU di Gazebo aggiunge solo rumore bianco a media nulla
(sigma 2e-4 rad/s), che integrato da' un random walk di ampiezza
trascurabile.  Un giroscopio MEMS vero ha invece un offset sistematico
sulla velocita' angolare: integrato nel tempo produce una deriva lineare
dell'heading, ed e' il termine che domina l'errore di orientamento.

Il bias si applica solo a angular_velocity.z, l'unico asse che conta per
un robot che si muove nel piano.  Tutto il resto del messaggio (header,
orientamento, accelerazioni, assi x e y, covarianze) passa invariato.

Ingresso:  /imu        uscita del plugin Gazebo (best effort, 100 Hz)
Uscita:    /imu_noisy  giroscopio come lo misurerebbe il robot
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu


class ImuNoise(Node):
    def __init__(self):
        super().__init__('imu_noise')

        # valore PROVVISORIO: da sostituire con il bias misurato sul robot
        # reale (media di angular_velocity.z a robot fermo)
        self.declare_parameter('gyro_bias', 0.001)     # rad/s
        # riservato a un futuro bias estratto a caso
        self.declare_parameter('seed', 0)

        self.bias = self.get_parameter('gyro_bias').value

        # il plugin pubblica in best effort: un subscriber reliable non
        # riceverebbe nulla
        self.create_subscription(Imu, 'imu', self.on_imu, qos_profile_sensor_data)
        # reliable (default): compatibile con subscriber best effort e reliable
        self.pub = self.create_publisher(Imu, 'imu_noisy', 10)

        self.get_logger().info(
            f'imu_noise: bias gyro z={self.bias:.4g} rad/s '
            f'({math.degrees(self.bias):.4f} deg/s), '
            f'deriva attesa {math.degrees(self.bias * 60.0):.2f} deg dopo 60 s')

    def on_imu(self, msg: Imu):
        # la covarianza resta quella del plugin: descrive il rumore bianco,
        # mentre il bias e' un errore sistematico e non va nella covarianza
        msg.angular_velocity.z += self.bias
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = ImuNoise()
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
