#!/usr/bin/env python3
"""wall_follow_controller — anello sull'angolo (fase 1, punto 2).

Sottoscrive:  wall_info   (limo_wf_msgs/WallInfo)
Pubblica:     cmd_vel     (geometry_msgs/Twist)

Nomi relativi: quelli veri arrivano dalla rimappatura in limo_wf_bringup.
La legge di controllo sta in wall_follow_controller_logic.py; qui c'e' solo la
colla con ROS, piu' il watchdog e l'abilitazione.
"""

import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult

from limo_wf_msgs.msg import WallInfo
from limo_wf.wall_follow_controller_logic import ControlParams, Phase, WallFollowController
from limo_wf.wall_geometry_detection import SIDE_LEFT, SIDE_RIGHT


class WallFollowControllerNode(Node):

    def __init__(self):
        super().__init__('wall_follow_controller')

        self.declare_parameter('enabled', False)      # parte fermo, apposta
        self.declare_parameter('d_ref', 0.30)
        self.declare_parameter('v_nominal', 0.40)
        self.declare_parameter('omega_n', 1.0)
        self.declare_parameter('zeta', 1.0)
        self.declare_parameter('max_omega', 1.5)
        self.declare_parameter('v_min', 0.05)
        self.declare_parameter('d_slow', 1.00)
        self.declare_parameter('v_floor', 0.05)
        self.declare_parameter('watchdog_timeout', 0.5)
        self.declare_parameter('v_search', 0.12)
        self.declare_parameter('advance_distance', 0.20)
        self.declare_parameter('omega_turn', 0.6)
        self.declare_parameter('front_stop', 0.40)
        self.declare_parameter('front_release', 0.20)
        self.declare_parameter('max_search_yaw', 2.62)
        self.declare_parameter('accel_max', 0.30)
        self.declare_parameter('decel_max', 1.00)
        self.declare_parameter('front_turn', 0.75)
        self.declare_parameter('v_turn', 0.25)
        self.declare_parameter('turn_radius_min', 0.22)
        self.declare_parameter('v_orbit', 0.30)
        self.add_on_set_parameters_callback(self.on_param_change)

        gp = self.get_parameter
        self.params = ControlParams(
            side=SIDE_LEFT,                          # provvisorio: lo detta il messaggio
            d_ref=gp('d_ref').value,
            v_nominal=gp('v_nominal').value,
            omega_n=gp('omega_n').value,
            zeta=gp('zeta').value,
            max_omega=gp('max_omega').value,
            v_min=gp('v_min').value,
            d_slow=gp('d_slow').value,
            v_floor=gp('v_floor').value,    
            v_search=gp('v_search').value,
            advance_distance=gp('advance_distance').value,
            omega_turn=gp('omega_turn').value,
            front_stop=gp('front_stop').value,
            front_release=gp('front_release').value,
            max_search_yaw=gp('max_search_yaw').value,
            accel_max=gp('accel_max').value,
            decel_max=gp('decel_max').value,
            front_turn=gp('front_turn').value,
            v_turn=gp('v_turn').value,
            turn_radius_min=gp('turn_radius_min').value,
            v_orbit=gp('v_orbit').value,
        )
        self.controller = WallFollowController(self.params)
        self.watchdog_timeout = gp('watchdog_timeout').value

        self.last_msg_time = None
        self.stopped_by_watchdog = False
        self.prev_time = None
        self.prev_phase = ''

        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.create_subscription(WallInfo, 'wall_info', self.on_wall_info, 10)
        self.create_timer(0.1, self.on_watchdog)
        self.on_start()

    def on_start(self):
        k_d, k_psi = self.controller.gains()
        self.get_logger().info(
            f'wall_follow_controller avviato: d_ref={self.params.d_ref:.2f} m, '
            f'v={self.params.v_nominal:.2f} m/s, omega_n={self.params.omega_n:.2f}, '
            f'zeta={self.params.zeta:.2f}  ->  K_d={k_d:.2f}, K_psi={k_psi:.2f}')
        self.get_logger().info(
            f'velocita\': piena oltre {self.params.d_slow:.2f} m, zero a '
            f'{self.params.front_stop:.2f} m  ->  decelerazione '
            f'{self.controller.decel():.3f} m/s^2')
        self.get_logger().warn(
            "fermo: abilitare con  ros2 param set /wall_follow_controller enabled true")

    # ------------------------------------------------------------------ #
    def is_enabled(self) -> bool:
        return bool(self.get_parameter('enabled').value)

    def publish(self, v: float, omega: float):
        msg = Twist()
        msg.linear.x = float(v)
        msg.angular.z = float(omega)
        self.pub.publish(msg)

    # ------------------------------------------------------------------ #
    def on_param_change(self, params):
        """Rilegge i parametri a caldo. Senza, ogni valore provato costa un riavvio:
        il costruttore li copia una volta sola in ControlParams."""
        for prm in params:
            if prm.name == 'watchdog_timeout':
                self.watchdog_timeout = prm.value
                self.get_logger().info(f'{prm.name} -> {prm.value}')
            elif hasattr(self.params, prm.name):
                setattr(self.params, prm.name, prm.value)
                self.get_logger().info(f'{prm.name} -> {prm.value}')
        return SetParametersResult(successful=True)

    # ----------------------------------------------------------------- #
    def on_wall_info(self, msg: WallInfo):
        self.last_msg_time = self.get_clock().now()
        self.stopped_by_watchdog = False

        # il lato lo detta chi rileva: una sola fonte di verita'
        self.params.side = SIDE_LEFT if msg.side == WallInfo.SIDE_LEFT else SIDE_RIGHT

        if not self.is_enabled():
            self.prev_time = None
            self.controller.phase = Phase.FOLLOW
            self.controller.advanced = 0.0
            self.controller.turned = 0.0
            self.controller.v_prev = 0.0
            self.publish(0.0, 0.0)
            return

        dt = 0.125 if self.prev_time is None else \
            max(1e-3, (self.last_msg_time - self.prev_time).nanoseconds * 1e-9)
        self.prev_time = self.last_msg_time

        cmd = self.controller.update(msg.ref_valid, msg.ref_distance, msg.ref_angle,
                                     msg.front_distance, dt, msg.ref_corner)
        self.publish(cmd.v, cmd.omega)

        if cmd.reason != self.prev_phase:
            self.get_logger().info(f'fase: {self.prev_phase or "-"} -> {cmd.reason}')
            self.prev_phase = cmd.reason

    # ------------------------------------------------------------------ #
    def on_watchdog(self):
        """Se wall_info smette di arrivare, il robot deve fermarsi: l'ultimo
        cmd_vel resterebbe valido e il robot continuerebbe a eseguirlo."""
        if self.last_msg_time is None or self.stopped_by_watchdog:
            return
        eta = (self.get_clock().now() - self.last_msg_time).nanoseconds * 1e-9
        if eta > self.watchdog_timeout:
            self.publish(0.0, 0.0)
            self.stopped_by_watchdog = True
            self.prev_time = None
            self.controller.v_prev = 0.0
            self.get_logger().error(
                f'watchdog: nessun wall_info da {eta:.1f} s, robot fermato')


def main():
    rclpy.init()
    node = WallFollowControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # fermare il robot prima di uscire, non lasciarlo in moto
        try:
            node.publish(0.0, 0.0)
            rclpy.spin_once(node, timeout_sec=0.1)
        except Exception:
            pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()