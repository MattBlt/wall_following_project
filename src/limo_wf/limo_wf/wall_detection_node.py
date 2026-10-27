"""
Wall_detection - dalla scansione LiDAR alla stima dellla parete seguita

Sottoscrive:    scan (sensor_msgs/LaserScan)
Pubblica:       wall_info (limo_wf_msgs/WallInfo)
                wall_markers (visualization_msgs/MarkerArray) - per RViz (debug)

Tutta la geometria sta in wall_geometry_detection.py; questo è solo il nodo ROS2
"""


import math

import rclpy
from geometry_msgs.msg import Point
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import TransformException, TransformListener, Buffer
from visualization_msgs.msg import Marker, MarkerArray

from limo_wf_msgs.msg import WallInfo
from limo_wf.wall_geometry_detection import (
    SIDE_LEFT, SIDE_RIGHT, DetectionParams, WallTracker, detect_wall, scan_to_points,
)


def yaw_from_quaternion(q) -> float:
    return math.atan2(2.0 * (q.w * q.z +q.x * q.y), 1.0 -2.0 * (q.y * q.y + q.z * q.z))



class WallDetection(Node):
    def __init__(self):
        super().__init__('wall_detection')

        # parametri: valori di default, sovrascritti dal YAML nel launch
        self.declare_parameter('laser_frame', '')        # vuoto = usa quello dello scan
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('side', 'left')
        self.declare_parameter('sector_half_width_deg', 85.0)
        self.declare_parameter('gap_threshold', 0.20)
        self.declare_parameter('split_threshold', 0.05)
        self.declare_parameter('merge_threshold', 0.025)
        self.declare_parameter('min_points', 8)
        self.declare_parameter('min_segment_length', 0.30)
        self.declare_parameter('max_wall_distance', 2.0)
        self.declare_parameter('max_wall_angle_deg', 60.0)
        self.declare_parameter('front_half_width', 0.20)
        self.declare_parameter('switch_margin', 0.10)
        self.declare_parameter('publish_markers', True)

        self.laser_frame = self.get_parameter('laser_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_markers = self.get_parameter('publish_markers').value

        side_str = self.get_parameter('side').value.lower()
        if side_str not in ('left', 'right'):
            raise ValueError(f"parametro 'side' non valido: {side_str!r}")
        self.side = SIDE_LEFT if side_str == 'left' else SIDE_RIGHT

        gp = self.get_parameter
        self.params = DetectionParams(
            side=self.side,
            sector_half_width=math.radians(gp('sector_half_width_deg').value),
            gap_threshold=gp('gap_threshold').value,
            split_threshold=gp('split_threshold').value,
            merge_threshold=gp('merge_threshold').value,
            min_points=gp('min_points').value,
            min_segment_length=gp('min_segment_length').value,
            max_wall_distance=gp('max_wall_distance').value,
            max_wall_angle=math.radians(gp('max_wall_angle_deg').value),
            front_half_width=gp('front_half_width').value,
            switch_margin=gp('switch_margin').value,
        )
        self.tracker = WallTracker(self.params)

        # TF statica laser -> base_link: letta una volta e tenuta in cache
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.laser_pose = None
        self.scan_info_logged = False

        # nomi PROVVISORI, vengono rimappati poi nel launch
        self.create_subscription(LaserScan, 'scan', self.on_scan, qos_profile_sensor_data)
        self.pub = self.create_publisher(WallInfo, 'wall_info', 10)
        self.marker_pub = self.create_publisher(MarkerArray, 'wall_markers', 10)

        self.get_logger().info(f'wall_detection avviato, lato: {side_str}')


    # ------------------------------------------------------------------ #
    def lookup_laser_pose(self, frame_from_scan: str) -> bool:
        frame = self.laser_frame or frame_from_scan
        if self.laser_frame and self.laser_frame != frame_from_scan:
            self.get_logger().warn(
                f"parametro laser_frame '{self.laser_frame}' diverso dal frame dello "
                f"scan '{frame_from_scan}': uso il parametro", once=True)
        try:
            tf = self.tf_buffer.lookup_transform(
                self.base_frame, frame, Time(), timeout=Duration(seconds=0.0))
        except TransformException as exc:
            self.get_logger().warn(
                f'TF {self.base_frame} <- {frame} non ancora disponibile: {exc}',
                throttle_duration_sec=2.0)
            return False
        t, q = tf.transform.translation, tf.transform.rotation
        self.laser_pose = (t.x, t.y, yaw_from_quaternion(q))
        self.get_logger().info(
            f'laser in {self.base_frame}: x={t.x:.3f} y={t.y:.3f} '
            f'yaw={math.degrees(self.laser_pose[2]):.1f} gradi')
        return True
    

    def log_scan_info(self, msg: LaserScan):
        n = len(msg.ranges)
        fov = math.degrees(msg.angle_increment * (n - 1))
        self.get_logger().info(
            f'scan: {n} raggi, [{math.degrees(msg.angle_min):.1f}, '
            f'{math.degrees(msg.angle_min) + fov:.1f}] gradi, passo '
            f'{math.degrees(msg.angle_increment):.3f} gradi, portata '
            f'[{msg.range_min:.2f}, {msg.range_max:.2f}] m, frame {msg.header.frame_id}')
        self.scan_info_logged = True

    
    # ------------------------------------------------------------------ #
    def on_scan(self, msg: LaserScan):
        if not self.scan_info_logged:
            self.log_scan_info(msg)
        if self.laser_pose is None and not self.lookup_laser_pose(msg.header.frame_id):
            return

        points = scan_to_points(msg.ranges, msg.angle_min, msg.angle_increment,
                                msg.range_min, msg.range_max, self.laser_pose)
        est = detect_wall(points, self.params, self.tracker)

        out = WallInfo()
        out.header.stamp = msg.header.stamp       # il tempo viene dallo scan
        out.header.frame_id = self.base_frame     # ma i dati sono in base_link
        out.wall_detected = bool(est.detected)
        out.side = WallInfo.SIDE_LEFT if self.side == SIDE_LEFT else WallInfo.SIDE_RIGHT
        out.distance = float(est.distance)              # nan se non rilevata
        out.angle = float(est.angle)                    # nan se non rilevata
        out.front_distance = float(est.front_distance)  # inf se davanti e' libero
        out.segment_length = float(est.segment_length)
        out.inlier_count = int(est.inlier_count)
        out.fit_residual = float(est.fit_residual)
        self.pub.publish(out)

        if not est.detected:
            self.get_logger().warn('nessuna parete sul lato seguito',
                                   throttle_duration_sec=1.0)

        if self.publish_markers:
            self.marker_pub.publish(self.make_markers(est, msg.header.stamp))

    
    # ------------------------------------------------------------------ #
    def make_markers(self, est, stamp) -> MarkerArray:
        """Grigio: tutti i segmenti estratti. Verde: la parete scelta."""
        def line_list(mid, segments, rgba, width):
            m = Marker()
            m.header.frame_id = self.base_frame
            m.header.stamp = stamp
            m.ns = 'wall_detection'
            m.id = mid
            m.type = Marker.LINE_LIST
            m.action = Marker.ADD
            m.pose.orientation.w = 1.0
            m.scale.x = width
            m.color.r, m.color.g, m.color.b, m.color.a = rgba
            for seg in segments:
                m.points.append(Point(x=float(seg.p_start[0]), y=float(seg.p_start[1])))
                m.points.append(Point(x=float(seg.p_end[0]), y=float(seg.p_end[1])))
            return m

        arr = MarkerArray()
        arr.markers.append(line_list(0, est.segments, (0.6, 0.6, 0.6, 0.8), 0.02))
        arr.markers.append(line_list(1, [est.wall] if est.wall is not None else [],
                                     (0.1, 0.8, 0.2, 1.0), 0.05))
        return arr


def main():
    rclpy.init()
    node = WallDetection()
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