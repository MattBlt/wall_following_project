import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from limo_wf_msgs.msg import WallInfo


class WallDetection(Node):
    def __init__(self):
        super().__init__('wall_detection')

        # parametri: valori di default, sovrascritti dal YAML nel launch
        self.declare_parameter('laser_frame', 'laser_link')
        self.declare_parameter('side', 'right')

        self.laser_frame = self.get_parameter('laser_frame').value
        self.side = self.get_parameter('side').value

        # nomi PROVVISORI, vengono rimappati poi nel launch
        self.create_subscription(LaserScan, 'scan', self.on_scan, 10)
        self.pub = self.create_publisher(WallInfo, 'wall_info', 10)

        self.get_logger().info(f'wall_detection avviato, lato: {self.side}')

    def on_scan(self, msg: LaserScan):
        out = WallInfo()
        out.header = msg.header
        out.wall_detected = False
        out.side = WallInfo.SIDE_RIGHT if self.side == 'right' else WallInfo.SIDE_LEFT
        self.pub.publish(out)


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
