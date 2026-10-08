import rclpy
from rclpy.node import Node

from std_msgs.msg import String

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            String,
            'scanTgit',
            self.listener_callback,
            10
            )
        self.subscription

    def listener_callback(self, msg):
        self.get_logger().info('Distance "%s"' % msg.data)

        