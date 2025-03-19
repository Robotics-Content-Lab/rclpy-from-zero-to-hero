import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

    def odom_callback(self, msg: Odometry):
        self.get_logger().info('Received Odometry message: %s' % msg)


def main(args=None):
    rclpy.init(args=args)
    subscriber_node = SubscriberNode()
    rclpy.spin(subscriber_node)
    subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
