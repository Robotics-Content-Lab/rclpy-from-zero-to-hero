#!/usr/bin/python3
# Import the necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy  # We import the QoSProfile and ReliabilityPolicy classes

class QoSSubscriber(Node):
    def __init__(self):
        super().__init__('qos_subscriber2')
        reliability = ReliabilityPolicy.BEST_EFFORT  # We set the reliability policy to BEST_EFFORT
        # We create a QoSProfile object with a depth of 10 and a reliability policy of BEST_EFFORT
        qos_profile = QoSProfile(depth=10, reliability=reliability)
        self.subscription = self.create_subscription(Float32, 'topic', self.listener_callback, qos_profile)

    def listener_callback(self, msg: Float32):
        self.get_logger().info(f'Heard: {msg.data}')

def main(args=None):
    rclpy.init(args=args)
    simple_subscriber = QoSSubscriber()
    rclpy.spin(simple_subscriber)
    simple_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
