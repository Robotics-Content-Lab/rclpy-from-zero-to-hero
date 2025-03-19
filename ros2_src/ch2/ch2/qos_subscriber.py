#!/usr/bin/python3
# Import the necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy  # We import the QoSProfile and ReliabilityPolicy classes

class QoSSubscriber(Node):
    def __init__(self):
        """QoSSubscriber class that uses parameters to define the QoS profile"""
        super().__init__('qos_subscriber')
        self.declare_parameter('reliability_policy', 'reliable')  # We declare a parameter called reliability_policy with a default value of reliable
        reliability_policy = self.get_parameter('reliability_policy').value  # We get the value of the reliability_policy parameter
        # We check the value of the reliability_policy parameter and set the reliability policy accordingly
        if reliability_policy == 'reliable':
            reliability = ReliabilityPolicy.RELIABLE
            self.get_logger().info('Reliability policy: RELIABLE')
        else:
            reliability = ReliabilityPolicy.BEST_EFFORT
            self.get_logger().info(f'Reliability policy: {reliability_policy}')
        # We create a QoSProfile object with a depth of 10 and a reliability policy of RELIABLE
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
