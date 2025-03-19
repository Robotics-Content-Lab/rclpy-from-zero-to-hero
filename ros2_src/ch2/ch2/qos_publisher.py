#!/usr/bin/python3
# Import the necessary libraries
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Float32
from rclpy.qos import QoSProfile, ReliabilityPolicy  # We import the QoSProfile and ReliabilityPolicy classes

class QoSPublisher(Node):
    """QoSPublisher class that uses parameters to define the QoS profile"""
    def __init__(self):
        super().__init__('qos_publisher')
        # We create a QoSProfile object with a depth of 10 and a reliability policy of BEST_EFFORT
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.publisher = self.create_publisher(Float32, 'topic', qos_profile) # We pass the QoSProfile object to the create_publisher method
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.msg_count = 0

    def publish_msg(self):
        msg = Float32()
        msg.data = float(self.msg_count)
        self.publisher.publish(msg)
        self.get_logger().info(f'Publishing: {msg.data}')
        self.msg_count += 1

def main(args=None):
    rclpy.init(args=args)
    simple_publisher = QoSPublisher()
    rclpy.spin(simple_publisher)
    simple_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
