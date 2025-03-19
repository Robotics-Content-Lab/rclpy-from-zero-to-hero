import rclpy
from rclpy.node import Node
from example_interfaces.msg import String


class SimplePublisher(Node):
    def __init__(self):
        super().__init__('simple_publisher')
        self.publisher_ = self.create_publisher(String, '/example_topic', 10)
        self.timer = self.create_timer(1.0, self.publish_message)  # Publish every 1 second
        self.counter = 0
        self.get_logger().info('Publisher Node has been started.')

    def publish_message(self):
        msg = String()
        msg.data = f"Hello, ROS 2! Count: {self.counter}"
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data}")
        self.counter += 1


def main(args=None):
    rclpy.init(args=args)
    node = SimplePublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Publisher Node stopped.')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
