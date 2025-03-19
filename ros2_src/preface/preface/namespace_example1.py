import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ExampleNode(Node):

    def __init__(self):
        super().__init__('example_node')
        self.get_logger().info(f"Node name: {self.get_name()}")
        self.get_logger().info(f"Node running in namespace: {self.get_namespace()}")
        sub = self.create_subscription(String, '~/topic', lambda msg: self.get_logger().info(f"Received: {msg.data}"), 10)
        pub = self.create_publisher(String, '~/topic', 10)
        self.get_logger().info(f"Subscribed to topic: {sub.topic_name}")
        self.get_logger().info(f"Publishing to topic: {pub.topic_name}")


def main(args=None):
    rclpy.init(args=args)
    node = ExampleNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()