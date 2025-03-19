import rclpy
from rclpy.node import Node


class TimerTestClass(Node):
    def __init__(self):
        super().__init__('timer_test')
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Hello, World!')


def main():
    rclpy.init()
    node = TimerTestClass()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()