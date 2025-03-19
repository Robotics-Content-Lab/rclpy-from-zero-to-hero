#!/usr/bin/python3
# Import the necessary libraries
import rclpy
from rclpy.node import Node 
from sensor_msgs.msg import LaserScan  # Import the LaserScan message type


class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('blocking_node')
        self.msg_cnt = 0
        self.declare_parameter('rate', 1.0)  # We define a ROS 2 parameter, which is a variable that can be set from the command line, launch file, or another node
        self.rate = self.create_rate(self.get_parameter('rate').value)  # We create a rate object that will be used to control the timing of the sleep function
        self.subscriber = self.create_subscription(LaserScan, '/scan', self.callback, 10)

    def callback(self, msg: LaserScan):
        self.get_logger().info(f"I received msg #{self.msg_cnt}")
        self.msg_cnt += 1
        # self.rate.sleep() is mimicking the time it takes to process the message, e.g. during SLAM
        self.rate.sleep()
        

def main(args=None):
    rclpy.init(args=args)
    subscriber = ScanSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()