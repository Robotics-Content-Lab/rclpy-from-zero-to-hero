#!/usr/bin/env python3
import time

import rclpy
import rclpy.callback_groups
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String

class LongRunningComputationNode(Node):
    def __init__(self):
        super().__init__('long_running_computation_node')
        cb = rclpy.callback_groups.ReentrantCallbackGroup()
        self.subscription = self.create_subscription(
            String,
            'chatter',
            self.listener_callback,
            10,
            callback_group=cb)
        self.timer = self.create_timer(1.0, self.timer_callback, callback_group=cb)
        self.pub = self.create_publisher(String, 'chatter', 10)
        self.computation_complete = False
        self.get_logger().info('Node initialized, starting long-running computation')
        self.long_running_computation()

    def long_running_computation(self):
        # Simulate a long-running computation
        for i in range(10):
            # Do some work (simulate with sleep)
            time.sleep(1)
            self.get_logger().info(f'Computation step {i+1} completed')
        self.computation_complete = True
        self.get_logger().info('Long-running computation completed')

    def listener_callback(self, msg):
        self.get_logger().info(f'Received message: {msg.data}')

    def timer_callback(self):
        self.pub.publish(String(data='Hello, world!'))
        if not self.computation_complete:
            self.get_logger().info('Timer callback executed during computation')
        else:
            self.get_logger().info('Timer callback executed after computation')
            self.destroy_node()
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = LongRunningComputationNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()

if __name__ == '__main__':
    main()
