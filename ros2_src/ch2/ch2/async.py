#!/usr/bin/env python3
import time

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from concurrent.futures import ThreadPoolExecutor


class CoroutineNode(Node):
    def __init__(self):
        super().__init__('async_task_node')
        self.thread = ThreadPoolExecutor(max_workers=1)  # Create a thread pool with 1 threads
        self.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        self.get_logger().info('Starting asynchronous task')
        future = self.thread.submit(self.long_running_task)  # Submit the long-running task to the thread pool
        future.add_done_callback(self.task_done_callback)  # Add a callback to the future object to handle the result

    def long_running_task(self):
        start_time = self.get_clock().now().to_msg()
        start_time_str = time.strftime('%H:%M:%S', time.localtime(start_time.sec + start_time.nanosec / 1e9))
        self.get_logger().info(f'Task started at {start_time_str}')
        time.sleep(5)  # Simulate long-running operation
        fin_time = self.get_clock().now().to_msg()
        duration = (fin_time.sec + fin_time.nanosec / 1e9) -( start_time.sec - start_time.nanosec / 1e9)
        return f'Task started at {start_time_str} and finished at {duration}'

    def task_done_callback(self, future):
        result = future.result()
        self.get_logger().info(f'Asynchronous task completed with result: {result}')

def main(args=None):
    rclpy.init(args=args)
    node = CoroutineNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    node.thread.shutdown()
    node.executor.shutdown()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()