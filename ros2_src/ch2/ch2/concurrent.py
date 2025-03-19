import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import LaserScan
from example_interfaces.msg import String

class DataProcessor(Node):
    def __init__(self):
        super().__init__('data_processor')
        
        self.rate = self.create_rate(1)
        # Callback groups
        self.sensor_data_cg = ReentrantCallbackGroup()  # Callback group for sensor data
        self.command_cg = ReentrantCallbackGroup()  # Callback group for user commands

        # Subscriptions
        self.sensor_sub = self.create_subscription(
            LaserScan,
            'scan',
            self.sensor_callback,
            10,
            callback_group=self.sensor_data_cg
        )

        self.command_sub = self.create_subscription(
            String,
            'user_command',
            self.command_callback,
            10,
            callback_group=self.command_cg
        )

        # Publisher
        self.processed_data_pub = self.create_publisher(String, 'processed_data', 10)

        # Processing state
        self.processing_setting = 'default'

    def sensor_callback(self, msg: LaserScan):
        self.get_logger().info(f'Processing sensor data with setting: {self.processing_setting}')
        # Simulate data processing delay
        self.rate.sleep()

        processed_data = f'Processed msg @ {msg.header.stamp} with setting {self.processing_setting}'
        self.processed_data_pub.publish(String(data=processed_data))

    def command_callback(self, msg):
        self.get_logger().info(f'Received command to change processing setting to: {msg.data}')
        self.processing_setting = msg.data

def main(args=None):
    rclpy.init(args=args)
    node = DataProcessor()
    executor = MultiThreadedExecutor()  # Create a multi-threaded executor
    executor.add_node(node)  # Add the node to the executor
    
    try:
        executor.spin()  #Spin the executor
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
