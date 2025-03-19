import rclpy
from rclpy.node import Node


# Our class inherits from the rclpy.node.Node class
class FirstNode(Node):
    def __init__(self) -> None:
        # we call the constructor of the parent class in order to initialize the Node object
        # the parameter we pass is the node name
        super().__init__('hagrid')
        # Create a timer that calls timer_callback every second
        # - the self.timer_callback function is 1.0/second executed 
        self.timer = self.create_timer(timer_period_sec=1.0, callback=self.timer_callback)

    def timer_callback(self) -> None:
        # We use the get_logger() method of the parent class to
        # Log a message to the info stream
        self.get_logger().info('You are a wizard Harry')


# The main method will be used as entrypoint by ROS 2
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # Initialize an instance of our FirstNode class
    node = FirstNode()
    # Spin the node and wait for events
    rclpy.spin(node)
    # Clean up the node, Frees resources used by the node
    node.destroy_node()
    # Shutdown the ROS 2 client library
    rclpy.shutdown()


# Entrypoint of the script in case we start it via Python
if __name__ == '__main__':
    main()