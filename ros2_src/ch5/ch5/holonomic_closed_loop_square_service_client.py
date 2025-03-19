import rclpy
from rclpy.node import Node
from ch5_interfaces.srv import DriveInSquare  # Import the custom service type we defined earlier

class HolonomicClosedLoopSquareClient(Node):
    def __init__(self):
        super().__init__('holonomic_closed_loop_square_client')  # Call the parent class constructor and name the node

        # Create a service client
        self.client = self.create_client(DriveInSquare, 'start_square')  
        # Create a client for the 'DriveInSquare' service type, which will interact with the 'start_square' service
        
        # Wait until the service becomes available
        while not self.client.wait_for_service(timeout_sec=5.0):  
            # Loop until the service is available, checking every 5 seconds
            self.get_logger().info('Waiting for the service to be available...')  
            # Log a message each time the client checks for service availability
        
        self.get_logger().info('Service is available, ready to send request')  
        # Log a message indicating the service is available and the client is ready to send requests

        # Define the request
        self.request = DriveInSquare.Request()  
        # Create a request object of type 'DriveInSquare.Request' to hold the data that will be sent to the server

    def send_request(self, square_side, times):  # Define a method to send a request to the service server
        self.request.length = square_side  # Set the 'length' field of the request to the desired square side length
        self.request.times = times  # Set the 'times' field of the request to the desired number of repetitions
        self.future = self.client.call_async(self.request)  
        # Send the request asynchronously to the service server and get a future object to track the response
        
        self.get_logger().info(f'Sending request to drive in square: side={square_side}, times={times}')  
        # Log a message indicating that the request has been sent, including the square side and times

        rclpy.spin_until_future_complete(self, self.future)  
        # Block the node until the future is complete, meaning until the server responds

        if self.future.result() is not None:  
            # Check if the future has a result, meaning the server responded successfully
            self.get_logger().info('Service call succeeded: Robot is driving in a square.')  
            # Log a success message if the service call was successful
        else:
            self.get_logger().error('Service call failed!')  
            # Log an error message if the service call failed

def main(args=None):  # Define the main function to start the client node
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    node = HolonomicClosedLoopSquareClient()  # Instantiate the client node

    # Send the request with desired square size and number of times to repeat
    node.send_request(square_side=2.0, times=2)  
    # Send a request to the server, asking the robot to drive in a square with 2.0m sides, repeated 2 times

    node.destroy_node()  # Destroy the node to clean up resources
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library

if __name__ == '__main__':  # Standard Python entry point
    main()  # Call the main function to run the client

