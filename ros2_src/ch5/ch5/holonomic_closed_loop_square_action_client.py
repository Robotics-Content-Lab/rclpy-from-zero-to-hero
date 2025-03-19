import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from ch5_interfaces.action import DriveSquareAction  # Import the custom action type we defined earlier

class DriveSquareActionClient(Node):
    def __init__(self):
        super().__init__('drive_square_action_client')  # Call the parent class constructor and name the node

        # Create an action client
        self.action_client = ActionClient(self, DriveSquareAction, 'drive_square')  
        # Create an action client for the 'DriveSquareAction' action type, which will interact with the 'drive_square' action server
        
        self.get_logger().info('Waiting for action server...')
        self.action_client.wait_for_server()  
        # Wait until the action server is available
        
        self.get_logger().info('Action server available. Ready to send goal.')  
        # Log a message indicating the action server is available and the client is ready to send goals

    def send_goal(self, side_length, repetitions, time_limit):  
        # Define a method to send a goal to the action server
        goal_msg = DriveSquareAction.Goal()  # Create a goal message object of type 'DriveSquareAction.Goal'
        goal_msg.side_length = side_length  # Set the 'side_length' field of the goal to the desired square side length
        goal_msg.repetitions = repetitions  # Set the 'repetitions' field of the goal to the desired number of repetitions
        goal_msg.time_limit = time_limit  # Set the 'time_limit' field of the goal to the desired time limit
        
        self.get_logger().info(f'Sending goal: Drive in square with side {side_length} meters, repeated {repetitions} times, with a time limit of {time_limit} seconds.')
        # Log a message indicating that the goal has been sent, including the square side length, repetitions, and time limit
        
        # Send the goal asynchronously to the action server and register a callback for the feedback and goal response
        self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        ).add_done_callback(self.goal_response_callback)
    
    def feedback_callback(self, feedback_msg):
        # Callback function to handle feedback from the action server
        feedback = feedback_msg.feedback  # Extract the feedback from the feedback message
        self.get_logger().info(f'Feedback msg:\nCurrent iteration: {feedback.current_iteration}, Current side: {feedback.current_side}')  
        # Log the current iteration and side based on the feedback received

    def goal_response_callback(self, future):
        # Callback function to handle the response from the action server regarding the goal
        goal_handle = future.result()  # Retrieve the goal handle from the future
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected by the server.')  # Log if the goal was rejected by the server
            return

        self.get_logger().info('Goal accepted by the server.')  # Log if the goal was accepted by the server
        
        # Register a callback to handle the result from the action server
        goal_handle.get_result_async().add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Callback function to handle the result from the action server
        result = future.result().result  # Retrieve the result from the future
        if result.success:
            self.get_logger().info('Goal succeeded: Robot completed the square driving pattern.')  # Log if the goal succeeded
        else:
            self.get_logger().info('Goal failed: Could not complete the square driving pattern.')  # Log if the goal failed

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client library
    action_client = DriveSquareActionClient()  # Instantiate the action client node

    # Send the goal with desired square size, number of repetitions, and time limit
    action_client.send_goal(side_length=2.0, repetitions=2, time_limit=120)  
    # Send a goal to the server, asking the robot to drive in a square with 2.0m sides, repeated 2 times, with a time limit of 120 seconds

    rclpy.spin(action_client)  # Spin the node to handle callbacks
    
    action_client.destroy_node()  # Destroy the node to clean up resources
    rclpy.shutdown()  # Shutdown the ROS 2 Python client library

if __name__ == '__main__':  # Standard Python entry point
    main()  # Call the main function to run the action client
