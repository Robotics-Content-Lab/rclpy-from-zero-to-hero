#!/usr/bin/python3
import rclpy  # Import the rclpy module
from rclpy.node import Node  # Import the Node class from rclpy
from example_interfaces.msg import String  # Import the String message from example_interfaces
from rclpy.qos import qos_profile_system_default  # Import the qos_profile_system_default quality of service profile

class PointsPublisher(Node):  # Define a class named PointsPublisher that inherits from Node
    def __init__(self):  # Define the constructor method
        super().__init__('dumbledore')  # Call the constructor of the parent class
        self.points = 0  # Initialize the points variable to 0
        self.publisher = self.create_publisher(msg_type=String, topic='/points', qos_profile=qos_profile_system_default)  # Create a publisher that publishes messages of type String on the '/points' topic
        # The qos_profile argument defines the quality of service settings for the publisher. Roughly speaking, it determines what happens if the publisher is publishing messages faster than the subscriber can process them.
        # The qos_profile_system_default is a predefined quality of service profile that is suitable for most of our use cases.
        # We will learn more about quality of service profiles in the next chapter.
        self.timer = self.create_timer(1.0, self.publish_points)  # Create a timer that calls the publish_points method every 1 second

    def publish_points(self):  # Define the publish_points method
        self.points += 10  # Increment the points variable by 10
        message = String()  # Create an instance of the String message
        message.data = f'{self.points} points for Gryffindor'  # Set the data field of the message to a formatted string
        self.publisher.publish(message)  # Publish the message

def main(args=None):  # Define the main function
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    publisher = PointsPublisher()  # Create an instance of the PointsPublisher class
    rclpy.spin(publisher)  # Enter a loop to process ROS 2 events
    publisher.destroy_node()  # Destroy the publisher node
    rclpy.shutdown()  # Shutdown the ROS 2 client library

if __name__ == '__main__':  # Check if the script is being run directly
    main()  # Call the main function
