#!/usr/bin/python3
import rclpy  # Import the rclpy module
from rclpy.node import Node  # Import the Node class from rclpy
# ROS 2 requires the use of the QoS (Quality of Service) settings to determine how messages are sent and received. We will learn more about that in a later class.
from rclpy.qos import qos_profile_system_default  # Import the qos_profile_system_default object from the rclpy.qos module
from example_interfaces.msg import String  # Import the String message from example_interfaces

class PointsSubscriber(Node):  # Define a class named PointsSubscriber that inherits from Node
    def __init__(self):  # Define the constructor method
        super().__init__('ron')  # Call the constructor of the parent class to intialize the Node object
        self.msg_cnt = 0  # Create a counter variable inside the class
        self.subscriber = self.create_subscription(String, '/some_points', self.callback, qos_profile_system_default)  # Create a subscriber that subscribes to messages of type String on the '/points' topic
        # Subscribers requrie a callback function that is called when a message is received. In this case the callback function is called self.callback method of our class.
        # Further the subscribers qos_profile needs to match the publishers qos_profile, otherwise the subscriber will not receive any messages.

    def callback(self, msg: String):  # Define the callback method that is called everytime a message is received on the '/points' topic
        self.get_logger().info(f"I heard msg #{self.msg_cnt}: {msg.data}")  # Log the message to the info stream
        self.msg_cnt += 1  # Increment the message counter


def main(args=None):  # Define the main function 
    rclpy.init(args=args)  # Initialize the ROS 2 client library
    publisher = PointsSubscriber()  # Create an instance of the PointsSubscriber class
    rclpy.spin(publisher)  # Enter a loop to process ROS 2 events
    publisher.destroy_node()  # Destroy the publisher node
    rclpy.shutdown()  # Shutdown the ROS 2 client library


if __name__ == '__main__':  # Check if the script is being run directly
    main()  # Call the main function   