from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    publisher = Node(
        package='ch1',
        executable='first_publisher',
        name='doumbledore',
        output='screen'
    )
    subscriber = Node(
        package='ch1',
        executable='first_subscriber',
        name='ron',
        output='screen'
    )
    
    return LaunchDescription([
        publisher,
        subscriber
    ])