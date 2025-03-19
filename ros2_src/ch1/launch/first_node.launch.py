from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ch1',
            executable='first_node',
            name='first_node',
            output='screen'
        )
    ])