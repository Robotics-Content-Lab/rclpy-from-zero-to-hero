from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ch1',
            executable='first_publisher',
            name='snape',
            remappings=[('/points', '/house_points')],
            output='screen'
        ),
        Node(
            package='ch1',
            executable='first_subscriber',
            name='harry',
            remappings=[('/some_points', '/house_points')],
            output='screen'
        )
    ])