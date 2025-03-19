from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare the reliability policy argument
    reliability_policy_arg = DeclareLaunchArgument(
        'reliability_policy',
        default_value='best_effort',
        choices=['best_effort', 'reliable']
    )

    publisher = Node(
        package='ch2',
        executable='qos_publisher',
        name='pub',
        output='screen'
    )

    # Define the subscriber node, including the 'reliability_policy' parameter dynamically set
    subscriber = Node(
        package='ch2',
        executable='qos_subscriber',
        name='sub',
        output='screen',
        parameters=[{
            'reliability_policy': LaunchConfiguration('reliability_policy')  # LaunchConfiguration will substitute the value of the 'reliability_policy' argument
        }]
    )

    # Create and return the launch description with the declared arguments and nodes
    return LaunchDescription([
        reliability_policy_arg,
        publisher,
        subscriber
    ])
