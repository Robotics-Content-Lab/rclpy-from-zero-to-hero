from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Path to the turtlebot3_gazebo launch file
    turtlebot3_gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'empty_world.launch.py'
    )
    # Launch file include
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_gazebo_launch_file),)

    # Publisher node
    publisher_node = Node(
        package='hw1_pkg',
        executable='publisher_node',
        name='publisher_node'
    )

    # Subscriber node
    subscriber_node = Node(
        package='hw1_pkg',
        executable='subscriber_node',
        name='subscriber_node'
    )

    # Launch description
    return LaunchDescription([
        launch_file,
        publisher_node,
        subscriber_node,
    ])
