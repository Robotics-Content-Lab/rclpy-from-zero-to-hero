from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import os

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory('sjtu_drone_bringup'), 'launch', 'sjtu_drone_gazebo.launch.py'),
            ),
            launch_arguments={'world': 'empty'}.items(),
        ),
        Node(
            package="ch5",
            executable="drone_mission_node",
            name="drone_mission_node",
            output="screen",
            prefix="xterm -e"
        ),
        Node(
            package="ch5",
            executable="drone_mission_node_server",
            name="drone_mission_node_server",
            output="screen",
            prefix="xterm -e"
        )
    ])