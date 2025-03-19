#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    preface_path = get_package_share_directory('preface')
    turtlebot3_gazebo_path = get_package_share_directory('turtlebot3_gazebo')

    config_path = os.path.join(
        preface_path, "rviz", "config.rviz"
    )
    

    return LaunchDescription([
        IncludeLaunchDescription(PythonLaunchDescriptionSource(
                os.path.join(turtlebot3_gazebo_path, 'launch', 'turtlebot3_world.launch.py')
        )),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", config_path
            ],
            output="screen",
        ),
    ])
