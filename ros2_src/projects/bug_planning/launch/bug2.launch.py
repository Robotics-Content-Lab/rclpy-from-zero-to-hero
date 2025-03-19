#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time", default="false")
    rviz_config_dir = os.path.join(
        get_package_share_directory('bug_planning'),
        'config',
        'config.rviz')
    print("---------------"*8)
    print(f"{rviz_config_dir=}")
    print("---------------"*8)
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_turtlebot3_gazebo, 'launch', 'turtlebot3_house.launch.py'),
            )
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=["-d", rviz_config_dir]
        ),
    ])
