#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='preface',
            executable='params',
            name='diff_drive_controller_default',
            parameters=[
                {'wheel_radius': 0.5},
                {'baseline_distance': 1.9},
                {'max_linear_velocity': 0.6},
                {'max_angular_velocity': 1.0}
            ],
            output='screen'
        ),
        Node(
            package='preface',
            executable='params',
            name='differential_drive_node',
            parameters=['/home/ros_user/ros2_ws/src/preface/params.yaml'],
            output='screen'
        ),
    ])