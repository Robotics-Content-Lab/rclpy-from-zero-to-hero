#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
def generate_launch_description():
    return LaunchDescription([
        Node(
            package='preface',
            executable='namespace_example1',
            name='robot1',
            output='screen'
        ),
        Node(
            package='preface',
            executable='namespace_example1',
            name='robot2',
            namespace='robot2_namespace',
            output='screen'
        ),
        Node(
            package='preface',
            executable='namespace_example2',
            name='robot11',
            output='screen'
        ),
        Node(
            package='preface',
            executable='namespace_example2',
            name='robot22',
            namespace='robot2_namespace',
            output='screen'
        )
    ])