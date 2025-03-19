import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    bag_path = os.path.join(get_package_share_directory('ekf_localization'), 'bags', '1', 'bag.mcap')
    return LaunchDescription([
        ExecuteProcess(
            cmd=['xterm', '-hold', '-e', f'ros2 bag play {bag_path} --start-paused --rate 1.0'],
            output='screen',
            shell=True
        ),
    ])
