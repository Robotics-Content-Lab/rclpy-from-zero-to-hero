import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    bag_path = os.path.join(get_package_share_directory('ch4'), 'bag', 'square', 'square.mcap')
    return LaunchDescription([
        # Node(
        #     package='hw4_pkg',
        #     executable='ekf_holonomic',
        #     name='ekf_holonomic',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     prefix="xterm -hold -e",
        #     remappings=[('/cmd_vel', '/robotino/cmd_vel'), ('/imu', '/robotino/imu/out')]
        # ),
        ExecuteProcess(
            cmd=['xterm', '-hold', '-e', f'ros2 bag play {bag_path} --start-paused --rate 1.0'],
            output='screen',
            shell=True
        ),
        Node(
            package='ch4',
            executable='odometry_comparison',
            name='odometry_comparison',
            remappings=[('/odom', '/robotino/odom')],
            output='screen'
        )
    ])
