from launch import LaunchDescription
from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare('ch2'),
                'launch',
                'simulation.launch.py'
            ]))
        ),
        Node(
            package='hw3_pkg',
            executable='trajectory_controller',
            name='trajectory_controller',
            output='screen'
        ),
        Node(
            package='hw3_pkg',
            executable='odometry',
            name='odometry',
            output='screen'
        )
    ])
