from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import LaunchConfiguration
import os


def generate_launch_description():
    config_path = os.path.join(get_package_share_directory('hw2_pkg'), 'config.rviz')
    min_samples = LaunchConfiguration('min_samples')
    dist_thresh = LaunchConfiguration('dist_thresh')

    # Declare the 'reliability_policy' launch argument with choices for validation
    min_samples_arg = DeclareLaunchArgument(
        'min_samples',
        default_value="20",
        description='The number of samples in a neighborhood for a point to be considered as a core point'
    )
    dist_thresh_arg = DeclareLaunchArgument(
        'dist_thresh',
        default_value="0.1",
        description='The maximum distance between two samples for one to be considered as in the neighborhood of the other'
    )
    
    # Path to the turtlebot3_gazebo launch file
    turtlebot3_gazebo_launch_file = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    )
    # Launch file include
    launch_file = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_gazebo_launch_file),)

    # Publisher node
    sensor_proc = Node(
        package='hw2_pkg',
        executable='sensor_proc',
        name='sensor_proc',
        parameters=[{'min_samples': min_samples, 'dist_thresh': dist_thresh}]
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', config_path]
    )

    # Launch description
    return LaunchDescription([
        min_samples_arg,
        dist_thresh_arg,
        rviz,
        launch_file,
        sensor_proc
    ])
