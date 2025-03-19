from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='empty_world',
        choices=['empty_world', 'turtlebot3_world']
    )

    ch2_simulation_launch_path = PathJoinSubstitution([
        FindPackageShare('ch2'),
        'launch',
        'simulation.launch.py'
    ])
    ch2_simulation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ch2_simulation_launch_path),
        launch_arguments={'world_file': LaunchConfiguration('world_file')}.items()
    )

    return LaunchDescription([
        world_file_arg,
        ch2_simulation_launch,
    ])