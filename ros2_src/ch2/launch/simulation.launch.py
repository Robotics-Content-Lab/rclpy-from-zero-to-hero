from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PythonExpression
from launch.conditions import IfCondition


def generate_launch_description():
    world_file_arg = DeclareLaunchArgument(
        'world_file',
        default_value='empty_world',
        choices=['empty_world', 'turtlebot3_world']
    )

    empty_world_launch_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'launch',
        'empty_world.launch.py'
    ])

    turtlebot3_world_launch_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_gazebo'),
        'launch',
        'turtlebot3_world.launch.py'
    ])

    launch_file_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(empty_world_launch_path),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('world_file'), '" == "empty_world"']))
    )

    turtlebot3_launch_file_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(turtlebot3_world_launch_path),
        condition=IfCondition(PythonExpression(['"', LaunchConfiguration('world_file'), '" == "turtlebot3_world"']))
    )

    return LaunchDescription([
        world_file_arg,
        launch_file_include,
        turtlebot3_launch_file_include
    ])