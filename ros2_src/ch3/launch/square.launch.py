from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def get_control_node(context, *args, **kwargs) -> Node:
    control_strategy = LaunchConfiguration('control_strategy').perform(context)
    square_side = float(LaunchConfiguration('square_side').perform(context))
    square_times = int(LaunchConfiguration('square_times').perform(context))
    if control_strategy == 'open-loop':
        return [Node(
            package='ch3',
            executable='open_loop_square',
            name='open_loop_square',
            parameters=[{'square_side': square_side, 'square_times': square_times}],
            output='screen'
        )]
    elif control_strategy == 'closed-loop':
        return [Node(
            package='ch3',
            executable='closed_loop_p_square',
            name='closed_loop_p_square',
            parameters=[{'square_side': square_side, 'square_times': square_times}],
            output='screen'
        )]


def generate_launch_description():
    control_strategy_arg = DeclareLaunchArgument(
        'control_strategy',
        default_value='open-loop',
        choices=['open-loop', 'closed-loop']
    )
    square_side_arg = DeclareLaunchArgument(
        'square_side',
        default_value='1.0',
        description='Side length of the square'
    )
    square_times_arg = DeclareLaunchArgument(
        'square_times',
        default_value='1',
        description='Number of times the robot should go around the square'
    )

    simulation = PathJoinSubstitution([
        FindPackageShare('ch2'),
        'launch',
        'simulation.launch.py'
    ])

    launch_file_include = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(simulation)
    )

    return LaunchDescription([
        control_strategy_arg,
        square_side_arg,
        square_times_arg,
        launch_file_include,
        OpaqueFunction(function=get_control_node)
    ])