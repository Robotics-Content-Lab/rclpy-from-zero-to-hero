import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown

import xacro


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    use_gui = DeclareLaunchArgument('use_gui', default_value='true', choices=['true', 'false'],
                                    description='Whether to execute gzclient')
    x_pos_arg = DeclareLaunchArgument(
        'x_pos', default_value='-0.55',
        description='Initial x position of the robot'
    )
    y_pos_arg = DeclareLaunchArgument(
        'y_pos', default_value='6.2',
        description='Initial y position of the robot'
    )
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='-90.0',
        description='Initial yaw of the robot'
    )
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    yaw = LaunchConfiguration('yaw')

    rviz_conf_path = os.path.join(
        get_package_share_directory('project'), 'config', 'rviz.rviz'
    )
    xacro_file = os.path.join(
        get_package_share_directory('project'),
        'urdf',
        'turtlebot3_burger_camera.xacro'
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()

    world_file = os.path.join(
        get_package_share_directory('project'),
        'worlds', 'project.world'
    )

    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    def launch_gzclient(context, *args, **kwargs):
        if context.launch_configurations.get('use_gui') == 'true':
            return [IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
                ),
                launch_arguments={'verbose': 'true', 'use_sim_time': use_sim_time}.items()
            )]
        return []

    return LaunchDescription([
        declare_use_sim_time_cmd,
        use_gui,
        x_pos_arg,
        y_pos_arg,
        yaw_arg,
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            arguments=[robot_desc]
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{'use_sim_time': use_sim_time, 'robot_description': robot_desc}],
            output='screen',
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': world_file,
                              'verbose': 'true',
                              'extra_gazebo_args': 'verbose',
                              'use_sim_time': use_sim_time}.items()
        ),

        OpaqueFunction(function=launch_gzclient),

        Node(
            package='project',
            executable='spawn_robot',
            arguments=[robot_desc],
            parameters=[{'x_pos': x_pos, 'y_pos': y_pos, 'yaw': yaw}],
            output='screen'
        ),

        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            arguments=[
                "-d", rviz_conf_path,
                '--ros-args', '--log-level', "warn"
                ],
            on_exit=Shutdown()
        ),
    ])