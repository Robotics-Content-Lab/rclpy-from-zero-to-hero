from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

import xacro, os

def generate_launch_description():
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')

    xacro_file_name = 'robotino_platform_kinect.urdf.xacro'
    xacro_file = os.path.join(
        get_package_share_directory('robotino_description'),
        'urdf', xacro_file_name
    )
    robot_description_config = xacro.process_file(xacro_file)
    robot_desc = robot_description_config.toxml()
    model_ns = 'robotino'

    spawn_robotino = Node(
            package='robotino_bringup',
            executable='spawn_robotino',
            arguments=[robot_desc, model_ns],
            output='screen'
        )


    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzserver.launch.py')
            ),
            launch_arguments={'world': '',
                              'verbose': 'true',
                              'extra_gazebo_args': 'verbose'}.items()
        ),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_gazebo_ros, 'launch', 'gzclient.launch.py')
            ),
            launch_arguments={'verbose': 'true'}.items()
        ),
        spawn_robotino
    ])