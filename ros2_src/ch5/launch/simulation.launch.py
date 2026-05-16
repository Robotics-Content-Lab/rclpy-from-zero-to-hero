"""Launch Gazebo Harmonic with a Robotino robot (ch5)."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Return the ch5 simulation launch description."""
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    xacro_file = os.path.join(
        get_package_share_directory('robotino_description'),
        'urdf', 'robotino_platform_kinect.urdf.xacro'
    )
    robot_desc = xacro.process_file(xacro_file).toxml()
    model_ns = 'robotino'  # no leading slash per Harmonic convention

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    spawn_robotino = Node(
        package='robotino_bringup',
        executable='spawn_robotino',
        arguments=[robot_desc, model_ns],
        output='screen',
    )

    return LaunchDescription([gz_sim, clock_bridge, spawn_robotino])
