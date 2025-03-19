import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    save_fig = DeclareLaunchArgument('save_fig', default_value='true', choices=['true', 'false'], description='Save the pyplot figure')    
    
    simulation = IncludeLaunchDescription(
        os.path.join(get_package_share_directory('ekf_localization'),
                    'launch',
                    'simulation.launch.py')
    )

    ekf_node = Node(
        package="ekf_localization",
        name="known_correspondences",
        executable="ekf_localization_with_known_correspondences",
        output="screen",
        prefix=["xterm -hold -e"],
        parameters=[
            {"save_fig": LaunchConfiguration('save_fig')}
        ]
    )

    rviz = Node(
        package="rviz2",
        name="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(get_package_share_directory('ekf_localization'), 'config', 'rviz.rviz')]
    )

    return LaunchDescription([
        save_fig,
        simulation,
        ekf_node,
        rviz
    ])