import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration



def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    map_path = os.path.join(get_package_share_directory('project'), 'map', 'map.yaml')
    delcare_map_cmd = DeclareLaunchArgument(
            'map',
            default_value=map_path,
            description='Full path to map file to load')

    param_file_path = os.path.join(get_package_share_directory('project'), 'config', 'nav2_params.yaml')
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=param_file_path,
        description='Full path to the ROS 2 parameters file to use for all launched nodes')

    nav2_launch_dir = os.path.join(get_package_share_directory('nav2_bringup'), 'launch')

    nav2_navigation_path = os.path.join(nav2_launch_dir, 'navigation_launch.py')

    nav2_localization_path = os.path.join(nav2_launch_dir, 'localization_launch.py')

    return LaunchDescription([
        declare_use_sim_time_cmd,
        delcare_map_cmd,
        declare_params_file_cmd,
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(nav2_navigation_path),
        #     launch_arguments={
        #         'map': map_path,
        #         'use_sim_time': use_sim_time,
        #         'params_file': param_file_path}.items(),
        #     ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_localization_path),
            launch_arguments={
                'map': map_path,
                'use_sim_time': use_sim_time,
                'params_file': param_file_path}.items(),
            ),
    ])
