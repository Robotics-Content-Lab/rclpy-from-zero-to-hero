import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription, LaunchContext
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import RegisterEventHandler
from launch.events.process import ProcessStarted
from launch.event_handlers.on_process_start import OnProcessStart


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    
    project_dir = get_package_share_directory('project')

    simulation_launch_file_path = os.path.join(project_dir, 'launch', 'simulation.launch.py')
    simulation_launch_file = PythonLaunchDescriptionSource(simulation_launch_file_path)
    launch_simulation = IncludeLaunchDescription(simulation_launch_file, launch_arguments={'use_sim_time': use_sim_time}.items())
    
    nav2_launch_file_path = os.path.join(project_dir, 'launch', 'nav2.launch.py')
    nav2_launch_file = PythonLaunchDescriptionSource(nav2_launch_file_path)
    launch_nav2 = IncludeLaunchDescription(nav2_launch_file, launch_arguments={'use_sim_time': use_sim_time}.items())

    def start_nav2(context: LaunchContext):
        return [launch_nav2]

    return LaunchDescription([
        declare_use_sim_time_cmd,
        RegisterEventHandler(
            OnProcessStart(
                target_action=launch_simulation,
                on_start=lambda event, context: context.emit_event(
                    OpaqueFunction(function=start_nav2)
                )
            )
        ),
        launch_simulation
    ])
