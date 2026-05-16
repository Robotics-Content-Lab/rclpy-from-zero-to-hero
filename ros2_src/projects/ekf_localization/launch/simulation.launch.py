"""Launch Gazebo Harmonic with TurtleBot3 and EKF localization landmarks."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Return the ekf_localization simulation launch description."""
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    turtlebot3_launch_dir = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'), 'launch'
    )

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='0.0')
    y_pose = LaunchConfiguration('y_pose', default='0.0')

    # turtlebot3_gazebo's empty_world SDF — world name inside is "default"
    world = os.path.join(
        get_package_share_directory('turtlebot3_gazebo'),
        'worlds',
        'empty_world.world'
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    robot_state_publisher_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir,
                         'robot_state_publisher.launch.py')
        ),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    spawn_turtlebot_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(turtlebot3_launch_dir, 'spawn_turtlebot3.launch.py')
        ),
        launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()
    )

    landmark_spawner = Node(
        package='ekf_localization',
        name='landmark_spawner',
        executable='landmark_spawner',
    )

    teleop = Node(
        package='turtlebot3_teleop',
        name='teleop',
        executable='teleop_keyboard',
        prefix='xterm -e',
    )

    ld = LaunchDescription()
    ld.add_action(gz_sim)
    ld.add_action(clock_bridge)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(landmark_spawner)
    ld.add_action(teleop)
    return ld
