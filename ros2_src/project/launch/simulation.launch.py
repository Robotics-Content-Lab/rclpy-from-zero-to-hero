"""Launch Gazebo Harmonic with the project TurtleBot3 + camera world."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
import xacro


def generate_launch_description():
    """Return the project simulation launch description."""
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')
    x_pos_arg = DeclareLaunchArgument(
        'x_pos', default_value='-0.55',
        description='Initial x position of the robot')
    y_pos_arg = DeclareLaunchArgument(
        'y_pos', default_value='6.2',
        description='Initial y position of the robot')
    yaw_arg = DeclareLaunchArgument(
        'yaw', default_value='-90.0',
        description='Initial yaw of the robot')
    x_pos = LaunchConfiguration('x_pos')
    y_pos = LaunchConfiguration('y_pos')
    yaw = LaunchConfiguration('yaw')

    pkg_share = get_package_share_directory('project')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    rviz_conf_path = os.path.join(pkg_share, 'config', 'rviz.rviz')
    xacro_file = os.path.join(
        pkg_share, 'urdf', 'turtlebot3_burger_camera.xacro')
    robot_desc = xacro.process_file(xacro_file).toxml()
    world_file = os.path.join(pkg_share, 'worlds', 'project.world')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    topic_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu[gz.msgs.IMU',
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        output='screen',
    )

    return LaunchDescription([
        declare_use_sim_time_cmd,
        x_pos_arg,
        y_pos_arg,
        yaw_arg,

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
            arguments=[robot_desc],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            parameters=[{
                'use_sim_time': use_sim_time,
                'robot_description': robot_desc,
            }],
            output='screen',
        ),

        gz_sim,
        clock_bridge,
        topic_bridge,

        # Spawn via ros_gz_sim's `create` executable — it speaks gz-transport
        # directly, so no ROS<->gz service bridge is needed for /world/default/create.
        # robot_state_publisher above publishes the URDF on /robot_description, which
        # `-topic` consumes. Yaw arg stays in degrees for book continuity; convert here.
        Node(
            package='ros_gz_sim',
            executable='create',
            output='screen',
            arguments=[
                '-world', 'default',
                '-topic', 'robot_description',
                '-name', 'turtlebot3_burger',
                '-x', x_pos,
                '-y', y_pos,
                '-z', '0.01',
                '-Y', PythonExpression([yaw, ' * 0.017453292519943295']),
            ],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_conf_path,
                       '--ros-args', '--log-level', 'warn'],
            on_exit=Shutdown(),
        ),
    ])
