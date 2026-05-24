"""Migration tests: assert ekf_localization targets Gazebo Harmonic, not Classic."""
from pathlib import Path
import xml.etree.ElementTree as ET

PKG = Path(__file__).parents[1]


def test_gazebo_util_no_classic_gazebo_msgs():
    """Ensure no Classic gazebo_msgs / Iron-era /spawn_entity references."""
    content = (PKG / 'ekf_localization' / 'utils' / 'gazebo.py').read_text()
    assert 'gazebo_msgs' not in content, \
        'Classic gazebo_msgs import found in ekf_localization/utils/gazebo.py'
    assert '/spawn_entity' not in content, \
        'Classic /spawn_entity endpoint found in gazebo.py'


def test_gazebo_util_uses_ros_gz_sim_create():
    """Ensure spawning shells out to `ros2 run ros_gz_sim create`.

    Harmonic's /world/<name>/create is a gz-transport service. parameter_bridge
    doesn't bridge services, so we use ros_gz_sim's create executable directly.
    """
    content = (PKG / 'ekf_localization' / 'utils' / 'gazebo.py').read_text()
    assert 'ros_gz_sim' in content and 'create' in content, \
        'gazebo.py should invoke ros_gz_sim create'


def test_gazebo_util_no_rclpy_spawn_entity_client():
    """Ensure the rclpy SpawnEntity client pattern is gone — it hangs on Harmonic."""
    content = (PKG / 'ekf_localization' / 'utils' / 'gazebo.py').read_text()
    assert 'create_client' not in content, \
        ('gazebo.py still constructs an rclpy create_client; '
         '/world/default/create is gz-transport-only, not ROS-bridged')
    assert 'from ros_gz_interfaces' not in content, \
        'gazebo.py should not import from ros_gz_interfaces (SpawnEntity client removed)'


def test_simulation_launch_no_classic_gazebo():
    """Ensure no Classic gzserver/gzclient/gazebo_ros references remain."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'gzserver' not in content, \
        'Classic gzserver reference found in ekf_localization simulation.launch.py'
    assert 'gzclient' not in content, \
        'Classic gzclient reference found in ekf_localization simulation.launch.py'
    assert 'gazebo_ros' not in content, \
        'Classic gazebo_ros reference found in ekf_localization simulation.launch.py'


def test_simulation_launch_uses_gz_sim():
    """Ensure gz_sim.launch.py is used."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'gz_sim.launch.py' in content, \
        'gz_sim.launch.py not found in ekf_localization simulation.launch.py'


def test_simulation_launch_has_clock_bridge():
    """Ensure the clock bridge is present."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' in content, \
        'Clock bridge missing from ekf_localization simulation.launch.py'


def test_landmark_model_sdf_version():
    """Ensure Landmark/model.sdf declares SDF version 1.9."""
    tree = ET.parse(PKG / 'models' / 'Landmark' / 'model.sdf')
    version = tree.getroot().get('version')
    assert version == '1.9', \
        f"Landmark/model.sdf SDF version is '{version}', expected '1.9'"


def test_package_xml_no_classic_deps():
    """Ensure Classic Gazebo packages are not declared as dependencies."""
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'gazebo_ros' not in deps, \
        'Classic gazebo_ros in ekf_localization/package.xml'
    assert 'gazebo_msgs' not in deps, \
        'Classic gazebo_msgs in ekf_localization/package.xml'


def test_package_xml_has_ros_gz_deps():
    """Ensure ros_gz_sim and ros_gz_bridge are declared.

    ros_gz_interfaces is no longer needed — spawning now goes through ros_gz_sim's
    `create` executable rather than the SpawnEntity service client.
    """
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'ros_gz_sim' in deps, \
        'ros_gz_sim missing from ekf_localization/package.xml'
    assert 'ros_gz_bridge' in deps, \
        'ros_gz_bridge missing from ekf_localization/package.xml'
