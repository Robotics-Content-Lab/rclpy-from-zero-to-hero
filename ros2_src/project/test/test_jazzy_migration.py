"""Migration tests: assert project package targets Gazebo Harmonic, not Classic."""
from pathlib import Path
import xml.etree.ElementTree as ET

PKG = Path(__file__).parents[1]


# ── B5.1 spawn_robot.py ──────────────────────────────────────────────────────

def test_spawn_robot_uses_ros_gz_interfaces():
    """Ensure ros_gz_interfaces is imported, not gazebo_msgs."""
    content = (PKG / 'project' / 'spawn_robot.py').read_text()
    assert 'gazebo_msgs' not in content, \
        'Classic gazebo_msgs import found in spawn_robot.py'
    assert 'ros_gz_interfaces' in content, \
        'ros_gz_interfaces import missing from spawn_robot.py'


def test_spawn_robot_uses_harmonic_endpoint():
    """Ensure the Harmonic /world/ service endpoint is used."""
    content = (PKG / 'project' / 'spawn_robot.py').read_text()
    assert '/spawn_entity' not in content, \
        'Classic /spawn_entity endpoint found in spawn_robot.py'
    assert '/world/' in content, \
        'Harmonic /world/<name>/create endpoint missing from spawn_robot.py'


def test_spawn_robot_uses_entity_factory():
    """Ensure entity_factory fields are used, not the Classic req.xml."""
    content = (PKG / 'project' / 'spawn_robot.py').read_text()
    assert 'entity_factory' in content, \
        'EntityFactory not used in spawn_robot.py request'
    assert 'req.xml' not in content, \
        'Classic req.xml field still present in spawn_robot.py'


# ── B5.2 xacro plugins ───────────────────────────────────────────────────────

def test_xacro_no_classic_plugins():
    """Ensure no libgazebo_ros_* plugin declarations remain in the xacro."""
    content = (PKG / 'urdf' / 'turtlebot3_burger_camera.xacro').read_text()
    assert 'filename="libgazebo_ros_' not in content, \
        'Classic libgazebo_ros_* plugin declaration found in xacro'


def test_xacro_has_harmonic_diff_drive():
    """Ensure gz-sim-diff-drive-system is present."""
    content = (PKG / 'urdf' / 'turtlebot3_burger_camera.xacro').read_text()
    assert 'gz-sim-diff-drive-system' in content, \
        'gz-sim-diff-drive-system missing from xacro'


def test_xacro_has_harmonic_joint_state_publisher():
    """Ensure gz-sim-joint-state-publisher-system is present."""
    content = (PKG / 'urdf' / 'turtlebot3_burger_camera.xacro').read_text()
    assert 'gz-sim-joint-state-publisher-system' in content, \
        'gz-sim-joint-state-publisher-system missing from xacro'


def test_xacro_has_harmonic_sensors():
    """Ensure at least one Harmonic sensor system plugin is present."""
    content = (PKG / 'urdf' / 'turtlebot3_burger_camera.xacro').read_text()
    assert 'gz-sim-sensors-system' in content \
        or 'gz-sim-imu-system' in content, \
        'No gz-sim sensor system found in xacro'


def test_xacro_has_gz_frame_id():
    """Ensure gz_frame_id overrides are present on sensor blocks."""
    content = (PKG / 'urdf' / 'turtlebot3_burger_camera.xacro').read_text()
    assert 'gz_frame_id' in content, \
        '<gz_frame_id> override missing from xacro sensor blocks'


def test_turtlebot3_sdf_no_classic_plugins():
    """Ensure no libgazebo_ros_* plugin declarations remain in the SDF."""
    sdf_path = (
        PKG / 'models' / 'turltebot3_burger_camera'
        / 'turtlebot3_burger_camera.sdf'
    )
    content = sdf_path.read_text()
    assert "filename='libgazebo_ros_" not in content, \
        'Classic libgazebo_ros_* plugin declaration found in SDF'


def test_turtlebot3_sdf_copy_deleted():
    """Ensure the backup copy SDF file was removed."""
    copy_path = (
        PKG / 'models' / 'turltebot3_burger_camera'
        / 'turtlebot3_burger_camera copy.sdf'
    )
    assert not copy_path.exists(), \
        "Backup 'turtlebot3_burger_camera copy.sdf' should be deleted"


# ── B5.3 SDF versions ────────────────────────────────────────────────────────

def test_world_sdf_version():
    """Ensure project.world declares SDF version 1.9."""
    tree = ET.parse(PKG / 'worlds' / 'project' / 'project.world')
    version = tree.getroot().get('version')
    assert version == '1.9', \
        f"project.world SDF version is '{version}', expected '1.9'"


def test_world_no_classic_plugins():
    """Ensure no libgazebo_ plugin declarations remain in the world file."""
    content = (PKG / 'worlds' / 'project' / 'project.world').read_text()
    assert 'filename="libgazebo_' not in content \
        and "filename='libgazebo_" not in content, \
        'Classic libgazebo_ plugin declaration found in project.world'


def test_project_model_sdf_version():
    """Ensure project/model.sdf declares SDF version 1.9."""
    tree = ET.parse(PKG / 'models' / 'project' / 'model.sdf')
    version = tree.getroot().get('version')
    assert version == '1.9', \
        f"project/model.sdf SDF version is '{version}', expected '1.9'"


def test_robot_model_sdf_version():
    """Ensure turtlebot3_burger_camera.sdf declares SDF version 1.9."""
    tree = ET.parse(
        PKG / 'models' / 'turltebot3_burger_camera'
        / 'turtlebot3_burger_camera.sdf'
    )
    version = tree.getroot().get('version')
    assert version == '1.9', \
        f"turtlebot3_burger_camera.sdf SDF version is '{version}', expected '1.9'"


# ── B5.4 launch file + hook ──────────────────────────────────────────────────

def test_simulation_launch_no_classic_gazebo():
    """Ensure no Classic gzserver/gzclient/gazebo_ros references remain."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'gzserver' not in content, \
        'Classic gzserver reference found in simulation.launch.py'
    assert 'gzclient' not in content, \
        'Classic gzclient reference found in simulation.launch.py'
    assert 'gazebo_ros' not in content, \
        'Classic gazebo_ros package reference found in simulation.launch.py'


def test_simulation_launch_uses_gz_sim():
    """Ensure gz_sim.launch.py is used."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'gz_sim.launch.py' in content, \
        'gz_sim.launch.py not found in project simulation.launch.py'


def test_simulation_launch_has_clock_bridge():
    """Ensure the clock bridge is present."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' in content, \
        'Clock bridge missing from project simulation.launch.py'


def test_simulation_launch_has_topic_bridges():
    """Ensure all sensor/actuator topic bridges are declared."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    for topic in ['/scan', '/imu', '/image_raw', '/camera_info',
                  '/cmd_vel', '/odom', '/joint_states']:
        assert topic in content, \
            f"Bridge for topic '{topic}' missing from simulation.launch.py"


def test_simulation_launch_spawns_via_ros_gz_sim_create():
    """Ensure the launch spawns via ros_gz_sim's `create` executable.

    /world/default/create is gz-transport-only, so parameter_bridge can't bridge
    it; ros_gz_sim's create speaks gz-transport directly.
    """
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert "package='ros_gz_sim'" in content, \
        'simulation.launch.py does not invoke a ros_gz_sim Node for spawning'
    assert "executable='create'" in content, \
        'simulation.launch.py does not invoke `create` for spawning'


def test_simulation_launch_does_not_run_project_spawn_robot():
    """Ensure the legacy rclpy spawn_robot Node isn't wired in.

    spawn_robot.py is kept in tree as a teaching reference for the SpawnEntity
    client pattern, but the launch path doesn't depend on it because it would
    hang waiting on the unbridged /world/default/create service.
    """
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert "executable='spawn_robot'" not in content, \
        ('simulation.launch.py still launches the project.spawn_robot Node; '
         '/world/default/create is unbridged, so this would hang')


def test_hook_renamed_and_uses_gz_env_var():
    """Ensure hook_gz.sh exists and uses GZ_SIM_RESOURCE_PATH."""
    hook = PKG / 'hook' / 'hook_gz.sh'
    assert hook.exists(), \
        'hook_gz.sh not found — hook_gazebo.sh must be renamed'
    content = hook.read_text()
    assert 'GAZEBO_MODEL_PATH' not in content, \
        'Classic GAZEBO_MODEL_PATH still in hook_gz.sh'
    assert 'GZ_SIM_RESOURCE_PATH' in content, \
        'GZ_SIM_RESOURCE_PATH missing from hook_gz.sh'


def test_old_hook_deleted():
    """Ensure the old hook_gazebo.sh has been removed."""
    old_hook = PKG / 'hook' / 'hook_gazebo.sh'
    assert not old_hook.exists(), \
        'Old hook_gazebo.sh still exists — rename it to hook_gz.sh'


def test_setup_py_references_new_hook():
    """Ensure setup.py references the renamed hook file."""
    content = (PKG / 'setup.py').read_text()
    assert 'hook_gazebo.sh' not in content, \
        'Old hook_gazebo.sh reference still in setup.py'
    assert 'hook_gz.sh' in content, \
        'New hook_gz.sh reference missing from setup.py'


# ── B7 package.xml ───────────────────────────────────────────────────────────

def test_package_xml_no_classic_gazebo_deps():
    """Ensure Classic Gazebo packages are not declared as dependencies."""
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'gazebo_ros' not in deps, 'Classic gazebo_ros in project/package.xml'
    assert 'gazebo_msgs' not in deps, \
        'Classic gazebo_msgs in project/package.xml'
    assert 'gazebo_plugins' not in deps, \
        'Classic gazebo_plugins in project/package.xml'


def test_package_xml_has_ros_gz_deps():
    """Ensure ros_gz_sim, ros_gz_bridge, ros_gz_interfaces are declared."""
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'ros_gz_sim' in deps, \
        'ros_gz_sim missing from project/package.xml'
    assert 'ros_gz_bridge' in deps, \
        'ros_gz_bridge missing from project/package.xml'
    assert 'ros_gz_interfaces' in deps, \
        'ros_gz_interfaces missing from project/package.xml'
