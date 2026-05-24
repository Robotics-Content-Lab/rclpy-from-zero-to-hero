"""Assert ch5 simulation launch file targets Gazebo Harmonic, not Classic."""
from pathlib import Path
import xml.etree.ElementTree as ET

PKG = Path(__file__).parents[1]


def test_ch5_launch_no_classic_gazebo():
    """Ensure no Classic gzserver/gzclient/gazebo_ros references remain."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'gzserver' not in content, \
        'Classic gzserver reference found in ch5 launch'
    assert 'gzclient' not in content, \
        'Classic gzclient reference found in ch5 launch'
    assert 'gazebo_ros' not in content, \
        'Classic gazebo_ros package reference found in ch5 launch'


def test_ch5_launch_uses_ros_gz_sim():
    """Ensure ros_gz_sim and gz_sim.launch.py are referenced."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert 'ros_gz_sim' in content, \
        'ros_gz_sim not found in ch5 launch file'
    assert 'gz_sim.launch.py' in content, \
        'gz_sim.launch.py not found in ch5 launch file'


def test_ch5_launch_has_clock_bridge():
    """Ensure the clock bridge argument is present."""
    content = (PKG / 'launch' / 'simulation.launch.py').read_text()
    assert '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' in content, \
        'Clock bridge argument missing from ch5 launch file'


def test_ch5_package_xml_has_ros_gz_deps():
    """Ensure ros_gz_sim and ros_gz_bridge are declared as dependencies."""
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'ros_gz_sim' in deps, 'ros_gz_sim missing from ch5/package.xml'
    assert 'ros_gz_bridge' in deps, \
        'ros_gz_bridge missing from ch5/package.xml'


def test_ch5_package_xml_no_classic_deps():
    """Ensure Classic gazebo_ros is not in package.xml."""
    tree = ET.parse(PKG / 'package.xml')
    deps = {e.text for e in tree.findall('depend')}
    assert 'gazebo_ros' not in deps, \
        'Classic gazebo_ros still in ch5/package.xml'
