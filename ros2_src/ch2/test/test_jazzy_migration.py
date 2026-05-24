"""Assert turtlebot3_gazebo launch file names referenced in simulation.launch.py exist on disk.

Run after sourcing the Jazzy install: colcon test --packages-select ch2
"""
import os
import pytest


def _tb3_launch_dir():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('turtlebot3_gazebo'), 'launch')
    except Exception:
        pytest.skip('turtlebot3_gazebo not installed — run inside the Jazzy container')


def test_empty_world_launch_exists():
    launch_dir = _tb3_launch_dir()
    path = os.path.join(launch_dir, 'empty_world.launch.py')
    assert os.path.exists(path), (
        f"empty_world.launch.py not found in {launch_dir}; "
        "update ch2/launch/simulation.launch.py with the correct Jazzy TB3 launch name"
    )


def test_turtlebot3_world_launch_exists():
    launch_dir = _tb3_launch_dir()
    path = os.path.join(launch_dir, 'turtlebot3_world.launch.py')
    assert os.path.exists(path), (
        f"turtlebot3_world.launch.py not found in {launch_dir}; "
        "update ch2/launch/simulation.launch.py with the correct Jazzy TB3 launch name"
    )
