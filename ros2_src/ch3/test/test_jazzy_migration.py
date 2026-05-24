"""Assert ch2's simulation.launch.py (which ch3 delegates to) is accessible.

Run after sourcing the Jazzy install: colcon test --packages-select ch3
"""
import os
import pytest


def test_ch2_simulation_launch_accessible():
    try:
        from ament_index_python.packages import get_package_share_directory
        ch2_launch_dir = os.path.join(get_package_share_directory('ch2'), 'launch')
    except Exception:
        pytest.skip('ch2 not installed — build ch2 first then run colcon test')

    path = os.path.join(ch2_launch_dir, 'simulation.launch.py')
    assert os.path.exists(path), (
        f"ch2/launch/simulation.launch.py not found at {path}; "
        "ensure ch2 is built before testing ch3"
    )
