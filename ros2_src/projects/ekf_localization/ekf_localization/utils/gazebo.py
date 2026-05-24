"""Helper functions for spawning objects in Gazebo Harmonic.

Uses `ros_gz_sim`'s `create` executable rather than an rclpy SpawnEntity client —
Harmonic's /world/<name>/create lives on gz-transport, not ROS, and `parameter_bridge`
only bridges topics. Shelling out to `ros2 run ros_gz_sim create` matches what TB3's
`spawn_turtlebot3.launch.py` does and avoids needing a service bridge.
"""
import os
import subprocess
import tempfile
import time

import yaml

# Must match <world name="..."> in the world SDF loaded by the simulation launch file.
# turtlebot3_gazebo's empty world uses "default".
WORLD_NAME = 'default'


def load_landmarks(filename):
    with open(filename, 'r') as f:
        landmarks = yaml.safe_load(f)
    return landmarks


def spawn_landmarks(node, landmarks, model_xml):
    """Spawn each landmark by shelling out to `ros2 run ros_gz_sim create`.

    `node` is kept in the signature for caller compatibility and is used only
    for logging. The landmark YAML may contain duplicate coordinates; gz rejects
    duplicate model names but we tolerate failures and continue.
    """
    # gz `create` reads the SDF from a file path — write the model XML once.
    with tempfile.NamedTemporaryFile(
            suffix='.sdf', mode='w', delete=False) as f:
        f.write(model_xml)
        sdf_path = f.name

    # Give gz_sim a moment to advertise /world/default/create. spawn_turtlebot3
    # already gates ordering ahead of this Node in the launch, so a short wait
    # is usually sufficient.
    time.sleep(5)

    try:
        for i, landmark in enumerate(landmarks):
            name = f'cylinder_{i}'
            result = subprocess.run(
                [
                    'ros2', 'run', 'ros_gz_sim', 'create',
                    '-world', WORLD_NAME,
                    '-file', sdf_path,
                    '-name', name,
                    '-x', str(landmark['x']),
                    '-y', str(landmark['y']),
                    '-z', '0',
                ],
                check=False,
                capture_output=True,
                text=True,
            )
            if result.returncode == 0:
                node.get_logger().info(f'Spawned: {name}')
            else:
                node.get_logger().error(
                    f'Failed to spawn {name} (rc={result.returncode}): '
                    f'{result.stderr.strip()}'
                )
    finally:
        os.unlink(sdf_path)
