"""Helper functions for spawning objects in Gazebo Harmonic."""
from geometry_msgs.msg import Pose
import rclpy
from ros_gz_interfaces.srv import SpawnEntity
import yaml

# Must match <world name="..."> in the world SDF loaded by the simulation launch file.
# turtlebot3_gazebo's empty world uses "default".
WORLD_NAME = 'default'


def load_landmarks(filename):
    with open(filename, 'r') as f:
        landmarks = yaml.safe_load(f)
    return landmarks


def spawn_landmarks(node, landmarks, model_xml):
    client = node.create_client(SpawnEntity, f'/world/{WORLD_NAME}/create')

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    for i, landmark in enumerate(landmarks):
        request = SpawnEntity.Request()
        request.entity_factory.name = 'cylinder_' + str(i)
        request.entity_factory.sdf = model_xml
        pose = Pose()
        pose.position.x = landmark['x']
        pose.position.y = landmark['y']
        pose.position.z = 0.
        request.entity_factory.pose = pose

        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            if future.result().success:
                node.get_logger().info('Spawned: %s' % request.entity_factory.name)
            else:
                node.get_logger().error(
                    'Failed to spawn entity: %s' % request.entity_factory.name)
        else:
            node.get_logger().error('Service call failed')
