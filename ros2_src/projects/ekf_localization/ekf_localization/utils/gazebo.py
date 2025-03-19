"""File containg helper functions for spawning objects in Gazebo."""
import yaml
import os

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose


def load_landmarks(filename):
    with open(filename, 'r') as f:
        landmarks = yaml.safe_load(f)
    return landmarks


def spawn_landmarks(node, landmarks, model_xml):
    client = node.create_client(SpawnEntity, "/spawn_entity")

    while not client.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    request = SpawnEntity.Request()

    for i, landmark in enumerate(landmarks):
        # Set the name and model
        request.name = 'cylinder_' + str(i)
        request.xml = model_xml
        # Set the pose
        request.robot_namespace = ""
        pose = Pose()
        pose.position.x = landmark['x']
        pose.position.y = landmark['y']
        pose.position.z = 0.  # Assuming ground level, adjust as needed
        request.initial_pose = pose

        # Call the service
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future)
        if future.result() is not None:
            if future.result().success:
                node.get_logger().info('Spawned: %s' % request.name)
            else:
                node.get_logger().error('Failed to spawn entity: %s' % future.result().status_message)
        else:
            node.get_logger().error('Service call failed')
