#!/usr/bin/env python3
from math import radians
import sys

import rclpy
from ros_gz_interfaces.srv import SpawnEntity
from tf_transformations import quaternion_from_euler

# Must match <world name="..."> in project/worlds/project/project.world
WORLD_NAME = 'default'


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_robot')
    cli = node.create_client(SpawnEntity, f'/world/{WORLD_NAME}/create')

    content = sys.argv[1]

    x_pos = node.declare_parameter('x_pos', -0.55).value
    y_pos = node.declare_parameter('y_pos', 6.2).value
    yaw = node.declare_parameter('yaw', -90.0).value
    (x, y, z, w) = quaternion_from_euler(0, 0, radians(yaw))

    req = SpawnEntity.Request()
    req.entity_factory.name = 'turtlebot3_burger'
    # Pass URDF XML directly — Gazebo Harmonic's built-in URDF importer converts it at spawn time
    req.entity_factory.sdf = content
    req.entity_factory.pose.position.x = x_pos
    req.entity_factory.pose.position.y = y_pos
    req.entity_factory.pose.position.z = 0.01
    req.entity_factory.pose.orientation.x = x
    req.entity_factory.pose.orientation.y = y
    req.entity_factory.pose.orientation.z = z
    req.entity_factory.pose.orientation.w = w
    req.entity_factory.relative_to = 'world'

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info('Result: ' + str(future.result().success))
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
