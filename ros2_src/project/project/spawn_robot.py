#!/usr/bin/env python3
import sys
from math import radians
import rclpy
from gazebo_msgs.srv import SpawnEntity
from tf_transformations import quaternion_from_euler


def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node('spawn_robot')
    cli = node.create_client(SpawnEntity, '/spawn_entity')

    content = sys.argv[1]

    req = SpawnEntity.Request()
    req.xml = content
    x_pos = node.declare_parameter('x_pos', -0.55).value
    y_pos = node.declare_parameter('y_pos', 6.2).value
    yaw = node.declare_parameter('yaw', -90.0).value
    (x, y, z, w) = quaternion_from_euler(0, 0, radians(yaw))
    req.initial_pose.position.x = x_pos
    req.initial_pose.position.y = y_pos
    req.initial_pose.position.z = 0.01
    req.initial_pose.orientation.x = x
    req.initial_pose.orientation.y = y
    req.initial_pose.orientation.z = z
    req.initial_pose.orientation.w = w
    req.reference_frame = 'world'

    while not cli.wait_for_service(timeout_sec=1.0):
        node.get_logger().info('service not available, waiting again...')

    future = cli.call_async(req)
    rclpy.spin_until_future_complete(node, future)

    if future.result() is not None:
        node.get_logger().info(
            'Result ' + str(future.result().success) + ' ' + future.result().status_message)
    else:
        node.get_logger().info('Service call failed %r' % (future.exception(),))

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()