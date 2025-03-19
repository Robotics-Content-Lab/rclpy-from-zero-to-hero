from typing import Callable
import numpy as np
from sklearn.cluster import DBSCAN

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor

def cartesian_to_polar(x, y):
    """Convert Cartesian coordinates to polar."""
    r = np.sqrt(x**2 + y**2)
    theta = np.arctan2(y, x)
    return r, theta

def generate_msg(scan: LaserScan, ranges: np.ndarray, centroids: np.ndarray, get_clock: Callable):
    """Generate a LaserScan message for testing."""
    msg = LaserScan()
    msg.header = scan.header  # Copy header from the original scan
    msg.header.stamp = get_clock().now().to_msg()  # Update the timestamp
    msg.angle_min = scan.angle_min
    msg.angle_max = scan.angle_max
    msg.angle_increment = scan.angle_increment
    msg.time_increment = scan.time_increment
    msg.scan_time = scan.scan_time
    msg.range_min = scan.range_min
    msg.range_max = scan.range_max

    # Initialize with 'inf' for all angles
    clustered_ranges = np.full(len(ranges), float('inf'))
    
    # Ensure theta is correctly normalized and mapped
    for centroid in centroids:
        r, theta = cartesian_to_polar(centroid[0], centroid[1])

        # Normalize theta to be within the range of the scan angles
        if theta < scan.angle_min:
            theta += 2 * np.pi
        elif theta > scan.angle_max:
            theta -= 2 * np.pi

        # Calculate the angle index based on the normalized theta
        angle_index = int((theta - scan.angle_min) / scan.angle_increment)

        # Check if the calculated index is within the valid range
        if 0 <= angle_index < len(clustered_ranges):
            clustered_ranges[angle_index] = min(clustered_ranges[angle_index], r)  # Keep the closer distance

    msg.ranges = clustered_ranges.tolist()
    return msg

def cluster_laserscan(scan: LaserScan, dist_thresh=0.1, min_samples=5, get_clock: Callable=()) -> LaserScan:
    """
    Cluster laser scan data using DBSCAN algorithm.

    :param scan: The input LaserScan message.
    :param dist_thresh: The maximum distance between two samples for one to be considered as in the neighborhood of the other.
    :param min_samples: The number of samples in a neighborhood for a point to be considered as a core point.
    :param get_clock: Callable to get the current ROS time.
    :return: A LaserScan message with the clustered points.
    """
    # Convert scan ranges to numpy array for processing
    ranges = np.array(scan.ranges)
    angles = np.linspace(scan.angle_min, scan.angle_max, len(ranges))

    # Filter out 'inf' and 'nan' values
    valid_indices = np.isfinite(ranges)
    valid_ranges = ranges[valid_indices]
    valid_angles = angles[valid_indices]

    # Convert to Cartesian coordinates
    x = valid_ranges * np.cos(valid_angles)
    y = valid_ranges * np.sin(valid_angles)
    points = np.vstack((x, y)).T

    # Cluster using DBSCAN
    dbscan = DBSCAN(eps=dist_thresh, min_samples=min_samples).fit(points)
    labels = dbscan.labels_

    # Find the centroids of the clusters
    unique_labels = set(labels)
    centroids = np.array([points[labels == label].mean(axis=0) for label in unique_labels if label != -1])

    # Create a new LaserScan message for the clustered data
    clustered_scan = generate_msg(scan, ranges, centroids, get_clock)
    
    return clustered_scan


class SensorProcNode(Node):
    def __init__(self):
        super().__init__('sensor_proc_node')

        # Declare parameters
        self.declare_parameter('dist_thresh', 0.1)
        self.declare_parameter('min_samples', 5)

        self.msg = None

        # Create callback groups
        self.sensor_data_cg = ReentrantCallbackGroup()
        self.command_cg = ReentrantCallbackGroup()

        # QoS profiles
        qos_profile_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            depth=10
        )
        
        qos_profile_command = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=10
        )

        # Subscriptions
        self.create_subscription(
            LaserScan,
            '/scan',
            self.sensor_callback,
            qos_profile_sensor,
            callback_group=self.sensor_data_cg
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            qos_profile_command,
            callback_group=self.command_cg
        )

        # Publisher
        self.clustered_scan_pub = self.create_publisher(
            LaserScan,
            '/clustered_scan',
            qos_profile_sensor
        )

        # Timer
        self.create_timer(1/30, self.process_scan)

    def sensor_callback(self, msg):
        self.get_logger().info(f'Received sensor data')
        self.msg = msg

    def odom_callback(self, msg):
        self.get_logger().info(f'Received odometry data: {msg}')

    def process_scan(self):
        if self.msg:
            dist_thresh = self.get_parameter('dist_thresh').get_parameter_value().double_value
            min_samples = self.get_parameter('min_samples').get_parameter_value().integer_value
            clustered_scan = cluster_laserscan(self.msg, dist_thresh, min_samples, self.get_clock)
            self.clustered_scan_pub.publish(clustered_scan)

def main(args=None):
    rclpy.init(args=args)
    node = SensorProcNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    
    try:
        executor.spin()
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
