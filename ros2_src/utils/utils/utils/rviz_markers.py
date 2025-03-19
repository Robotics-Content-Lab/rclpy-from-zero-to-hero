from math import atan2, sqrt, hypot, radians, degrees, sin, cos
from tf_transformations import euler_from_quaternion, quaternion_from_euler
 

from visualization_msgs.msg import Marker
from geometry_msgs.msg import PoseStamped, Point

def create_goal_marker(msg: PoseStamped) -> Marker:
    marker = Marker()
    marker.header.frame_id = msg.header.frame_id
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position = msg.pose.position
    marker.pose.orientation = msg.pose.orientation
    marker.scale.x = 0.3
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 0.0
    marker.color.b = 1.0
    marker.color.a = 1.0
    marker.id = 1
    return marker


def delete_goal_marker(id: int) -> Marker:
    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = Marker.ARROW
    marker.action = Marker.DELETEALL
    marker.id = id
    return marker


def draw_line(pt1: Point, pt2: Point) -> Marker:
    length = hypot(pt2.x - pt1.x, pt2.y - pt1.y)
    yaw = atan2((pt2.y - pt1.y), (pt2.x - pt1.x))
    orientation = quaternion_from_euler(yaw=yaw)

    marker = Marker()
    marker.header.frame_id = "odom"
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.pose.position = pt1
    marker.pose.orientation = orientation
    marker.scale.x = length
    marker.scale.y = 0.1
    marker.scale.z = 0.1
    marker.color.r = 0.0
    marker.color.g = 1.0
    marker.color.b = 0.0
    marker.color.a = 0.5
    marker.id = 2
    return marker
