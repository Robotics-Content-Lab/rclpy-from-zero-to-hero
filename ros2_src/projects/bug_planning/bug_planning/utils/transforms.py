#!/usr/bin/env python3

from math import atan2, asin, sin, cos, radians, degrees
from geometry_msgs.msg import Quaternion

def euler_from_quaternion(quat: Quaternion):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    x, y, z, w = quat.x, quat.y, quat.z, quat.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
    
    return [roll_x, pitch_y, yaw_z] # in radians

def euler_to_quaternion(roll=0.0, pitch=0.0, yaw=0.0) -> Quaternion:
    """Converts RPY euler angles, given in degrees, to quaternions.
    Args:
        roll (float, optional): _description_. Defaults to 0.0.
        pitch (float, optional): _description_. Defaults to 0.0.
        yaw (float, optional): _description_. Defaults to 0.0.
    Returns:
        Quaternion: _description_
    """
    roll = radians(roll)
    roll = radians(pitch)
    roll = radians(yaw)
    cy = cos(yaw * 0.5)
    sy = sin(yaw * 0.5)
    cp = cos(pitch * 0.5)
    sp = sin(pitch * 0.5)
    cr = cos(roll * 0.5)
    sr = sin(roll * 0.5)
    
    qw = cy * cp * cr + sy * sp * sr
    qx = cy * cp * sr - sy * sp * cr
    qy = sy * cp * sr + cy * sp * cr
    qz = sy * cp * cr - cy * sp * sr
    return Quaternion(w=qw, x=qx, y=qy, z=qz)

def normalize(angle: float) -> float:
    """Normalize the angle between pi, -pi
    Args:
        angle (float): angle in rad
    Returns:
        float: normalized angle in rad
    """
    return atan2(sin(angle), cos(angle))