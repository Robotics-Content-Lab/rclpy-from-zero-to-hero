from math import atan2, sqrt, hypot, radians, degrees, sin, cos
from tf_transformations import euler_from_quaternion, quaternion_from_euler

def normalize(angle: float) -> float:
    """Normalize the angle between pi, -pi
    Args:
        angle (float): angle in rad
    Returns:
        float: normalized angle in rad
    """
    return atan2(sin(angle), cos(angle))