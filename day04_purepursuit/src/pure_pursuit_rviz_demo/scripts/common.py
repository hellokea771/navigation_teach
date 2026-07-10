import math

# Default parameters
DEFAULT_LOOKAHEAD_DISTANCE = 1.0
DEFAULT_TARGET_SPEED = 0.6
DEFAULT_PATH_TYPE = "s_shape"

# Frame IDs
ODOM_FRAME = "odom"
BASE_LINK_FRAME = "base_link"

# Topic names
REFERENCE_PATH_TOPIC = "/reference_path"
CMD_VEL_TOPIC = "/cmd_vel"
ODOM_TOPIC = "/odom"
ACTUAL_PATH_TOPIC = "/actual_path"
LOOKAHEAD_POINT_TOPIC = "/lookahead_point"

# Visualisation marker colours (r, g, b, a)
LOOKAHEAD_COLOR = (1.0, 0.0, 0.0, 0.8)  # red


def heading_from_quaternion(q):
    """Extract yaw angle from a geometry_msgs/Quaternion."""
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def quaternion_from_yaw(yaw):
    """Create a Quaternion message from a yaw angle."""
    from geometry_msgs.msg import Quaternion
    half_yaw = yaw * 0.5
    return Quaternion(
        x=0.0, y=0.0,
        z=math.sin(half_yaw),
        w=math.cos(half_yaw),
    )
