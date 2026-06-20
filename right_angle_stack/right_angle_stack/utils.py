import math

from geometry_msgs.msg import Quaternion

# 通用数学工具。world 使用 ENU，base_link 使用 FLU。


def normalize_angle(angle):
    return (angle + math.pi) % (2.0 * math.pi) - math.pi


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.w = math.cos(yaw * 0.5)
    q.z = math.sin(yaw * 0.5)
    return q


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(dx, dy, yaw):
    """world 坐标差转到 base_link：local_x 向前，local_y 向左。"""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def body_to_world(local_x, local_y, origin_x, origin_y, yaw):
    """base_link 坐标点转到 world，用于局部锥桶观测建图。"""
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        origin_x + c * local_x - s * local_y,
        origin_y + s * local_x + c * local_y,
    )


def color_key(color):
    value = (color or '').lower()
    if 'blue' in value:
        return 'blue'
    if 'yellow' in value:
        return 'yellow'
    if 'red' in value:
        return 'red'
    return 'unknown'


def set_marker_color(marker, color):
    key = color_key(color)
    marker.color.a = 0.95
    if key == 'blue':
        marker.color.r = 0.02
        marker.color.g = 0.16
        marker.color.b = 1.0
    elif key == 'yellow':
        marker.color.r = 1.0
        marker.color.g = 0.78
        marker.color.b = 0.02
    elif key == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.04
        marker.color.b = 0.02
    else:
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8
