#!/usr/bin/env python3
import math
import random
import xml.etree.ElementTree as ET

import rclpy
from fsd_common_msgs.msg import Cone, Map
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node



def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def world_to_body(dx, dy, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return c * dx + s * dy, -s * dx + c * dy


def load_cones_from_sdf(path):
    tree = ET.parse(path)
    root = tree.getroot()
    cones = []
    for include in root.findall(".//include"):
        uri = include.findtext("uri", default="")
        name = include.findtext("name", default="")
        pose_text = include.findtext("pose", default="0 0 0 0 0 0")
        pose_values = [float(v) for v in pose_text.split()]
        x = pose_values[0] if len(pose_values) > 0 else 0.0
        y = pose_values[1] if len(pose_values) > 1 else 0.0
        z = pose_values[2] if len(pose_values) > 2 else 0.0
        label = f"{uri} {name}".lower()
        if "blue" in label:
            color = "blue"
        elif "yellow" in label:
            color = "yellow"
        elif "red" in label:
            color = "red"
        else:
            color = "unknown"
        cones.append((color, x, y, z))
    return cones


DEFAULT_CONES = [
    ("blue", -2.0, -15.0, 0.0), ("yellow", 2.0, -15.0, 0.0),
    ("blue", -2.0, -10.0, 0.0), ("yellow", 2.0, -10.0, 0.0),
    ("blue", -2.0, -5.0, 0.0),  ("yellow", 2.0, -5.0, 0.0),
    ("blue", -2.0, 0.0, 0.0),   ("yellow", 2.0, 0.0, 0.0),
    ("blue", -1.31, 4.32, 0.0), ("yellow", 2.76, 3.83, 0.0),
    ("blue", 0.68, 8.23, 0.0),  ("yellow", 4.93, 7.07, 0.0),
    ("blue", 3.77, 11.32, 0.0), ("yellow", 8.17, 9.24, 0.0),
    ("blue", 7.68, 13.31, 0.0), ("yellow", 12.0, 10.0, 0.0),
    ("blue", 12.0, 14.0, 0.0),  ("yellow", 17.0, 10.0, 0.0),
    ("blue", 17.0, 14.0, 0.0),  ("yellow", 22.0, 10.0, 0.0),
    ("blue", 22.0, 14.0, 0.0),  ("yellow", 27.0, 10.0, 0.0),
    ("blue", 27.0, 14.0, 0.0),
]


class TrackPerception(Node):
    def __init__(self):
        super().__init__("track_perception")
        self.declare_parameter("track_sdf", "")
        self.declare_parameter("max_range", 18.0)
        self.declare_parameter("lateral_range", 9.0)
        self.declare_parameter("rear_margin", 1.0)
        self.declare_parameter("position_noise_std", 0.05)
        self.declare_parameter("publish_rate", 10.0)
        self.declare_parameter("map_topic", "/perception/cones")
        self.max_range = float(self.get_parameter("max_range").value)
        self.lateral_range = float(self.get_parameter("lateral_range").value)
        self.rear_margin = float(self.get_parameter("rear_margin").value)
        self.noise_std = float(self.get_parameter("position_noise_std").value)
        track_sdf = str(self.get_parameter("track_sdf").value)
        if track_sdf:
            try:
                self.cones = load_cones_from_sdf(track_sdf)
            except Exception as exc:
                self.get_logger().warn(f"Failed to load track SDF, using built-in cone list: {exc}")
                self.cones = DEFAULT_CONES
        else:
            self.cones = DEFAULT_CONES
        self.x = 0.0
        self.y = -15.0
        self.yaw = 1.57079632679
        self.map_pub = self.create_publisher(Map, self.get_parameter("map_topic").value, 10)
        self.create_subscription(PoseStamped, "/localization/pose", self.on_pose, 10)
        self.create_timer(1.0 / float(self.get_parameter("publish_rate").value), self.on_timer)
        self.get_logger().info(f"Track perception publishing {len(self.cones)} known cones in base_link frame.")

    def on_pose(self, msg):
        self.x = msg.pose.position.x
        self.y = msg.pose.position.y
        self.yaw = quaternion_to_yaw(msg.pose.orientation)

    def make_local_cone(self, color, local_x, local_y, z):
        cone = Cone()
        cone.position.x = local_x + random.gauss(0.0, self.noise_std)
        cone.position.y = local_y + random.gauss(0.0, self.noise_std)
        cone.position.z = z
        cone.color = color
        cone.pose_confidence = 0.9
        cone.color_confidence = 0.98
        return cone

    def on_timer(self):
        stamp = self.get_clock().now().to_msg()
        cone_map = Map()
        cone_map.header.stamp = stamp
        cone_map.header.frame_id = "base_link"
        for color, wx, wy, wz in self.cones:
            local_x, local_y = world_to_body(wx - self.x, wy - self.y, self.yaw)
            if local_x < -self.rear_margin or local_x > self.max_range:
                continue
            if abs(local_y) > self.lateral_range:
                continue
            cone = self.make_local_cone(color, local_x, local_y, wz)
            if color == "blue":
                cone_map.cone_blue.append(cone)
            elif color == "yellow":
                cone_map.cone_yellow.append(cone)
            elif color == "red":
                cone_map.cone_red.append(cone)
            else:
                cone_map.cone_unknown.append(cone)
        self.map_pub.publish(cone_map)


def main(args=None):
    rclpy.init(args=args)
    node = TrackPerception()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
