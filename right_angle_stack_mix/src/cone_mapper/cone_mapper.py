#!/usr/bin/env python3
import math

import rclpy
from fsd_common_msgs.msg import Cone, Map
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def quaternion_to_yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def body_to_world(local_x, local_y, origin_x, origin_y, yaw):
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        origin_x + c * local_x - s * local_y,
        origin_y + s * local_x + c * local_y,
    )


def set_marker_color(marker, color):
    marker.color.a = 0.95
    if color == 'blue':
        marker.color.r = 0.02
        marker.color.g = 0.16
        marker.color.b = 1.0
    elif color == 'yellow':
        marker.color.r = 1.0
        marker.color.g = 0.78
        marker.color.b = 0.02
    elif color == 'red':
        marker.color.r = 1.0
        marker.color.g = 0.04
        marker.color.b = 0.02
    else:
        marker.color.r = 0.8
        marker.color.g = 0.8
        marker.color.b = 0.8


class ConeMapper(Node):
    def __init__(self):
        super().__init__('cone_mapper')

        self.declare_parameter('perception_map_topic', '/perception/cones')
        self.declare_parameter('pose_topic', '/localization/pose')
        self.declare_parameter('merge_distance', 0.75)
        self.declare_parameter('marker_scale_x', 0.35)
        self.declare_parameter('marker_scale_y', 0.35)
        self.declare_parameter('marker_scale_z', 0.56)

        self.merge_distance = float(self.get_parameter('merge_distance').value)
        self.marker_scale_x = float(self.get_parameter('marker_scale_x').value)
        self.marker_scale_y = float(self.get_parameter('marker_scale_y').value)
        self.marker_scale_z = float(self.get_parameter('marker_scale_z').value)
        self.pose = None
        self.landmarks = {
            'blue': [],
            'yellow': [],
            'red': [],
            'unknown': [],
        }

        self.map_pub = self.create_publisher(Map, '/estimation/slam/map', 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/visualization/cone_map', 10)

        self.create_subscription(
            PoseStamped,
            self.get_parameter('pose_topic').value,
            self.on_pose,
            10,
        )
        self.create_subscription(
            Map,
            self.get_parameter('perception_map_topic').value,
            self.on_map_detection,
            10,
        )
        self.create_timer(0.2, self.publish_outputs)
        self.get_logger().info('Cone mapper started. Local perception is transformed into the world ENU frame.')

    def on_pose(self, msg):
        self.pose = (
            msg.pose.position.x,
            msg.pose.position.y,
            quaternion_to_yaw(msg.pose.orientation),
        )

    def on_map_detection(self, msg):
        self.process_cones(msg.header.frame_id, msg.cone_red, 'red')
        self.process_cones(msg.header.frame_id, msg.cone_blue, 'blue')
        self.process_cones(msg.header.frame_id, msg.cone_yellow, 'yellow')
        self.process_cones(msg.header.frame_id, msg.cone_unknown, 'unknown')
        self.publish_outputs()

    def process_cones(self, frame_id, cones, fallback_color):
        for cone in cones:
            color = cone.color if cone.color else fallback_color
            transformed = self.to_world(frame_id, cone)
            if transformed is None:
                continue
            wx, wy = transformed
            self.merge_landmark(color, wx, wy, cone.pose_confidence, cone.color_confidence)

    def to_world(self, frame_id, cone):
        if frame_id == 'world':
            return cone.position.x, cone.position.y
        if self.pose is None:
            return None
        x, y, yaw = self.pose
        return body_to_world(cone.position.x, cone.position.y, x, y, yaw)

    def merge_landmark(self, color, x, y, pose_confidence, color_confidence):
        bucket = self.landmarks[color]
        closest = None
        best_distance = float('inf')
        for landmark in bucket:
            dist = math.hypot(landmark['x'] - x, landmark['y'] - y)
            if dist < best_distance:
                best_distance = dist
                closest = landmark

        if closest is None or best_distance > self.merge_distance:
            bucket.append({
                'x': x,
                'y': y,
                'count': 1,
                'pose_confidence': pose_confidence,
                'color_confidence': color_confidence,
            })
            return

        count = min(closest['count'] + 1, 30)
        alpha = 1.0 / count
        closest['x'] = (1.0 - alpha) * closest['x'] + alpha * x
        closest['y'] = (1.0 - alpha) * closest['y'] + alpha * y
        closest['count'] = count
        closest['pose_confidence'] = max(closest['pose_confidence'], pose_confidence)
        closest['color_confidence'] = max(closest['color_confidence'], color_confidence)

    def make_cone(self, color, landmark):
        cone = Cone()
        cone.position.x = landmark['x']
        cone.position.y = landmark['y']
        cone.position.z = 0.0
        cone.color = color
        cone.pose_confidence = float(landmark['pose_confidence'])
        cone.color_confidence = float(landmark['color_confidence'])
        return cone

    def publish_outputs(self):
        stamp = self.get_clock().now().to_msg()
        msg = Map()
        msg.header.stamp = stamp
        msg.header.frame_id = 'world'
        msg.cone_blue = [self.make_cone('blue', item) for item in self.landmarks['blue']]
        msg.cone_yellow = [self.make_cone('yellow', item) for item in self.landmarks['yellow']]
        msg.cone_red = [self.make_cone('red', item) for item in self.landmarks['red']]
        msg.cone_unknown = [self.make_cone('unknown', item) for item in self.landmarks['unknown']]
        self.map_pub.publish(msg)

        markers = MarkerArray()
        clear = Marker()
        clear.header = msg.header
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        marker_id = 0
        for color, bucket in self.landmarks.items():
            for landmark in bucket:
                marker = Marker()
                marker.header = msg.header
                marker.ns = f'{color}_cones'
                marker.id = marker_id
                marker_id += 1
                marker.type = Marker.CYLINDER
                marker.action = Marker.ADD
                marker.pose.position.x = landmark['x']
                marker.pose.position.y = landmark['y']
                marker.pose.position.z = self.marker_scale_z / 2.0
                marker.pose.orientation.w = 1.0
                marker.scale.x = self.marker_scale_x
                marker.scale.y = self.marker_scale_y
                marker.scale.z = self.marker_scale_z
                set_marker_color(marker, color)
                markers.markers.append(marker)
        self.marker_pub.publish(markers)


def main(args=None):
    rclpy.init(args=args)
    node = ConeMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()