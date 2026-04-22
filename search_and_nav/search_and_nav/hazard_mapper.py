import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from search_and_nav.tf_utils import TFHelper
from search_and_nav.marker_db import MarkerDB


class HazardMapper(Node):
    def __init__(self):
        super().__init__('hazard_mapper')

        self.declare_parameter('laser_angle_window_deg', 4.0)
        self.declare_parameter('duplicate_distance_threshold', 0.45)
        self.declare_parameter('min_confirmations', 3)

        duplicate_threshold = self.get_parameter('duplicate_distance_threshold').value
        min_confirmations = self.get_parameter('min_confirmations').value
        self.db = MarkerDB(duplicate_threshold, min_confirmations)

        self.tf = TFHelper(self)
        self.latest_scan = None

        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)
        self.hazard_found_pub = self.create_publisher(String, '/snc/hazard_found', 10)

        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)
        self.create_subscription(String, '/snc/detection', self.on_detection, 10)

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_detection(self, msg: String):
        """
        Input format:
        "hazard_id,bearing_deg"
        example: "3,-12.5"
        """
        if self.latest_scan is None:
            self.get_logger().warn('No scan yet')
            return

        try:
            hazard_id_str, bearing_deg_str = msg.data.split(',')
            hazard_id = int(hazard_id_str)
            bearing_deg = float(bearing_deg_str)
        except ValueError:
            self.get_logger().warn(f'Invalid detection string: {msg.data}')
            return

        range_m = self.range_from_bearing_deg(bearing_deg)
        if range_m is None:
            self.get_logger().warn('No valid range for detection')
            return

        point_cam = PointStamped()
        point_cam.header.frame_id = 'base_link'
        point_cam.header.stamp = self.get_clock().now().to_msg()
        bearing_rad = math.radians(bearing_deg)
        point_cam.point.x = range_m * math.cos(bearing_rad)
        point_cam.point.y = range_m * math.sin(bearing_rad)
        point_cam.point.z = 0.0

        point_map = self.tf.transform_point(point_cam, 'map')
        if point_map is None:
            return

        entry, _is_new = self.db.add_observation(
            hazard_id,
            point_map.point.x,
            point_map.point.y
        )

        self.publish_marker(entry.hazard_id, entry.x_map, entry.y_map, entry.count)

        if entry.count == self.db.min_confirmations:
            s = String()
            s.data = str(entry.hazard_id)
            self.hazard_found_pub.publish(s)

    def range_from_bearing_deg(self, bearing_deg):
        scan = self.latest_scan
        if scan is None:
            return None

        center = math.radians(bearing_deg)
        half_window = math.radians(self.get_parameter('laser_angle_window_deg').value) / 2.0

        values = []
        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment
            if abs(angle - center) <= half_window:
                if math.isfinite(r) and scan.range_min < r < scan.range_max:
                    values.append(r)

        if not values:
            return None

        values.sort()
        return values[len(values) // 2]

    def publish_marker(self, hazard_id, x_map, y_map, count):
        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hazards'
        marker.id = int(hazard_id)
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x_map
        marker.pose.position.y = y_map
        marker.pose.position.z = 0.15
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.22
        marker.scale.y = 0.22
        marker.scale.z = 0.22
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = max(0.0, 1.0 - min(count, 5) / 5.0)
        marker.color.b = 0.0
        self.marker_pub.publish(marker)


def main():
    rclpy.init()
    node = HazardMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()