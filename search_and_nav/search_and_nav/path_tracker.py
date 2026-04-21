import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class HazardMapper(Node):
    def __init__(self):
        super().__init__('hazard_mapper')

        self.latest_scan = None
        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)
        self.hazard_found_pub = self.create_publisher(String, '/snc/hazard_found', 10)

        self.create_subscription(LaserScan, '/scan', self.on_scan, 10)

        # Replace this with your real detector topic later
        self.create_subscription(String, '/snc/mock_detection', self.on_detection, 10)

    def on_scan(self, msg: LaserScan):
        self.latest_scan = msg

    def on_detection(self, msg: String):
        if self.latest_scan is None:
            self.get_logger().warn('Detection received but no scan yet')
            return

        # Temporary mock example:
        # msg.data format: "hazard_id,bearing_deg"
        try:
            hazard_id_str, bearing_deg_str = msg.data.split(',')
            hazard_id = int(hazard_id_str)
            bearing_deg = float(bearing_deg_str)
        except ValueError:
            self.get_logger().warn('Invalid detection format')
            return

        range_est = self.get_range_for_bearing(math.radians(bearing_deg))
        if range_est is None:
            self.get_logger().warn('No valid range for detection')
            return

        x_base = range_est * math.cos(math.radians(bearing_deg))
        y_base = range_est * math.sin(math.radians(bearing_deg))

        # TODO:
        # transform (x_base, y_base) from robot/camera frame into map frame using TF
        x_map = x_base
        y_map = y_base

        marker = Marker()
        marker.header.frame_id = 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'hazards'
        marker.id = hazard_id
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = x_map
        marker.pose.position.y = y_map
        marker.pose.position.z = 0.2
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        marker.color.a = 1.0
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0

        self.marker_pub.publish(marker)

        found = String()
        found.data = str(hazard_id)
        self.hazard_found_pub.publish(found)

    def get_range_for_bearing(self, bearing_rad):
        scan = self.latest_scan
        if scan is None:
            return None

        index = int((bearing_rad - scan.angle_min) / scan.angle_increment)
        window = range(max(0, index - 2), min(len(scan.ranges), index + 3))

        vals = [
            scan.ranges[i] for i in window
            if math.isfinite(scan.ranges[i]) and scan.range_min < scan.ranges[i] < scan.range_max
        ]

        if not vals:
            return None

        vals.sort()
        return vals[len(vals) // 2]


def main():
    rclpy.init()
    node = HazardMapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()