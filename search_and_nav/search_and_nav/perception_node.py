import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from par_snc.tf_utils import TFHelper
from par_snc.marker_db import MarkerDB


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.tf = TFHelper(self)
        self.db = MarkerDB()

        self.scan = None

        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)
        self.found_pub = self.create_publisher(String, '/snc/hazard_found', 10)
        self.start_pub = self.create_publisher(Empty, '/snc/start_detected', 10)

        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(String, '/snc/detection', self.detect_cb, 10)

    def scan_cb(self, msg):
        self.scan = msg

    def detect_cb(self, msg):
        # format: "id,bearing"
        hid, bearing = msg.data.split(',')
        hid = int(hid)
        bearing = float(bearing)

        r = self.get_range(bearing)
        if r is None:
            return

        pt = PointStamped()
        pt.header.frame_id = "base_link"
        pt.point.x = r * math.cos(math.radians(bearing))
        pt.point.y = r * math.sin(math.radians(bearing))

        map_pt = self.tf.transform_point(pt)
        if map_pt is None:
            return

        entry, _ = self.db.add_observation(hid, map_pt.point.x, map_pt.point.y)

        self.publish_marker(entry)

        if entry.count == 3:
            s = String()
            s.data = str(hid)
            self.found_pub.publish(s)

    def get_range(self, bearing):
        if self.scan is None:
            return None
        return min(self.scan.ranges)

    def publish_marker(self, entry):
        m = Marker()
        m.header.frame_id = "map"
        m.id = entry.hazard_id
        m.type = Marker.SPHERE

        m.pose.position.x = entry.x_map
        m.pose.position.y = entry.y_map

        m.scale.x = 0.2
        m.scale.y = 0.2
        m.scale.z = 0.2

        m.color.a = 1.0
        m.color.r = 1.0

        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()