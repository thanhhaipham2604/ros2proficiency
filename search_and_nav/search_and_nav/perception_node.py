import math
import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Empty, Float32MultiArray
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

from nav_and_search.tf_utils import TFHelper
from nav_and_search.marker_db import MarkerDB


class PerceptionNode(Node):

    def __init__(self):
        super().__init__('perception_node')

        self.tf = TFHelper(self)
        self.db = MarkerDB()

        self.scan = None

        # Publishers
        self.marker_pub = self.create_publisher(Marker, '/hazards', 10)
        self.found_pub = self.create_publisher(String, '/snc/hazard_found', 10)
        self.start_pub = self.create_publisher(Empty, '/snc/start_detected', 10)

        # Subscribers
        self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.create_subscription(Float32MultiArray, '/find_object_2d/objects', self.objects_cb, 10)

    # ------------------------------
    # SENSOR INPUT
    # ------------------------------
    def scan_cb(self, msg):
        self.scan = msg

    # ------------------------------
    # OBJECT DETECTION (VISION)
    # ------------------------------
    def objects_cb(self, msg):
        if not msg.data:
            return

        data = msg.data
        step = 12  # structure from find_object_2d

        for i in range(0, len(data), step):

            obj_id = int(data[i])

            # -------- START MARKER --------
            if obj_id == 13:
                self.get_logger().info("Start marker detected")
                self.start_pub.publish(Empty())
                continue

            # -------- HAZARD MARKERS --------
            if 1 <= obj_id <= 12:

                bearing = self.estimate_bearing(data, i)

                self.process_detection(obj_id, bearing)

    # ------------------------------
    # BEARING ESTIMATION (image → angle)
    # ------------------------------
    def estimate_bearing(self, data, i):
        x_center = data[i + 1]

        image_width = 640.0
        fov = 60.0  # degrees

        offset = (x_center - image_width / 2) / (image_width / 2)

        return offset * (fov / 2)

    # ------------------------------
    # CORE FUSION PIPELINE
    # ------------------------------
    def process_detection(self, hid, bearing):

        r = self.get_range(bearing)
        if r is None:
            return

        pt = PointStamped()
        pt.header.frame_id = "base_link"

        angle = math.radians(bearing)

        pt.point.x = r * math.cos(angle)
        pt.point.y = r * math.sin(angle)

        map_pt = self.tf.transform_point(pt)
        if map_pt is None:
            return

        entry, _ = self.db.add_observation(hid, map_pt.point.x, map_pt.point.y)

        self.publish_marker(entry)

        if entry.count == 3:
            s = String()
            s.data = str(hid)
            self.found_pub.publish(s)

    # ------------------------------
    # LASER RANGE (CORRECT VERSION)
    # ------------------------------
    def get_range(self, bearing):
        if self.scan is None:
            return None

        angle = math.radians(bearing)

        index = int((angle - self.scan.angle_min) / self.scan.angle_increment)

        if index < 0 or index >= len(self.scan.ranges):
            return None

        return self.scan.ranges[index]

    # ------------------------------
    # VISUALIZATION
    # ------------------------------
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
        m.color.g = 0.0
        m.color.b = 0.0

        self.marker_pub.publish(m)


def main():
    rclpy.init()
    node = PerceptionNode()
    rclpy.spin(node)
    rclpy.shutdown()