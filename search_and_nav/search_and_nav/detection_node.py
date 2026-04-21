#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray 
from tf2_ros import TransformListener, Buffer
import math

class HazardLocatorNode(Node):
    def __init__(self):
        super().__init__('hazard_locator_node')
        
        self.START_MARKER_ID = 13.0 
        

        self.is_started = False
        self.found_hazards = {} # Save as {id: (x, y)}
        self.current_scan = None
        
        # --- TF & SUBSCRIBERS ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        self.obj_sub = self.create_subscription(
            Float32MultiArray, '/objects', self.object_callback, 10)
            
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # --- PUBLISHER ---
        # 
        self.marker_pub = self.create_publisher(MarkerArray, '/hazards', 10)
            
        self.get_logger().info("Hazard Locator Node is ready...")

    def scan_callback(self, msg):
        self.current_scan = msg

    def object_callback(self, msg):
        # find_object_2d : [id, width, height, mat...]
        if len(msg.data) == 0 or self.current_scan is None:
            return

        obj_id = msg.data[0]

        # 1. Cehck START
        if obj_id == self.START_MARKER_ID:
            if not self.is_started:
                self.is_started = True
                self.get_logger().info("Found Start Sign")
            return # Not save Start Location

        # 2. Check HAZARD MARKERS
        if self.is_started:
            # Check Laser
            distance = self.current_scan.ranges[0] 

            if not math.isinf(distance) and not math.isnan(distance):
                # Copute location
                self.compute_and_store_hazard(obj_id, distance)

    def compute_and_store_hazard(self, obj_id, dist):
        try:
            # Check robot location
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, 
                                                    timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Robot rotation
            q = trans.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

            # Location of the obstuctor in robot
            obj_x_robot = dist * math.cos(0.0)
            obj_y_robot = dist * math.sin(0.0)

            # Change to map locations
            map_x = trans.transform.translation.x + (obj_x_robot * math.cos(yaw) - obj_y_robot * math.sin(yaw))
            map_y = trans.transform.translation.y + (obj_x_robot * math.sin(yaw) + obj_y_robot * math.cos(yaw))

            # Save and report if this sign has not trained
            if obj_id not in self.found_hazards:
                self.found_hazards[obj_id] = (map_x, map_y)
                self.get_logger().info(f"Found HAZARD ID {int(obj_id)} at: x={map_x:.2f}, y={map_y:.2f}")
                self.publish_markers() # publish topic /hazards

        except Exception as e:
            pass

    def publish_markers(self):
        marker_array = MarkerArray()
        for h_id, (x, y) in self.found_hazards.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.id = int(h_id)
            marker.type = Marker.CYLINDER 
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0 
            marker_array.markers.append(marker)
        
        self.marker_pub.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = HazardLocatorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()