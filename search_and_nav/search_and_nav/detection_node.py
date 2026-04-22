#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, Empty
from visualization_msgs.msg import Marker, MarkerArray 
from tf2_ros import TransformListener, Buffer
import math

class HazardLocatorNode(Node):
    def __init__(self):
        super().__init__('hazard_locator_node')
        
        self.START_MARKER_ID = 13.0 
        self.is_started = False
        self.found_hazards = {} 
        self.current_scan = None
        
        # --- TF & SUBSCRIBERS ---
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # FIXED: Subscription topic updated to match find_object_2d output
        self.obj_sub = self.create_subscription(
            Float32MultiArray, '/find_object_2d/objects', self.object_callback, 10)
            
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)

        # --- PUBLISHERS ---
        self.marker_pub = self.create_publisher(MarkerArray, '/hazards', 10)
        
        # Trigger for the mission_node/manager
        self.start_pub = self.create_publisher(Empty, '/snc/start_detected', 10)
            
        self.get_logger().info("Hazard Locator Node is ready and listening to /find_object_2d/objects")

    def scan_callback(self, msg):
        self.current_scan = msg

    def object_callback(self, msg):
        if len(msg.data) == 0:
            return

        obj_id = msg.data[0]

        # 1. Check START (Move this ABOVE the scan check)
        if obj_id == self.START_MARKER_ID:
            if not self.is_started:
                self.is_started = True
                self.get_logger().info("!!! START MARKER DETECTED - Triggering Exploration !!!")
                self.start_pub.publish(Empty())
            return 

        # 2. Check HAZARD MARKERS (Only if started AND scan is available)
        if self.is_started:
            if self.current_scan is None:
                self.get_logger().warn("Marker seen, but waiting for LaserScan data...")
                return
                
            distance = self.get_range_for_bearing(0.0) 
            if distance is not None:
                self.compute_and_store_hazard(obj_id, distance)

        obj_id = msg.data[0]

        # 1. Check START (ID 13)
        if obj_id == self.START_MARKER_ID:
            if not self.is_started:
                self.is_started = True
                self.get_logger().info("!!! START MARKER DETECTED - Triggering Exploration !!!")
                self.start_pub.publish(Empty())
            return 

        # 2. Check HAZARD MARKERS (IDs 1-12)
        if self.is_started:
            # We assume the object is centered in the camera view (0.0 rad)
            distance = self.get_range_for_bearing(0.0) 
            if distance is not None:
                self.compute_and_store_hazard(obj_id, distance)

    def get_range_for_bearing(self, bearing_rad):
        scan = self.current_scan
        if scan is None:
            return None

        # Find index in lidar array
        index = int((bearing_rad - scan.angle_min) / scan.angle_increment)
        # Use a small window to avoid noise
        window = range(max(0, index - 2), min(len(scan.ranges), index + 3))

        vals = [
            scan.ranges[i] for i in window
            if math.isfinite(scan.ranges[i]) and scan.range_min < scan.ranges[i] < scan.range_max
        ]

        if not vals:
            return None

        vals.sort()
        return vals[len(vals) // 2] # Return median distance

    def compute_and_store_hazard(self, obj_id, dist):
        try:
            # Get robot transform in the map
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now, 
                                                    timeout=rclpy.duration.Duration(seconds=0.1))
            
            # Robot heading (yaw)
            q = trans.transform.rotation
            yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))

            # Transform relative object position to global map position
            map_x = trans.transform.translation.x + (dist * math.cos(yaw))
            map_y = trans.transform.translation.y + (dist * math.sin(yaw))

            # Save if it's a new unique hazard
            if obj_id not in self.found_hazards:
                self.found_hazards[obj_id] = (map_x, map_y)
                self.get_logger().info(f"Recorded HAZARD ID {int(obj_id)} at: x={map_x:.2f}, y={map_y:.2f}")
                self.publish_markers()

        except Exception as e:
            # Silently fail if TF isn't ready yet
            pass

    def publish_markers(self):
        marker_array = MarkerArray()
        for h_id, (x, y) in self.found_hazards.items():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = int(h_id)
            marker.type = Marker.CYLINDER 
            marker.action = Marker.ADD
            marker.pose.position.x = x
            marker.pose.position.y = y
            marker.pose.position.z = 0.5
            marker.scale.x = 0.3
            marker.scale.y = 0.3
            marker.scale.z = 0.5
            marker.color.a = 1.0
            marker.color.r = 1.0 # Red markers for hazards
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