#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformListener, Buffer
import math

class HazardLocatorNode(Node):
    def __init__(self):
        super().__init__('hazard_locator_node')
        
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        

        self.obj_sub = self.create_subscription(
            Float32MultiArray,
            '/objects',
            self.object_callback,
            10)
        self.get_logger().info("Node is listening to find_object_2d...")
            

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10)
            
        self.current_scan = None

    def scan_callback(self, msg):
        self.current_scan = msg

    def object_callback(self, msg):
        if len(msg.data) == 0 or self.current_scan is None:
            return


        
        angle_offset = 0.0 

        distance = self.current_scan.ranges[0] 

        if not math.isinf(distance):
            self.get_logger().info(f"Detect object from {distance}m")
            self.compute_map_coordinates(distance, angle_offset)

    def compute_map_coordinates(self, dist, angle):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform('map', 'base_link', now)
            

            obj_x_robot = dist * math.cos(angle)
            obj_y_robot = dist * math.sin(angle)
            

            self.get_logger().info(f"Detect object in the map!")
            
        except Exception as e:
            self.get_logger().error(f"Error TF: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = HazardLocatorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()