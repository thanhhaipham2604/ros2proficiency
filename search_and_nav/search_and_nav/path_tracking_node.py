import rclpy
from rclpy.node import Node
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped

class PathTrackingNode(Node):
    def __init__(self):
        super().__init__('path_tracking_node')
        self.path = []
        self.pub = self.create_publisher(Path, 'cmd_pose', 10)