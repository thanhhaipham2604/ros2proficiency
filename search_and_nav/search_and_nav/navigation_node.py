import math
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from std_msgs.msg import String, Empty
from nav_msgs.msg import OccupancyGrid, Path
from nav2_msgs.action import NavigateToPose

from par_snc.tf_utils import TFHelper
from par_snc.map_utils import extract_frontiers, cluster_frontiers, centroid_world


class NavigationNode(Node):

    def __init__(self):
        super().__init__('navigation_node')

        self.state = "WAITING"

        self.map = None
        self.path = Path()
        self.path.header.frame_id = "map"

        self.return_queue = []

        self.tf = TFHelper(self)
        self.nav = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.path_pub = self.create_publisher(Path, '/path_explore', 10)
        self.return_pub = self.create_publisher(Path, '/path_return', 10)

        self.create_subscription(OccupancyGrid, '/map', self.map_cb, 10)
        self.create_subscription(String, '/snc/mission_state', self.state_cb, 10)
        self.create_subscription(Empty, '/snc/return_request', self.return_cb, 10)

        self.timer = self.create_timer(2.0, self.loop)

    def map_cb(self, msg):
        self.map = msg

    def state_cb(self, msg):
        self.state = msg.data

    def loop(self):
        if self.state == "EXPLORING":
            self.explore()
            self.record_path()

        elif self.state == "RETURNING":
            self.return_home()

    # -------- EXPLORATION --------
    def explore(self):
        if self.map is None:
            return

        frontiers = extract_frontiers(self.map)
        clusters = cluster_frontiers(frontiers)

        if not clusters:
            return

        rx, ry, _ = self.tf.lookup_xy_yaw()

        best = None
        best_score = -1e9

        for c in clusters:
            x, y = centroid_world(c, self.map)
            d = math.hypot(x - rx, y - ry)
            score = len(c) - d

            if score > best_score:
                best_score = score
                best = (x, y)

        if best:
            self.send_goal(best[0], best[1], rx, ry)

    # -------- PATH RECORD --------
    def record_path(self):
        pose = self.tf.make_pose()
        if pose:
            self.path.poses.append(pose)
            self.path_pub.publish(self.path)

    # -------- RETURN --------
    def return_cb(self, _):
        self.return_queue = list(reversed(self.path.poses))
        self.return_pub.publish(self.path)

    def return_home(self):
        if not self.return_queue:
            return

        pose = self.return_queue.pop(0)
        self.send_pose_goal(pose)

    # -------- NAV2 --------
    def send_goal(self, x, y, rx, ry):
        yaw = math.atan2(y - ry, x - rx)
        pose = self.tf.make_pose_xy(x, y, yaw)
        self.send_pose_goal(pose)

    def send_pose_goal(self, pose):
        if not self.nav.wait_for_server(timeout_sec=1.0):
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.nav.send_goal_async(goal)


def main():
    rclpy.init()
    node = NavigationNode()
    rclpy.spin(node)
    rclpy.shutdown()