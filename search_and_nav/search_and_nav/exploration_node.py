import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import NavigateToPose

from search_and_nav.tf_utils import TFHelper
from search_and_nav.map_utils import extract_frontiers, cluster_frontiers, centroid_world


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.declare_parameter('frontier_min_cluster_size', 8)
        self.declare_parameter('goal_blacklist_timeout', 30.0)
        self.declare_parameter('goal_timeout_sec', 90.0)
        self.declare_parameter('timer_period', 2.0)
        self.declare_parameter('min_goal_distance', 0.5)

        self.state = 'WAITING_FOR_START'
        self.latest_map = None
        self.goal_in_progress = False
        self.current_goal = None
        self.goal_handle = None
        self.result_future = None
        self.goal_start_time = None
        self.blacklisted_goals = []

        self.tf = TFHelper(self)
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_subscription(String, '/snc/mission_state', self.on_state, 10)

        map_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            depth=1
        )
        self.create_subscription(OccupancyGrid, '/map', self.on_map, map_qos)

        self.frontier_pub = self.create_publisher(MarkerArray, '/snc/debug/frontiers', 10)
        self.goal_pub = self.create_publisher(Marker, '/snc/debug/frontier_goal', 10)

        period = self.get_parameter('timer_period').value
        self.timer = self.create_timer(period, self.on_timer)

    def on_state(self, msg: String):
        self.state = msg.data
        if self.state != 'EXPLORING' and self.goal_in_progress:
            self.cancel_goal()

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def on_timer(self):
        if self.state != 'EXPLORING':
            return

        if self.latest_map is None:
            self.get_logger().info('Waiting for /map')
            return

        if self.goal_in_progress:
            self.check_goal_timeout()
            return

        robot_pose = self.tf.lookup_xy_yaw('map', 'base_link')
        if robot_pose is None:
            return

        rx, ry, _ = robot_pose

        frontiers = extract_frontiers(self.latest_map)
        clusters = cluster_frontiers(frontiers)

        min_cluster = self.get_parameter('frontier_min_cluster_size').value
        clusters = [c for c in clusters if len(c) >= min_cluster]
        self.publish_frontier_markers(clusters)

        if not clusters:
            self.get_logger().info('No frontier clusters available')
            return

        best_goal = self.select_best_goal(clusters, rx, ry)
        if best_goal is None:
            self.get_logger().info('No valid frontier goal found')
            return

        gx, gy = best_goal
        self.send_nav_goal(gx, gy, rx, ry)

    def select_best_goal(self, clusters, rx, ry):
        best_score = -1e9
        best = None
        min_goal_distance = self.get_parameter('min_goal_distance').value

        for cluster in clusters:
            cx, cy = centroid_world(cluster, self.latest_map)

            if self.is_blacklisted(cx, cy):
                continue

            dist = math.hypot(cx - rx, cy - ry)
            if dist < min_goal_distance:
                continue

            # Larger cluster = better, closer = better
            score = 1.5 * float(len(cluster)) - 2.0 * dist

            if score > best_score:
                best_score = score
                best = (cx, cy)

        return best

    def send_nav_goal(self, x, y, rx, ry):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose server unavailable')
            return

        yaw = math.atan2(y - ry, x - rx)
        goal = NavigateToPose.Goal()
        goal.pose = TFHelper.make_pose_stamped(self, x, y, yaw, frame_id='map')
        goal.behavior_tree = ''

        self.publish_goal_marker(x, y)

        self.get_logger().info(f'Sending frontier goal ({x:.2f}, {y:.2f})')
        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)

        self.goal_in_progress = True
        self.current_goal = (x, y)
        self.goal_start_time = self.get_clock().now()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Goal rejected')
            self.goal_in_progress = False
            if self.current_goal is not None:
                self.blacklist_goal(*self.current_goal)
            self.current_goal = None
            return

        self.goal_handle = goal_handle
        self.result_future = goal_handle.get_result_async()
        self.result_future.add_done_callback(self.goal_result_callback)
        self.get_logger().info('Goal accepted')

    def goal_result_callback(self, future):
        self.goal_in_progress = False
        try:
            result = future.result()
            status = result.status
        except Exception as e:
            self.get_logger().warn(f'Goal failed: {e}')
            status = None

        if status == 4:
            self.get_logger().info('Goal succeeded')
        else:
            self.get_logger().warn(f'Goal ended with status={status}')
            if self.current_goal is not None:
                self.blacklist_goal(*self.current_goal)

        self.goal_handle = None
        self.result_future = None
        self.goal_start_time = None
        self.current_goal = None

    def feedback_callback(self, feedback_msg):
        try:
            dist = feedback_msg.feedback.distance_remaining
            self.get_logger().debug(f'Distance remaining: {dist:.2f}')
        except Exception:
            pass

    def cancel_goal(self):
        if self.goal_handle is not None:
            self.goal_handle.cancel_goal_async()

        self.goal_in_progress = False
        self.goal_handle = None
        self.result_future = None
        self.goal_start_time = None
        self.current_goal = None

    def check_goal_timeout(self):
        if self.goal_start_time is None or self.current_goal is None:
            return

        timeout = self.get_parameter('goal_timeout_sec').value
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9

        if elapsed >= timeout:
            self.get_logger().warn('Goal timeout, cancelling')
            self.blacklist_goal(*self.current_goal)
            self.cancel_goal()

    def blacklist_goal(self, x, y):
        expiry = (
            self.get_clock().now().nanoseconds / 1e9
            + self.get_parameter('goal_blacklist_timeout').value
        )
        self.blacklisted_goals.append((x, y, expiry))

    def is_blacklisted(self, x, y, threshold=0.7):
        now = self.get_clock().now().nanoseconds / 1e9
        self.blacklisted_goals = [g for g in self.blacklisted_goals if g[2] > now]

        for bx, by, _ in self.blacklisted_goals:
            if math.hypot(x - bx, y - by) < threshold:
                return True
        return False

    def publish_frontier_markers(self, clusters):
        arr = MarkerArray()
        stamp = self.get_clock().now().to_msg()

        for i, cluster in enumerate(clusters):
            x, y = centroid_world(cluster, self.latest_map)

            m = Marker()
            m.header.frame_id = 'map'
            m.header.stamp = stamp
            m.ns = 'frontiers'
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = x
            m.pose.position.y = y
            m.pose.position.z = 0.08
            m.pose.orientation.w = 1.0
            m.scale.x = 0.15
            m.scale.y = 0.15
            m.scale.z = 0.15
            m.color.a = 0.8
            m.color.r = 0.0
            m.color.g = 1.0
            m.color.b = 0.0
            arr.markers.append(m)

        self.frontier_pub.publish(arr)

    def publish_goal_marker(self, x, y):
        m = Marker()
        m.header.frame_id = 'map'
        m.header.stamp = self.get_clock().now().to_msg()
        m.ns = 'frontier_goal'
        m.id = 0
        m.type = Marker.SPHERE
        m.action = Marker.ADD
        m.pose.position.x = x
        m.pose.position.y = y
        m.pose.position.z = 0.12
        m.pose.orientation.w = 1.0
        m.scale.x = 0.22
        m.scale.y = 0.22
        m.scale.z = 0.22
        m.color.a = 0.9
        m.color.r = 1.0
        m.color.g = 0.0
        m.color.b = 0.0
        self.goal_pub.publish(m)


def main():
    rclpy.init()
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()