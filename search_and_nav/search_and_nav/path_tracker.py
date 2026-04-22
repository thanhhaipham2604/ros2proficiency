import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from std_msgs.msg import String, Empty
from nav_msgs.msg import Path
from nav2_msgs.action import NavigateToPose

from search_and_nav.tf_utils import TFHelper
from search_and_nav.map_utils import simplify_pose_list


class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.declare_parameter('explore_sample_distance', 0.20)
        self.declare_parameter('return_waypoint_spacing', 0.30)
        self.declare_parameter('waypoint_reached_timeout', 20.0)

        self.state = 'WAITING_FOR_START'
        self.tf = TFHelper(self)

        self.explore_path = Path()
        self.return_path = Path()
        self.explore_path.header.frame_id = 'map'
        self.return_path.header.frame_id = 'map'

        self.path_explore_pub = self.create_publisher(Path, '/path_explore', 10)
        self.path_return_pub = self.create_publisher(Path, '/path_return', 10)
        self.return_complete_pub = self.create_publisher(Empty, '/snc/return_complete', 10)

        self.create_subscription(String, '/snc/mission_state', self.on_state, 10)
        self.create_subscription(Empty, '/snc/return_request', self.on_return_request, 10)

        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.timer = self.create_timer(0.5, self.on_timer)

        self.return_queue = []
        self.goal_in_progress = False
        self.goal_handle = None
        self.goal_start_time = None

    def on_state(self, msg: String):
        self.state = msg.data

    def on_timer(self):
        if self.state == 'EXPLORING':
            self.record_explore_pose()

        if self.state == 'RETURNING_HOME':
            self.drive_return_queue()

    def record_explore_pose(self):
        pose_tuple = self.tf.lookup_xy_yaw('map', 'base_link')
        if pose_tuple is None:
            return
        x, y, yaw = pose_tuple
        pose = TFHelper.make_pose_stamped(self, x, y, yaw, frame_id='map')

        if self.should_append(self.explore_path, pose):
            self.explore_path.header.stamp = self.get_clock().now().to_msg()
            self.explore_path.poses.append(pose)
            self.path_explore_pub.publish(self.explore_path)

    def should_append(self, path, pose):
        if not path.poses:
            return True

        last = path.poses[-1].pose.position
        curr = pose.pose.position
        dist = math.hypot(curr.x - last.x, curr.y - last.y)
        threshold = self.get_parameter('explore_sample_distance').value
        return dist >= threshold

    def on_return_request(self, _msg):
        if not self.explore_path.poses:
            self.get_logger().warn('No explore path recorded')
            self.return_complete_pub.publish(Empty())
            return

        reversed_poses = list(reversed(self.explore_path.poses))
        spacing = self.get_parameter('return_waypoint_spacing').value
        self.return_queue = simplify_pose_list(reversed_poses, min_spacing=spacing)

        self.return_path.header.frame_id = 'map'
        self.return_path.header.stamp = self.get_clock().now().to_msg()
        self.return_path.poses = self.return_queue.copy()
        self.path_return_pub.publish(self.return_path)

        self.get_logger().info(f'Return queue prepared with {len(self.return_queue)} waypoints')

    def drive_return_queue(self):
        if self.goal_in_progress:
            self.check_goal_timeout()
            return

        if not self.return_queue:
            self.get_logger().info('Return complete')
            self.return_complete_pub.publish(Empty())
            return

        next_pose = self.return_queue.pop(0)
        self.send_goal(next_pose)

    def send_goal(self, pose):
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('navigate_to_pose server unavailable')
            return

        goal = NavigateToPose.Goal()
        goal.pose = pose
        goal.behavior_tree = ''

        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_callback)
        self.goal_in_progress = True
        self.goal_start_time = self.get_clock().now()

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Return waypoint rejected')
            self.goal_in_progress = False
            return

        self.goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future):
        self.goal_in_progress = False
        try:
            status = future.result().status
            self.get_logger().info(f'Return waypoint finished with status={status}')
        except Exception as e:
            self.get_logger().warn(f'Return waypoint failed: {e}')

        self.goal_handle = None
        self.goal_start_time = None

    def check_goal_timeout(self):
        if self.goal_start_time is None or self.goal_handle is None:
            return
        timeout = self.get_parameter('waypoint_reached_timeout').value
        elapsed = (self.get_clock().now() - self.goal_start_time).nanoseconds / 1e9
        if elapsed >= timeout:
            self.get_logger().warn('Return waypoint timeout; cancelling and continuing')
            self.goal_handle.cancel_goal_async()
            self.goal_in_progress = False
            self.goal_handle = None
            self.goal_start_time = None


def main():
    rclpy.init()
    node = PathTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()