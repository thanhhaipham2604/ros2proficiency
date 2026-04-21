import math

import rclpy
from geometry_msgs.msg import PointStamped, PoseStamped
from tf2_ros import Buffer, TransformListener


class TFHelper:
    def __init__(self, node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)

    def lookup_xy_yaw(self, target_frame='map', source_frame='base_link'):
        try:
            transform = self.buffer.lookup_transform(
                target_frame,
                source_frame,
                rclpy.time.Time()
            )
            tx = transform.transform.translation.x
            ty = transform.transform.translation.y
            q = transform.transform.rotation
            yaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z)
            )
            return tx, ty, yaw
        except Exception as e:
            self.node.get_logger().warn(f'TF lookup_xy_yaw failed: {e}')
            return None

    def transform_point(self, point_msg: PointStamped, target_frame='map'):
        try:
            return self.buffer.transform(point_msg, target_frame)
        except Exception as e:
            self.node.get_logger().warn(f'TF transform_point failed: {e}')
            return None

    @staticmethod
    def make_pose_stamped(node, x, y, yaw, frame_id='map'):
        pose = PoseStamped()
        pose.header.frame_id = frame_id
        pose.header.stamp = node.get_clock().now().to_msg()
        pose.pose.position.x = float(x)
        pose.pose.position.y = float(y)
        pose.pose.position.z = 0.0
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        return pose