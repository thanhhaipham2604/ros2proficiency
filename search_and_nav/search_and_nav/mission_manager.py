import enum

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class MissionState(enum.Enum):
    WAITING_FOR_START = 'WAITING_FOR_START'
    EXPLORING = 'EXPLORING'
    RETURNING_HOME = 'RETURNING_HOME'
    TELEOP_MODE = 'TELEOP_MODE'
    FINISHED = 'FINISHED'


class MissionManager(Node):
    def __init__(self):
        super().__init__('mission_manager')

        self.declare_parameter('total_expected_hazards', 5)
        self.declare_parameter('auto_return_after_seconds', 240.0)

        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.state_pub = self.create_publisher(String, '/snc/mission_state', 10)
        self.return_request_pub = self.create_publisher(Empty, '/snc/return_request', 10)

        self.create_subscription(Empty, '/trigger_start', self.on_trigger_start, 10)
        self.create_subscription(Empty, '/trigger_home', self.on_trigger_home, 10)
        self.create_subscription(Empty, '/trigger_teleop', self.on_trigger_teleop, 10)
        self.create_subscription(String, '/snc/hazard_found', self.on_hazard_found, 10)
        self.create_subscription(Empty, '/snc/return_complete', self.on_return_complete, 10)
        self.create_subscription(Empty, '/snc/start_detected', self.on_trigger_start, 10)

        self.state = MissionState.WAITING_FOR_START
        self.found_hazards = set()
        self.start_time = None

        self.timer = self.create_timer(1.0, self.on_timer)
        self.publish_state()

    def publish_state(self):
        s = String()
        s.data = self.state.value
        self.state_pub.publish(s)

        status = String()
        status.data = f'{self.state.value} | hazards_found={len(self.found_hazards)}'
        self.status_pub.publish(status)

    def on_trigger_start(self, _msg):
        if self.state == MissionState.WAITING_FOR_START:
            self.state = MissionState.EXPLORING
            self.start_time = self.get_clock().now()
            self.get_logger().info('Mission started')
            self.publish_state()

    def on_trigger_home(self, _msg):
        if self.state in [MissionState.EXPLORING, MissionState.TELEOP_MODE]:
            self.state = MissionState.RETURNING_HOME
            self.return_request_pub.publish(Empty())
            self.publish_state()

    def on_trigger_teleop(self, _msg):
        self.state = MissionState.TELEOP_MODE
        self.publish_state()

    def on_hazard_found(self, msg: String):
        self.found_hazards.add(msg.data)
        expected = self.get_parameter('total_expected_hazards').value
        if len(self.found_hazards) >= expected and self.state == MissionState.EXPLORING:
            self.state = MissionState.RETURNING_HOME
            self.return_request_pub.publish(Empty())
            self.publish_state()

    def on_return_complete(self, _msg):
        self.state = MissionState.FINISHED
        self.publish_state()

    def on_timer(self):
        if self.state != MissionState.EXPLORING or self.start_time is None:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        limit = self.get_parameter('auto_return_after_seconds').value
        if elapsed >= limit:
            self.get_logger().info('Auto-return timer reached')
            self.state = MissionState.RETURNING_HOME
            self.return_request_pub.publish(Empty())
            self.publish_state()


def main():
    rclpy.init()
    node = MissionManager()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()