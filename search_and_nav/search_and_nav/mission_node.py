import enum
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Empty


class MissionState(enum.Enum):
    WAITING = "WAITING"
    EXPLORING = "EXPLORING"
    RETURNING = "RETURNING"
    FINISHED = "FINISHED"


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        self.declare_parameter('total_expected_hazards', 5)

        self.state = MissionState.WAITING
        self.found = set()

        self.state_pub = self.create_publisher(String, '/snc/mission_state', 10)
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        self.return_pub = self.create_publisher(Empty, '/snc/return_request', 10)

        self.create_subscription(Empty, '/trigger_start', self.start_cb, 10)
        self.create_subscription(Empty, '/trigger_home', self.home_cb, 10)
        self.create_subscription(String, '/snc/hazard_found', self.hazard_cb, 10)

        self.timer = self.create_timer(1.0, self.loop)

    def publish(self):
        s = String()
        s.data = self.state.value
        self.state_pub.publish(s)

        st = String()
        st.data = f"{self.state.value} | hazards={len(self.found)}"
        self.status_pub.publish(st)

    def start_cb(self, _):
        if self.state == MissionState.WAITING:
            self.state = MissionState.EXPLORING

    def home_cb(self, _):
        self.state = MissionState.RETURNING
        self.return_pub.publish(Empty())

    def hazard_cb(self, msg):
        self.found.add(msg.data)
        total = self.get_parameter('total_expected_hazards').value

        if len(self.found) >= total:
            self.state = MissionState.RETURNING
            self.return_pub.publish(Empty())

    def loop(self):
        self.publish()


def main():
    rclpy.init()
    node = MissionNode()
    rclpy.spin(node)
    rclpy.shutdown()