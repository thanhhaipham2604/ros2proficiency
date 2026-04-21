import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from nav_msgs.msg import OccupancyGrid


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_node')

        self.state = 'WAITING_FOR_START'
        self.latest_map = None

        self.create_subscription(String, '/snc/mission_state', self.on_state, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map, 10)

        self.timer = self.create_timer(2.0, self.on_timer)

    def on_state(self, msg: String):
        self.state = msg.data

    def on_map(self, msg: OccupancyGrid):
        self.latest_map = msg

    def on_timer(self):
        if self.state != 'EXPLORING':
            return
        if self.latest_map is None:
            self.get_logger().info('Waiting for map...')
            return

        # TODO:
        # 1. Extract frontier cells
        # 2. Cluster frontiers
        # 3. Score candidate clusters
        # 4. Send best goal to Nav2
        self.get_logger().info('Exploration tick')


def main():
    rclpy.init()
    node = ExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()