import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String


class StartDetector(Node):
    def __init__(self):
        super().__init__('start_detector')

        self.start_pub = self.create_publisher(Empty, '/snc/start_detected', 10)
        self.create_subscription(String, '/snc/start_marker_detection', self.on_start_marker, 10)

    def on_start_marker(self, msg: String):
        if msg.data.strip().lower() == 'start':
            self.get_logger().info('Start marker detected')
            self.start_pub.publish(Empty())


def main():
    rclpy.init()
    node = StartDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()