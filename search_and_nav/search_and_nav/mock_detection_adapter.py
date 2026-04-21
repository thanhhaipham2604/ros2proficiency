import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MockDetectionAdapter(Node):
    def __init__(self):
        super().__init__('mock_detection_adapter')

        self.pub = self.create_publisher(String, '/snc/detection', 10)

        self.get_logger().info(
            'Publish test detections with:\n'
            'ros2 topic pub /snc/mock_input std_msgs/msg/String "{data: \'3,-10.0\'}" -1'
        )

        self.create_subscription(String, '/snc/mock_input', self.on_input, 10)

    def on_input(self, msg: String):
        self.pub.publish(msg)
        self.get_logger().info(f'Forwarded detection: {msg.data}')


def main():
    rclpy.init()
    node = MockDetectionAdapter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()