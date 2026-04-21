from tf2_ros import Buffer, TransformListener


class TFHelper:
    def __init__(self, node):
        self.node = node
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, node)