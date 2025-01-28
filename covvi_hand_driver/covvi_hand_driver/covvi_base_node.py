from typing import Any
from rclpy.node import Node


class CovviBaseNode(Node):
    def __init__(self, node_name: str = 'noname', **kwargs):
        super().__init__(node_name, **kwargs)

    def _default_callback(self, item: Any) -> None:
        self.get_logger().info(str(item))