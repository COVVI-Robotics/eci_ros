from typing import Any
from rclpy.node import Node


class CovviBaseNode(Node):
    def __init__(self, node_name: str = 'covvi_base_node', **kwargs):
        assert node_name
        super().__init__(node_name=node_name, **kwargs)

    def _get_full_ros2_name(self, name: str, service: str = '') -> str:
        return f'{self.get_namespace()}/{service}/{name}' if service else f'{self.get_namespace()}/{self.get_name()}/{name}'

    def _default_callback(self, item: Any) -> None:
        self.get_logger().info(str(item))