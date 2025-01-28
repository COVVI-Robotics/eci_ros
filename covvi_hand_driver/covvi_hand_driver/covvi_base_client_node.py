from typing import Callable

from covvi_hand_driver.covvi_base_node import CovviBaseNode


def public(func: Callable) -> Callable:
    setattr(func, 'public', True)
    return func


class CovviBaseClientNode(CovviBaseNode):
    def __init__(self, service: str = '', **kwargs):
        assert service
        super().__init__(**kwargs)
        self.eci_service = service
        self.get_logger().info(f'Service: {service}')
        assert service