import sys
from typing import Callable, Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode


class SystemStatusMsgSubscriberNode(CovviBaseClientNode):
    def __init__(self, callback: Callable | None = None, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_publisher_name_SystemStatusMsg = f'{self.get_namespace()}/{service}/SystemStatusMsg'
        self.get_logger().info(f'Creating ROS2 Subscriber: {full_publisher_name_SystemStatusMsg}')
        self.subscriber_SystemStatusMsg = self.create_subscription(
            covvi_interfaces.msg.SystemStatusMsg,
            full_publisher_name_SystemStatusMsg,
            callback if callback else self._default_callback,
            10,
        )
        self.get_logger().info(f'Created ROS2 Subscriber:  {full_publisher_name_SystemStatusMsg}')


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SystemStatusMsgSubscriberNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()