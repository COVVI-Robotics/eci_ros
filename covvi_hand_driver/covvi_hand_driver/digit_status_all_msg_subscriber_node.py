import sys
from typing import Callable, Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode


class DigitStatusAllMsgSubscriberNode(CovviBaseClientNode):
    def __init__(self, callback: Callable | None = None, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_publisher_name_DigitStatusAllMsg = f'{self.get_namespace()}/{service}/DigitStatusAllMsg'
        self.get_logger().info(f'Creating ROS2 Subscriber: {full_publisher_name_DigitStatusAllMsg}')
        self.subscriber_DigitStatusAllMsg = self.create_subscription(
            covvi_interfaces.msg.DigitStatusAllMsg,
            full_publisher_name_DigitStatusAllMsg,
            callback if callback else self._default_callback,
            10,
        )
        self.get_logger().info(f'Created ROS2 Subscriber:  {full_publisher_name_DigitStatusAllMsg}')


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = DigitStatusAllMsgSubscriberNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()