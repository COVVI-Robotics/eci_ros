import sys
from typing import Callable, Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode


class DigitPosnAllMsgSubscriberNode(CovviBaseClientNode):
    def __init__(self, callback: Callable | None = None, service: str = '', node_name: str = 'covvi_digit_posn_all_msg_subscriber_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_publisher_name_DigitPosnAllMsg = self._get_full_ros2_name('DigitPosnAllMsg', service=service)
        self.get_logger().info(f'Creating ROS2 Subscriber: {full_publisher_name_DigitPosnAllMsg}')
        self.subscriber_DigitPosnAllMsg = self.create_subscription(
            covvi_interfaces.msg.DigitPosnAllMsg,
            full_publisher_name_DigitPosnAllMsg,
            callback if callback else self._default_callback,
            10,
        )
        self.get_logger().info(f'Created ROS2 Subscriber:  {full_publisher_name_DigitPosnAllMsg}')


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_digit_posn_all_msg_subscriber_node = DigitPosnAllMsgSubscriberNode(service=service)
    rclpy.spin(covvi_digit_posn_all_msg_subscriber_node)
    covvi_digit_posn_all_msg_subscriber_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()