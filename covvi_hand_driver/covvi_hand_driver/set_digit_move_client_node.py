import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDigitMoveClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetDigitMove = f'{self.get_namespace()}/{service}/SetDigitMove'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDigitMove}')
        self.client_SetDigitMove = self.create_client(
            covvi_interfaces.srv.SetDigitMove,
            full_service_name_SetDigitMove,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDigitMove}')
        while not self.client_SetDigitMove.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDigitMove} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDigitMove}')
        
    @public
    def setDigitMove(self,
        digit:    covvi_interfaces.msg.Digit,
        position: int,
        speed:    covvi_interfaces.msg.Speed,
        power:    covvi_interfaces.msg.Percentage,
        limit:    covvi_interfaces.msg.Percentage,
    ) -> covvi_interfaces.msg.DigitMoveMsg:
        """Command to move a single digit"""
        request = covvi_interfaces.srv.SetDigitMove.Request()
        request.digit    = digit
        request.position = position
        request.speed    = speed
        request.power    = power
        request.limit    = limit
        self.get_logger().info(f'Calling service SetDigitMove asynchronously')
        future = self.client_SetDigitMove.call_async(request)
        self.get_logger().info(f'Called service SetDigitMove asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDigitMove is complete')
        response: covvi_interfaces.srv.SetDigitMove.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = SetDigitMoveClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()