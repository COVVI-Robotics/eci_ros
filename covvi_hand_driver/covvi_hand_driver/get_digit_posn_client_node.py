import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDigitPosnClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_digit_posn_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetDigitPosn = self._get_full_ros2_name('GetDigitPosn', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDigitPosn}')
        self.client_GetDigitPosn = self.create_client(
            covvi_interfaces.srv.GetDigitPosn,
            full_service_name_GetDigitPosn,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDigitPosn}')
        while not self.client_GetDigitPosn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDigitPosn} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDigitPosn}')
        
    @public
    def getDigitPosn(self,
        digit: covvi_interfaces.msg.Digit,
    ) -> covvi_interfaces.msg.DigitPosnMsg:
        """Get the digit position"""
        request = covvi_interfaces.srv.GetDigitPosn.Request()
        request.digit = digit
        self.get_logger().info(f'Calling service GetDigitPosn asynchronously')
        future = self.client_GetDigitPosn.call_async(request)
        self.get_logger().info(f'Called service GetDigitPosn asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDigitPosn is complete')
        response: covvi_interfaces.srv.GetDigitPosn.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_digit_posn_client_node = GetDigitPosnClientNode(service=service)
    rclpy.spin(covvi_get_digit_posn_client_node)
    covvi_get_digit_posn_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()