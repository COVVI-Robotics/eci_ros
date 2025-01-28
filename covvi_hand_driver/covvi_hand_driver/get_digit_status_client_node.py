import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDigitStatusClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetDigitStatus = f'{self.get_namespace()}/{service}/GetDigitStatus'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDigitStatus}')
        self.client_GetDigitStatus = self.create_client(
            covvi_interfaces.srv.GetDigitStatus,
            full_service_name_GetDigitStatus,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDigitStatus}')
        while not self.client_GetDigitStatus.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDigitStatus} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDigitStatus}')
        
    @public
    def getDigitStatus(self,
        digit: covvi_interfaces.msg.Digit,
    ) -> covvi_interfaces.msg.DigitStatusMsg:
        """Get the digit status flags"""
        request = covvi_interfaces.srv.GetDigitStatus.Request()
        request.digit = digit
        self.get_logger().info(f'Calling service GetDigitStatus asynchronously')
        future = self.client_GetDigitStatus.call_async(request)
        self.get_logger().info(f'Called service GetDigitStatus asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDigitStatus is complete')
        response: covvi_interfaces.srv.GetDigitStatus.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetDigitStatusClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()