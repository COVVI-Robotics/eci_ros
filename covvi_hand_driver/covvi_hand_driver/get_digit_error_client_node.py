import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDigitErrorClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetDigitError = f'{self.get_namespace()}/{service}/GetDigitError'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDigitError}')
        self.client_GetDigitError = self.create_client(
            covvi_interfaces.srv.GetDigitError,
            full_service_name_GetDigitError,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDigitError}')
        while not self.client_GetDigitError.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDigitError} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDigitError}')
        
    @public
    def getDigitError(self,
        digit: covvi_interfaces.msg.Digit,
    ) -> covvi_interfaces.msg.DigitErrorMsg:
        """Get digit error flags"""
        request = covvi_interfaces.srv.GetDigitError.Request()
        request.digit = digit
        self.get_logger().info(f'Calling service GetDigitError asynchronously')
        future = self.client_GetDigitError.call_async(request)
        self.get_logger().info(f'Called service GetDigitError asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDigitError is complete')
        response: covvi_interfaces.srv.GetDigitError.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetDigitErrorClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()