import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDigitStatusAllClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetDigitStatusAll = f'{self.get_namespace()}/{service}/GetDigitStatusAll'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDigitStatusAll}')
        self.client_GetDigitStatusAll = self.create_client(
            covvi_interfaces.srv.GetDigitStatusAll,
            full_service_name_GetDigitStatusAll,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDigitStatusAll}')
        while not self.client_GetDigitStatusAll.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDigitStatusAll} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDigitStatusAll}')
        
    @public
    def getDigitStatus_all(self) -> covvi_interfaces.msg.DigitStatusAllMsg:
        """Get all digit status flags"""
        request = covvi_interfaces.srv.GetDigitStatusAll.Request()
        self.get_logger().info(f'Calling service GetDigitStatusAll asynchronously')
        future = self.client_GetDigitStatusAll.call_async(request)
        self.get_logger().info(f'Called service GetDigitStatusAll asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDigitStatusAll is complete')
        response: covvi_interfaces.srv.GetDigitStatusAll.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetDigitStatusAllClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()