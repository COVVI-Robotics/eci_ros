import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciSerialNumberClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciSerialNumber = f'{self.get_namespace()}/{service}/GetEciSerialNumber'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciSerialNumber}')
        self.client_GetEciSerialNumber = self.create_client(
            covvi_interfaces.srv.GetEciSerialNumber,
            full_service_name_GetEciSerialNumber,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciSerialNumber}')
        while not self.client_GetEciSerialNumber.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciSerialNumber} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciSerialNumber}')
        
    @public
    def getEciSerialNumber(self) -> int:
        """Get the serial number of the ECI"""
        request = covvi_interfaces.srv.GetEciSerialNumber.Request()
        self.get_logger().info(f'Calling service GetEciSerialNumber asynchronously')
        future = self.client_GetEciSerialNumber.call_async(request)
        self.get_logger().info(f'Called service GetEciSerialNumber asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciSerialNumber is complete')
        response: covvi_interfaces.srv.GetEciSerialNumber.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEciSerialNumberClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()