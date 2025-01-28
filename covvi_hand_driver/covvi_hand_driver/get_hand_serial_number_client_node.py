import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandSerialNumberClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandSerialNumber = f'{self.get_namespace()}/{service}/GetHandSerialNumber'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandSerialNumber}')
        self.client_GetHandSerialNumber = self.create_client(
            covvi_interfaces.srv.GetHandSerialNumber,
            full_service_name_GetHandSerialNumber,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandSerialNumber}')
        while not self.client_GetHandSerialNumber.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandSerialNumber} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandSerialNumber}')
        
    @public
    def getHandSerialNumber(self) -> int:
        """Get the serial number of the Hand"""
        request = covvi_interfaces.srv.GetHandSerialNumber.Request()
        self.get_logger().info(f'Calling service GetHandSerialNumber asynchronously')
        future = self.client_GetHandSerialNumber.call_async(request)
        self.get_logger().info(f'Called service GetHandSerialNumber asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandSerialNumber is complete')
        response: covvi_interfaces.srv.GetHandSerialNumber.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetHandSerialNumberClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()