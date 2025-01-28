import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandErrorClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandError = f'{self.get_namespace()}/{service}/GetHandError'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandError}')
        self.client_GetHandError = self.create_client(
            covvi_interfaces.srv.GetHandError,
            full_service_name_GetHandError,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandError}')
        while not self.client_GetHandError.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandError} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandError}')
        
    @public
    def getHandError(self) -> bool:
        """Get the error status of the Hand"""
        request = covvi_interfaces.srv.GetHandError.Request()
        self.get_logger().info(f'Calling service GetHandError asynchronously')
        future = self.client_GetHandError.call_async(request)
        self.get_logger().info(f'Called service GetHandError asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandError is complete')
        response: covvi_interfaces.srv.GetHandError.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetHandErrorClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()