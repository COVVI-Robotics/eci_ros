import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciErrorClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciError = f'{self.get_namespace()}/{service}/GetEciError'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciError}')
        self.client_GetEciError = self.create_client(
            covvi_interfaces.srv.GetEciError,
            full_service_name_GetEciError,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciError}')
        while not self.client_GetEciError.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciError} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciError}')
        
    @public
    def getEciError(self) -> bool:
        """Get the error status of the ECI"""
        request = covvi_interfaces.srv.GetEciError.Request()
        self.get_logger().info(f'Calling service GetEciError asynchronously')
        future = self.client_GetEciError.call_async(request)
        self.get_logger().info(f'Called service GetEciError asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciError is complete')
        response: covvi_interfaces.srv.GetEciError.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetEciErrorClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()