import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandConnectedClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandConnected = f'{self.get_namespace()}/{service}/GetHandConnected'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandConnected}')
        self.client_GetHandConnected = self.create_client(
            covvi_interfaces.srv.GetHandConnected,
            full_service_name_GetHandConnected,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandConnected}')
        while not self.client_GetHandConnected.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandConnected} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandConnected}')
        
    @public
    def getHandConnected(self) -> bool:
        """Get the connected status of the Hand"""
        request = covvi_interfaces.srv.GetHandConnected.Request()
        self.get_logger().info(f'Calling service GetHandConnected asynchronously')
        future = self.client_GetHandConnected.call_async(request)
        self.get_logger().info(f'Called service GetHandConnected asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandConnected is complete')
        response: covvi_interfaces.srv.GetHandConnected.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetHandConnectedClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()