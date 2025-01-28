import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciConnectedClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciConnected = f'{self.get_namespace()}/{service}/GetEciConnected'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciConnected}')
        self.client_GetEciConnected = self.create_client(
            covvi_interfaces.srv.GetEciConnected,
            full_service_name_GetEciConnected,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciConnected}')
        while not self.client_GetEciConnected.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciConnected} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciConnected}')
        
    @public
    def getEciConnected(self) -> bool:
        """Get the connected status of the ECI"""
        request = covvi_interfaces.srv.GetEciConnected.Request()
        self.get_logger().info(f'Calling service GetEciConnected asynchronously')
        future = self.client_GetEciConnected.call_async(request)
        self.get_logger().info(f'Called service GetEciConnected asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciConnected is complete')
        response: covvi_interfaces.srv.GetEciConnected.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEciConnectedClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()