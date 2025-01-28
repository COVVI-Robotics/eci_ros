import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHelloClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHello = f'{self.get_namespace()}/{service}/GetHello'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHello}')
        self.client_GetHello = self.create_client(
            covvi_interfaces.srv.GetHello,
            full_service_name_GetHello,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHello}')
        while not self.client_GetHello.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHello} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHello}')
        
    @public
    def getHello(self) -> covvi_interfaces.msg.HelloMsg:
        """"""
        request = covvi_interfaces.srv.GetHello.Request()
        self.get_logger().info(f'Calling service GetHello asynchronously')
        future = self.client_GetHello.call_async(request)
        self.get_logger().info(f'Called service GetHello asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHello is complete')
        response: covvi_interfaces.srv.GetHello.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetHelloClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()