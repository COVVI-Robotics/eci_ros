import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetPinchConfigClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetPinchConfig = f'{self.get_namespace()}/{service}/GetPinchConfig'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetPinchConfig}')
        self.client_GetPinchConfig = self.create_client(
            covvi_interfaces.srv.GetPinchConfig,
            full_service_name_GetPinchConfig,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetPinchConfig}')
        while not self.client_GetPinchConfig.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetPinchConfig} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetPinchConfig}')
        
    @public
    def getPinchConfig(self) -> covvi_interfaces.msg.PinchConfigMsg:
        """Get pinch points"""
        request = covvi_interfaces.srv.GetPinchConfig.Request()
        self.get_logger().info(f'Calling service GetPinchConfig asynchronously')
        future = self.client_GetPinchConfig.call_async(request)
        self.get_logger().info(f'Called service GetPinchConfig asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetPinchConfig is complete')
        response: covvi_interfaces.srv.GetPinchConfig.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetPinchConfigClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()