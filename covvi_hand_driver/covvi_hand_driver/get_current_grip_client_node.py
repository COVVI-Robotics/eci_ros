import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetCurrentGripClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetCurrentGrip = f'{self.get_namespace()}/{service}/GetCurrentGrip'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetCurrentGrip}')
        self.client_GetCurrentGrip = self.create_client(
            covvi_interfaces.srv.GetCurrentGrip,
            full_service_name_GetCurrentGrip,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetCurrentGrip}')
        while not self.client_GetCurrentGrip.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetCurrentGrip} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetCurrentGrip}')
        
    @public
    def getCurrentGrip(self) -> covvi_interfaces.msg.CurrentGrip:
        """Get the current grip config"""
        request = covvi_interfaces.srv.GetCurrentGrip.Request()
        self.get_logger().info(f'Calling service GetCurrentGrip asynchronously')
        future = self.client_GetCurrentGrip.call_async(request)
        self.get_logger().info(f'Called service GetCurrentGrip asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetCurrentGrip is complete')
        response: covvi_interfaces.srv.GetCurrentGrip.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetCurrentGripClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()