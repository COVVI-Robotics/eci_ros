import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetHandPowerOnClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetHandPowerOn = f'{self.get_namespace()}/{service}/SetHandPowerOn'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetHandPowerOn}')
        self.client_SetHandPowerOn = self.create_client(
            covvi_interfaces.srv.SetHandPowerOn,
            full_service_name_SetHandPowerOn,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetHandPowerOn}')
        while not self.client_SetHandPowerOn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetHandPowerOn} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetHandPowerOn}')
        
    @public
    def setHandPowerOn(self) -> covvi_interfaces.msg.HandPowerMsg:
        """Power on the hand"""
        request = covvi_interfaces.srv.SetHandPowerOn.Request()
        self.get_logger().info(f'Calling service SetHandPowerOn asynchronously')
        future = self.client_SetHandPowerOn.call_async(request)
        self.get_logger().info(f'Called service SetHandPowerOn asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetHandPowerOn is complete')
        response: covvi_interfaces.srv.SetHandPowerOn.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = SetHandPowerOnClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()