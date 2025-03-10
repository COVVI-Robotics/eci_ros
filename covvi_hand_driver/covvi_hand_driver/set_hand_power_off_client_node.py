import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetHandPowerOffClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetHandPowerOff = f'{self.get_namespace()}/{service}/SetHandPowerOff'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetHandPowerOff}')
        self.client_SetHandPowerOff = self.create_client(
            covvi_interfaces.srv.SetHandPowerOff,
            full_service_name_SetHandPowerOff,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetHandPowerOff}')
        while not self.client_SetHandPowerOff.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetHandPowerOff} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetHandPowerOff}')
        
    @public
    def setHandPowerOff(self) -> covvi_interfaces.msg.HandPowerMsg:
        """Power off the hand"""
        request = covvi_interfaces.srv.SetHandPowerOff.Request()
        self.get_logger().info(f'Calling service SetHandPowerOff asynchronously')
        future = self.client_SetHandPowerOff.call_async(request)
        self.get_logger().info(f'Called service SetHandPowerOff asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetHandPowerOff is complete')
        response: covvi_interfaces.srv.SetHandPowerOff.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SetHandPowerOffClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()