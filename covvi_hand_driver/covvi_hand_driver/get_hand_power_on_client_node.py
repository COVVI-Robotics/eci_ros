import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandPowerOnClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandPowerOn = f'{self.get_namespace()}/{service}/GetHandPowerOn'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandPowerOn}')
        self.client_GetHandPowerOn = self.create_client(
            covvi_interfaces.srv.GetHandPowerOn,
            full_service_name_GetHandPowerOn,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandPowerOn}')
        while not self.client_GetHandPowerOn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandPowerOn} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandPowerOn}')
        
    @public
    def getHandPowerOn(self) -> bool:
        """Get the 'power on' status of the Hand"""
        request = covvi_interfaces.srv.GetHandPowerOn.Request()
        self.get_logger().info(f'Calling service GetHandPowerOn asynchronously')
        future = self.client_GetHandPowerOn.call_async(request)
        self.get_logger().info(f'Called service GetHandPowerOn asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandPowerOn is complete')
        response: covvi_interfaces.srv.GetHandPowerOn.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetHandPowerOnClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()