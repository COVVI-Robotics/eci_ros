import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciPowerOnClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciPowerOn = f'{self.get_namespace()}/{service}/GetEciPowerOn'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciPowerOn}')
        self.client_GetEciPowerOn = self.create_client(
            covvi_interfaces.srv.GetEciPowerOn,
            full_service_name_GetEciPowerOn,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciPowerOn}')
        while not self.client_GetEciPowerOn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciPowerOn} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciPowerOn}')
        
    @public
    def getEciPowerOn(self) -> bool:
        """Get the 'power on' status of the ECI"""
        request = covvi_interfaces.srv.GetEciPowerOn.Request()
        self.get_logger().info(f'Calling service GetEciPowerOn asynchronously')
        future = self.client_GetEciPowerOn.call_async(request)
        self.get_logger().info(f'Called service GetEciPowerOn asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciPowerOn is complete')
        response: covvi_interfaces.srv.GetEciPowerOn.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEciPowerOnClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()