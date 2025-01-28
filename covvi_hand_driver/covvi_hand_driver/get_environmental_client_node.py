import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEnvironmentalClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEnvironmental = f'{self.get_namespace()}/{service}/GetEnvironmental'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEnvironmental}')
        self.client_GetEnvironmental = self.create_client(
            covvi_interfaces.srv.GetEnvironmental,
            full_service_name_GetEnvironmental,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEnvironmental}')
        while not self.client_GetEnvironmental.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEnvironmental} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEnvironmental}')
        
    @public
    def getEnvironmental(self) -> covvi_interfaces.msg.EnvironmentalMsg:
        """Read temperature, battery voltage etc

        Temperature     (C)
        Humidity        (0-100%)
        Battery Voltage (mV)
        """
        request = covvi_interfaces.srv.GetEnvironmental.Request()
        self.get_logger().info(f'Calling service GetEnvironmental asynchronously')
        future = self.client_GetEnvironmental.call_async(request)
        self.get_logger().info(f'Called service GetEnvironmental asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEnvironmental is complete')
        response: covvi_interfaces.srv.GetEnvironmental.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEnvironmentalClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()