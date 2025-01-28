import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandDeviceClassTypeClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandDeviceClassType = f'{self.get_namespace()}/{service}/GetHandDeviceClassType'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandDeviceClassType}')
        self.client_GetHandDeviceClassType = self.create_client(
            covvi_interfaces.srv.GetHandDeviceClassType,
            full_service_name_GetHandDeviceClassType,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandDeviceClassType}')
        while not self.client_GetHandDeviceClassType.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandDeviceClassType} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandDeviceClassType}')
        
    @public
    def getHandDeviceClassType(self) -> covvi_interfaces.msg.DeviceClassType:
        """Get the 'device class' of the Hand"""
        request = covvi_interfaces.srv.GetHandDeviceClassType.Request()
        self.get_logger().info(f'Calling service GetHandDeviceClassType asynchronously')
        future = self.client_GetHandDeviceClassType.call_async(request)
        self.get_logger().info(f'Called service GetHandDeviceClassType asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandDeviceClassType is complete')
        response: covvi_interfaces.srv.GetHandDeviceClassType.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetHandDeviceClassTypeClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()