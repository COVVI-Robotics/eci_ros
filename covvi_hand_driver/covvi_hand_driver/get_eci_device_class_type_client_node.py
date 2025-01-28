import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciDeviceClassTypeClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciDeviceClassType = f'{self.get_namespace()}/{service}/GetEciDeviceClassType'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciDeviceClassType}')
        self.client_GetEciDeviceClassType = self.create_client(
            covvi_interfaces.srv.GetEciDeviceClassType,
            full_service_name_GetEciDeviceClassType,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciDeviceClassType}')
        while not self.client_GetEciDeviceClassType.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciDeviceClassType} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciDeviceClassType}')
        
    @public
    def getEciDeviceClassType(self) -> covvi_interfaces.msg.DeviceClassType:
        """Get the 'device class' of the ECI"""
        request = covvi_interfaces.srv.GetEciDeviceClassType.Request()
        self.get_logger().info(f'Calling service GetEciDeviceClassType asynchronously')
        future = self.client_GetEciDeviceClassType.call_async(request)
        self.get_logger().info(f'Called service GetEciDeviceClassType asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciDeviceClassType is complete')
        response: covvi_interfaces.srv.GetEciDeviceClassType.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEciDeviceClassTypeClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()