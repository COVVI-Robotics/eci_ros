import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDeviceIdentityClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_device_identity_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetDeviceIdentity = self._get_full_ros2_name('GetDeviceIdentity', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDeviceIdentity}')
        self.client_GetDeviceIdentity = self.create_client(
            covvi_interfaces.srv.GetDeviceIdentity,
            full_service_name_GetDeviceIdentity,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDeviceIdentity}')
        while not self.client_GetDeviceIdentity.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDeviceIdentity} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDeviceIdentity}')
        
    @public
    def getDeviceIdentity(self) -> covvi_interfaces.msg.DeviceIdentityMsg:
        """"""
        request = covvi_interfaces.srv.GetDeviceIdentity.Request()
        self.get_logger().info(f'Calling service GetDeviceIdentity asynchronously')
        future = self.client_GetDeviceIdentity.call_async(request)
        self.get_logger().info(f'Called service GetDeviceIdentity asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDeviceIdentity is complete')
        response: covvi_interfaces.srv.GetDeviceIdentity.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_device_identity_client_node = GetDeviceIdentityClientNode(service=service)
    rclpy.spin(covvi_get_device_identity_client_node)
    covvi_get_device_identity_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()