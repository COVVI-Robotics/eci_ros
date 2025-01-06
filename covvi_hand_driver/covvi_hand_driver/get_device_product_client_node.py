import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDeviceProductClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_device_product_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetDeviceProduct = self._get_full_ros2_name('GetDeviceProduct', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDeviceProduct}')
        self.client_GetDeviceProduct = self.create_client(
            covvi_interfaces.srv.GetDeviceProduct,
            full_service_name_GetDeviceProduct,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDeviceProduct}')
        while not self.client_GetDeviceProduct.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDeviceProduct} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDeviceProduct}')
        
    @public
    def getDeviceProduct(self) -> covvi_interfaces.msg.DeviceProductMsg:
        """"""
        request = covvi_interfaces.srv.GetDeviceProduct.Request()
        self.get_logger().info(f'Calling service GetDeviceProduct asynchronously')
        future = self.client_GetDeviceProduct.call_async(request)
        self.get_logger().info(f'Called service GetDeviceProduct asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDeviceProduct is complete')
        response: covvi_interfaces.srv.GetDeviceProduct.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_device_product_client_node = GetDeviceProductClientNode(service=service)
    rclpy.spin(covvi_get_device_product_client_node)
    covvi_get_device_product_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()