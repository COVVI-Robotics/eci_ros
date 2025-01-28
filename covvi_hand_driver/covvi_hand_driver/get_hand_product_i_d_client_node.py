import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandProductIDClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandProductID = f'{self.get_namespace()}/{service}/GetHandProductID'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandProductID}')
        self.client_GetHandProductID = self.create_client(
            covvi_interfaces.srv.GetHandProductID,
            full_service_name_GetHandProductID,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandProductID}')
        while not self.client_GetHandProductID.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandProductID} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandProductID}')
        
    @public
    def getHandProductID(self) -> covvi_interfaces.msg.Product:
        """Get the product ID of the Hand"""
        request = covvi_interfaces.srv.GetHandProductID.Request()
        self.get_logger().info(f'Calling service GetHandProductID asynchronously')
        future = self.client_GetHandProductID.call_async(request)
        self.get_logger().info(f'Called service GetHandProductID asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandProductID is complete')
        response: covvi_interfaces.srv.GetHandProductID.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetHandProductIDClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()