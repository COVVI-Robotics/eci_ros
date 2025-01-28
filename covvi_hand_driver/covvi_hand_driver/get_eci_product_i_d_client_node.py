import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciProductIDClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciProductID = f'{self.get_namespace()}/{service}/GetEciProductID'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciProductID}')
        self.client_GetEciProductID = self.create_client(
            covvi_interfaces.srv.GetEciProductID,
            full_service_name_GetEciProductID,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciProductID}')
        while not self.client_GetEciProductID.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciProductID} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciProductID}')
        
    @public
    def getEciProductID(self) -> covvi_interfaces.msg.Product:
        """Get the product ID of the ECI"""
        request = covvi_interfaces.srv.GetEciProductID.Request()
        self.get_logger().info(f'Calling service GetEciProductID asynchronously')
        future = self.client_GetEciProductID.call_async(request)
        self.get_logger().info(f'Called service GetEciProductID asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciProductID is complete')
        response: covvi_interfaces.srv.GetEciProductID.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetEciProductIDClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()