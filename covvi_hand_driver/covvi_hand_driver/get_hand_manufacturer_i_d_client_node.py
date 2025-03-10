import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetHandManufacturerIDClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetHandManufacturerID = f'{self.get_namespace()}/{service}/GetHandManufacturerID'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetHandManufacturerID}')
        self.client_GetHandManufacturerID = self.create_client(
            covvi_interfaces.srv.GetHandManufacturerID,
            full_service_name_GetHandManufacturerID,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetHandManufacturerID}')
        while not self.client_GetHandManufacturerID.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetHandManufacturerID} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetHandManufacturerID}')
        
    @public
    def getHandManufacturerID(self) -> int:
        """Get the manufacturer ID of the Hand"""
        request = covvi_interfaces.srv.GetHandManufacturerID.Request()
        self.get_logger().info(f'Calling service GetHandManufacturerID asynchronously')
        future = self.client_GetHandManufacturerID.call_async(request)
        self.get_logger().info(f'Called service GetHandManufacturerID asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetHandManufacturerID is complete')
        response: covvi_interfaces.srv.GetHandManufacturerID.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetHandManufacturerIDClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()