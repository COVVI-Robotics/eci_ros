import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetEciManufacturerIDClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetEciManufacturerID = f'{self.get_namespace()}/{service}/GetEciManufacturerID'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetEciManufacturerID}')
        self.client_GetEciManufacturerID = self.create_client(
            covvi_interfaces.srv.GetEciManufacturerID,
            full_service_name_GetEciManufacturerID,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetEciManufacturerID}')
        while not self.client_GetEciManufacturerID.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetEciManufacturerID} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetEciManufacturerID}')
        
    @public
    def getEciManufacturerID(self) -> int:
        """Get the manufacturer ID of the ECI"""
        request = covvi_interfaces.srv.GetEciManufacturerID.Request()
        self.get_logger().info(f'Calling service GetEciManufacturerID asynchronously')
        future = self.client_GetEciManufacturerID.call_async(request)
        self.get_logger().info(f'Called service GetEciManufacturerID asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetEciManufacturerID is complete')
        response: covvi_interfaces.srv.GetEciManufacturerID.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetEciManufacturerIDClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()