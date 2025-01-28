import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetMotorLimitsClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetMotorLimits = f'{self.get_namespace()}/{service}/GetMotorLimits'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetMotorLimits}')
        self.client_GetMotorLimits = self.create_client(
            covvi_interfaces.srv.GetMotorLimits,
            full_service_name_GetMotorLimits,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetMotorLimits}')
        while not self.client_GetMotorLimits.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetMotorLimits} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetMotorLimits}')
        
    @public
    def getMotorLimits(self) -> covvi_interfaces.msg.MotorLimitsMsg:
        """Get motor limits"""
        request = covvi_interfaces.srv.GetMotorLimits.Request()
        self.get_logger().info(f'Calling service GetMotorLimits asynchronously')
        future = self.client_GetMotorLimits.call_async(request)
        self.get_logger().info(f'Called service GetMotorLimits asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetMotorLimits is complete')
        response: covvi_interfaces.srv.GetMotorLimits.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetMotorLimitsClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()