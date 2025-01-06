import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class ResetUserGripsClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_reset_user_grips_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_ResetUserGrips = self._get_full_ros2_name('ResetUserGrips', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_ResetUserGrips}')
        self.client_ResetUserGrips = self.create_client(
            covvi_interfaces.srv.ResetUserGrips,
            full_service_name_ResetUserGrips,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_ResetUserGrips}')
        while not self.client_ResetUserGrips.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_ResetUserGrips} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_ResetUserGrips}')
        
    @public
    def resetUserGrips(self) -> None:
        """"""
        request = covvi_interfaces.srv.ResetUserGrips.Request()
        self.get_logger().info(f'Calling service ResetUserGrips asynchronously')
        future = self.client_ResetUserGrips.call_async(request)
        self.get_logger().info(f'Called service ResetUserGrips asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service ResetUserGrips is complete')
        response: covvi_interfaces.srv.ResetUserGrips.Response = future.result()


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_reset_user_grips_client_node = ResetUserGripsClientNode(service=service)
    rclpy.spin(covvi_reset_user_grips_client_node)
    covvi_reset_user_grips_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()