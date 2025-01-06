import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetGripNameClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_grip_name_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetGripName = self._get_full_ros2_name('GetGripName', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetGripName}')
        self.client_GetGripName = self.create_client(
            covvi_interfaces.srv.GetGripName,
            full_service_name_GetGripName,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetGripName}')
        while not self.client_GetGripName.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetGripName} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetGripName}')
        
    @public
    def getGripName(self,
        grip_name_index: covvi_interfaces.msg.GripNameIndex,
    ) -> covvi_interfaces.msg.GripName:
        """Get user grip name"""
        request = covvi_interfaces.srv.GetGripName.Request()
        request.grip_name_index = grip_name_index
        self.get_logger().info(f'Calling service GetGripName asynchronously')
        future = self.client_GetGripName.call_async(request)
        self.get_logger().info(f'Called service GetGripName asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetGripName is complete')
        response: covvi_interfaces.srv.GetGripName.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_grip_name_client_node = GetGripNameClientNode(service=service)
    rclpy.spin(covvi_get_grip_name_client_node)
    covvi_get_grip_name_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()