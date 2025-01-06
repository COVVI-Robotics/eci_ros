import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class RemoveUserGripClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_remove_user_grip_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_RemoveUserGrip = self._get_full_ros2_name('RemoveUserGrip', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_RemoveUserGrip}')
        self.client_RemoveUserGrip = self.create_client(
            covvi_interfaces.srv.RemoveUserGrip,
            full_service_name_RemoveUserGrip,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_RemoveUserGrip}')
        while not self.client_RemoveUserGrip.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_RemoveUserGrip} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_RemoveUserGrip}')
        
    @public
    def removeUserGrip(self,
        grip_name_index: covvi_interfaces.msg.GripNameIndex,
    ) -> covvi_interfaces.msg.UserGripResMsg:
        """"""
        request = covvi_interfaces.srv.RemoveUserGrip.Request()
        request.grip_name_index = grip_name_index
        self.get_logger().info(f'Calling service RemoveUserGrip asynchronously')
        future = self.client_RemoveUserGrip.call_async(request)
        self.get_logger().info(f'Called service RemoveUserGrip asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service RemoveUserGrip is complete')
        response: covvi_interfaces.srv.RemoveUserGrip.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_remove_user_grip_client_node = RemoveUserGripClientNode(service=service)
    rclpy.spin(covvi_remove_user_grip_client_node)
    covvi_remove_user_grip_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()