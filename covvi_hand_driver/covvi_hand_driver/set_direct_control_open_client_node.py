import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDirectControlOpenClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_set_direct_control_open_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_SetDirectControlOpen = self._get_full_ros2_name('SetDirectControlOpen', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDirectControlOpen}')
        self.client_SetDirectControlOpen = self.create_client(
            covvi_interfaces.srv.SetDirectControlOpen,
            full_service_name_SetDirectControlOpen,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDirectControlOpen}')
        while not self.client_SetDirectControlOpen.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDirectControlOpen} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDirectControlOpen}')
        
    @public
    def setDirectControlOpen(self,
        speed: covvi_interfaces.msg.Percentage = covvi_interfaces.msg.Percentage(value=50),
    ) -> covvi_interfaces.msg.DirectControlMsg:
        """"""
        request = covvi_interfaces.srv.SetDirectControlOpen.Request()
        request.speed = speed
        self.get_logger().info(f'Calling service SetDirectControlOpen asynchronously')
        future = self.client_SetDirectControlOpen.call_async(request)
        self.get_logger().info(f'Called service SetDirectControlOpen asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDirectControlOpen is complete')
        response: covvi_interfaces.srv.SetDirectControlOpen.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_set_direct_control_open_client_node = SetDirectControlOpenClientNode(service=service)
    rclpy.spin(covvi_set_direct_control_open_client_node)
    covvi_set_direct_control_open_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()