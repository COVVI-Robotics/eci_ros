import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetMotorCurrentAllClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_motor_current_all_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetMotorCurrentAll = self._get_full_ros2_name('GetMotorCurrentAll', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetMotorCurrentAll}')
        self.client_GetMotorCurrentAll = self.create_client(
            covvi_interfaces.srv.GetMotorCurrentAll,
            full_service_name_GetMotorCurrentAll,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetMotorCurrentAll}')
        while not self.client_GetMotorCurrentAll.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetMotorCurrentAll} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetMotorCurrentAll}')
        
    @public
    def getMotorCurrent_all(self) -> covvi_interfaces.msg.MotorCurrentAllMsg:
        """Get all motor currents

        Motor current is not available for rotation motor,
        The current value is in multiples of 16mA. e.g. 1 = 16mA, 64 = 1024mA
        """
        request = covvi_interfaces.srv.GetMotorCurrentAll.Request()
        self.get_logger().info(f'Calling service GetMotorCurrentAll asynchronously')
        future = self.client_GetMotorCurrentAll.call_async(request)
        self.get_logger().info(f'Called service GetMotorCurrentAll asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetMotorCurrentAll is complete')
        response: covvi_interfaces.srv.GetMotorCurrentAll.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_motor_current_all_client_node = GetMotorCurrentAllClientNode(service=service)
    rclpy.spin(covvi_get_motor_current_all_client_node)
    covvi_get_motor_current_all_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()