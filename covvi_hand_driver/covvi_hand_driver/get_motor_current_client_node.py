import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetMotorCurrentClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_motor_current_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetMotorCurrent = self._get_full_ros2_name('GetMotorCurrent', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetMotorCurrent}')
        self.client_GetMotorCurrent = self.create_client(
            covvi_interfaces.srv.GetMotorCurrent,
            full_service_name_GetMotorCurrent,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetMotorCurrent}')
        while not self.client_GetMotorCurrent.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetMotorCurrent} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetMotorCurrent}')
        
    @public
    def getMotorCurrent(self,
        digit: covvi_interfaces.msg.Digit5,
    ) -> covvi_interfaces.msg.MotorCurrentMsg:
        """Get motor current

        Motor current is not available for rotation motor,
        The current value is in multiples of 16mA. e.g. 1 = 16mA, 64 = 1024mA
        """
        request = covvi_interfaces.srv.GetMotorCurrent.Request()
        request.digit = digit
        self.get_logger().info(f'Calling service GetMotorCurrent asynchronously')
        future = self.client_GetMotorCurrent.call_async(request)
        self.get_logger().info(f'Called service GetMotorCurrent asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetMotorCurrent is complete')
        response: covvi_interfaces.srv.GetMotorCurrent.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_motor_current_client_node = GetMotorCurrentClientNode(service=service)
    rclpy.spin(covvi_get_motor_current_client_node)
    covvi_get_motor_current_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()