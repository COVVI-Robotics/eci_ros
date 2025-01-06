import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDigitPosnClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_set_digit_posn_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_SetDigitPosn = self._get_full_ros2_name('SetDigitPosn', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDigitPosn}')
        self.client_SetDigitPosn = self.create_client(
            covvi_interfaces.srv.SetDigitPosn,
            full_service_name_SetDigitPosn,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDigitPosn}')
        while not self.client_SetDigitPosn.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDigitPosn} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDigitPosn}')
        
    @public
    def setDigitPosn(self,
        speed:  covvi_interfaces.msg.Percentage,
        thumb:  int = -1,
        index:  int = -1,
        middle: int = -1,
        ring:   int = -1,
        little: int = -1,
        rotate: int = -1,
    ) -> covvi_interfaces.msg.DigitPosnSetMsg:
        """Set the digit position to move to and the movement speed for each digit and thumb rotation"""
        request = covvi_interfaces.srv.SetDigitPosn.Request()
        request.speed  = speed
        request.thumb  = thumb
        request.index  = index
        request.middle = middle
        request.ring   = ring
        request.little = little
        request.rotate = rotate
        self.get_logger().info(f'Calling service SetDigitPosn asynchronously')
        future = self.client_SetDigitPosn.call_async(request)
        self.get_logger().info(f'Called service SetDigitPosn asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDigitPosn is complete')
        response: covvi_interfaces.srv.SetDigitPosn.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_set_digit_posn_client_node = SetDigitPosnClientNode(service=service)
    rclpy.spin(covvi_set_digit_posn_client_node)
    covvi_set_digit_posn_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()