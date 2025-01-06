import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetDigitPosnAllClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_digit_posn_all_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetDigitPosnAll = self._get_full_ros2_name('GetDigitPosnAll', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetDigitPosnAll}')
        self.client_GetDigitPosnAll = self.create_client(
            covvi_interfaces.srv.GetDigitPosnAll,
            full_service_name_GetDigitPosnAll,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetDigitPosnAll}')
        while not self.client_GetDigitPosnAll.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetDigitPosnAll} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetDigitPosnAll}')
        
    @public
    def getDigitPosn_all(self) -> covvi_interfaces.msg.DigitPosnAllMsg:
        """Get all digit positions"""
        request = covvi_interfaces.srv.GetDigitPosnAll.Request()
        self.get_logger().info(f'Calling service GetDigitPosnAll asynchronously')
        future = self.client_GetDigitPosnAll.call_async(request)
        self.get_logger().info(f'Called service GetDigitPosnAll asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetDigitPosnAll is complete')
        response: covvi_interfaces.srv.GetDigitPosnAll.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_digit_posn_all_client_node = GetDigitPosnAllClientNode(service=service)
    rclpy.spin(covvi_get_digit_posn_all_client_node)
    covvi_get_digit_posn_all_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()