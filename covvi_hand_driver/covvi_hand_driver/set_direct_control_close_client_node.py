import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDirectControlCloseClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetDirectControlClose = f'{self.get_namespace()}/{service}/SetDirectControlClose'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDirectControlClose}')
        self.client_SetDirectControlClose = self.create_client(
            covvi_interfaces.srv.SetDirectControlClose,
            full_service_name_SetDirectControlClose,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDirectControlClose}')
        while not self.client_SetDirectControlClose.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDirectControlClose} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDirectControlClose}')
        
    @public
    def setDirectControlClose(self,
        speed: covvi_interfaces.msg.Speed = 50,
    ) -> covvi_interfaces.msg.DirectControlMsg:
        """"""
        request = covvi_interfaces.srv.SetDirectControlClose.Request()
        request.speed = speed
        self.get_logger().info(f'Calling service SetDirectControlClose asynchronously')
        future = self.client_SetDirectControlClose.call_async(request)
        self.get_logger().info(f'Called service SetDirectControlClose asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDirectControlClose is complete')
        response: covvi_interfaces.srv.SetDirectControlClose.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SetDirectControlCloseClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()