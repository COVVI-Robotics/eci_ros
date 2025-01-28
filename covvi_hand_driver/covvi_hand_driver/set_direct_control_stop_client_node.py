import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDirectControlStopClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetDirectControlStop = f'{self.get_namespace()}/{service}/SetDirectControlStop'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDirectControlStop}')
        self.client_SetDirectControlStop = self.create_client(
            covvi_interfaces.srv.SetDirectControlStop,
            full_service_name_SetDirectControlStop,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDirectControlStop}')
        while not self.client_SetDirectControlStop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDirectControlStop} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDirectControlStop}')
        
    @public
    def setDirectControlStop(self) -> covvi_interfaces.msg.DirectControlMsg:
        """"""
        request = covvi_interfaces.srv.SetDirectControlStop.Request()
        self.get_logger().info(f'Calling service SetDirectControlStop asynchronously')
        future = self.client_SetDirectControlStop.call_async(request)
        self.get_logger().info(f'Called service SetDirectControlStop asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDirectControlStop is complete')
        response: covvi_interfaces.srv.SetDirectControlStop.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = SetDirectControlStopClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()