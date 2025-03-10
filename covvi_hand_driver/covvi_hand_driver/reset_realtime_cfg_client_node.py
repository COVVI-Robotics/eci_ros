import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class ResetRealtimeCfgClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_ResetRealtimeCfg = f'{self.get_namespace()}/{service}/ResetRealtimeCfg'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_ResetRealtimeCfg}')
        self.client_ResetRealtimeCfg = self.create_client(
            covvi_interfaces.srv.ResetRealtimeCfg,
            full_service_name_ResetRealtimeCfg,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_ResetRealtimeCfg}')
        while not self.client_ResetRealtimeCfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_ResetRealtimeCfg} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_ResetRealtimeCfg}')
        
    @public
    def resetRealtimeCfg(self) -> None:
        """"""
        request = covvi_interfaces.srv.ResetRealtimeCfg.Request()
        self.get_logger().info(f'Calling service ResetRealtimeCfg asynchronously')
        future = self.client_ResetRealtimeCfg.call_async(request)
        self.get_logger().info(f'Called service ResetRealtimeCfg asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service ResetRealtimeCfg is complete')
        response: covvi_interfaces.srv.ResetRealtimeCfg.Response = future.result()


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = ResetRealtimeCfgClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()