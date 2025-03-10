import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class EnableAllRealtimeCfgClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_EnableAllRealtimeCfg = f'{self.get_namespace()}/{service}/EnableAllRealtimeCfg'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_EnableAllRealtimeCfg}')
        self.client_EnableAllRealtimeCfg = self.create_client(
            covvi_interfaces.srv.EnableAllRealtimeCfg,
            full_service_name_EnableAllRealtimeCfg,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_EnableAllRealtimeCfg}')
        while not self.client_EnableAllRealtimeCfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_EnableAllRealtimeCfg} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_EnableAllRealtimeCfg}')
        
    @public
    def enableAllRealtimeCfg(self) -> covvi_interfaces.msg.RealtimeCfg:
        """"""
        request = covvi_interfaces.srv.EnableAllRealtimeCfg.Request()
        self.get_logger().info(f'Calling service EnableAllRealtimeCfg asynchronously')
        future = self.client_EnableAllRealtimeCfg.call_async(request)
        self.get_logger().info(f'Called service EnableAllRealtimeCfg asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service EnableAllRealtimeCfg is complete')
        response: covvi_interfaces.srv.EnableAllRealtimeCfg.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = EnableAllRealtimeCfgClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()