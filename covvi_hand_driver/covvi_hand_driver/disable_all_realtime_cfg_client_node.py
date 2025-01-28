import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class DisableAllRealtimeCfgClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_DisableAllRealtimeCfg = f'{self.get_namespace()}/{service}/DisableAllRealtimeCfg'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_DisableAllRealtimeCfg}')
        self.client_DisableAllRealtimeCfg = self.create_client(
            covvi_interfaces.srv.DisableAllRealtimeCfg,
            full_service_name_DisableAllRealtimeCfg,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_DisableAllRealtimeCfg}')
        while not self.client_DisableAllRealtimeCfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_DisableAllRealtimeCfg} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_DisableAllRealtimeCfg}')
        
    @public
    def disableAllRealtimeCfg(self) -> covvi_interfaces.msg.RealtimeCfg:
        """"""
        request = covvi_interfaces.srv.DisableAllRealtimeCfg.Request()
        self.get_logger().info(f'Calling service DisableAllRealtimeCfg asynchronously')
        future = self.client_DisableAllRealtimeCfg.call_async(request)
        self.get_logger().info(f'Called service DisableAllRealtimeCfg asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service DisableAllRealtimeCfg is complete')
        response: covvi_interfaces.srv.DisableAllRealtimeCfg.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = DisableAllRealtimeCfgClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()