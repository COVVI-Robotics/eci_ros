import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetSystemStatusClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetSystemStatus = f'{self.get_namespace()}/{service}/GetSystemStatus'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetSystemStatus}')
        self.client_GetSystemStatus = self.create_client(
            covvi_interfaces.srv.GetSystemStatus,
            full_service_name_GetSystemStatus,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetSystemStatus}')
        while not self.client_GetSystemStatus.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetSystemStatus} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetSystemStatus}')
        
    @public
    def getSystemStatus(self) -> covvi_interfaces.msg.SystemStatusMsg:
        """Read system status

        Critical error flags
        Non-fatal errors
        Bluetooth Status
        Change Notifications
        """
        request = covvi_interfaces.srv.GetSystemStatus.Request()
        self.get_logger().info(f'Calling service GetSystemStatus asynchronously')
        future = self.client_GetSystemStatus.call_async(request)
        self.get_logger().info(f'Called service GetSystemStatus asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetSystemStatus is complete')
        response: covvi_interfaces.srv.GetSystemStatus.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetSystemStatusClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()