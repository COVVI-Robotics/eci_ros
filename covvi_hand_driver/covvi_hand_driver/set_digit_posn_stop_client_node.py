import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetDigitPosnStopClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetDigitPosnStop = f'{self.get_namespace()}/{service}/SetDigitPosnStop'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetDigitPosnStop}')
        self.client_SetDigitPosnStop = self.create_client(
            covvi_interfaces.srv.SetDigitPosnStop,
            full_service_name_SetDigitPosnStop,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetDigitPosnStop}')
        while not self.client_SetDigitPosnStop.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetDigitPosnStop} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetDigitPosnStop}')
        
    @public
    def setDigitPosnStop(self) -> covvi_interfaces.msg.DigitPosnSetMsg:
        """Set the digit movement to stop"""
        request = covvi_interfaces.srv.SetDigitPosnStop.Request()
        self.get_logger().info(f'Calling service SetDigitPosnStop asynchronously')
        future = self.client_SetDigitPosnStop.call_async(request)
        self.get_logger().info(f'Called service SetDigitPosnStop asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetDigitPosnStop is complete')
        response: covvi_interfaces.srv.SetDigitPosnStop.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SetDigitPosnStopClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()