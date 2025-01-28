import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetCurrentGripClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetCurrentGrip = f'{self.get_namespace()}/{service}/SetCurrentGrip'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetCurrentGrip}')
        self.client_SetCurrentGrip = self.create_client(
            covvi_interfaces.srv.SetCurrentGrip,
            full_service_name_SetCurrentGrip,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetCurrentGrip}')
        while not self.client_SetCurrentGrip.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetCurrentGrip} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetCurrentGrip}')
        
    @public
    def setCurrentGrip(self,
        grip_id: covvi_interfaces.msg.CurrentGripID,
    ) -> covvi_interfaces.msg.CurrentGripGripIdMsg:
        """Set the current grip via the Grip ID"""
        request = covvi_interfaces.srv.SetCurrentGrip.Request()
        request.grip_id = grip_id
        self.get_logger().info(f'Calling service SetCurrentGrip asynchronously')
        future = self.client_SetCurrentGrip.call_async(request)
        self.get_logger().info(f'Called service SetCurrentGrip asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetCurrentGrip is complete')
        response: covvi_interfaces.srv.SetCurrentGrip.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = SetCurrentGripClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()