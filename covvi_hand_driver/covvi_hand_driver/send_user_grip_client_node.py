import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SendUserGripClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SendUserGrip = f'{self.get_namespace()}/{service}/SendUserGrip'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SendUserGrip}')
        self.client_SendUserGrip = self.create_client(
            covvi_interfaces.srv.SendUserGrip,
            full_service_name_SendUserGrip,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SendUserGrip}')
        while not self.client_SendUserGrip.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SendUserGrip} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SendUserGrip}')
        
    @public
    def sendUserGrip(self,
        grip_name_index: covvi_interfaces.msg.GripNameIndex,
        user_grip:       covvi_interfaces.msg.UserGripID,
    ) -> covvi_interfaces.msg.UserGripResMsg:
        """"""
        request = covvi_interfaces.srv.SendUserGrip.Request()
        request.grip_name_index = grip_name_index
        request.user_grip       = user_grip
        self.get_logger().info(f'Calling service SendUserGrip asynchronously')
        future = self.client_SendUserGrip.call_async(request)
        self.get_logger().info(f'Called service SendUserGrip asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SendUserGrip is complete')
        response: covvi_interfaces.srv.SendUserGrip.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SendUserGripClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()