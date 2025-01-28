import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetOrientationClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetOrientation = f'{self.get_namespace()}/{service}/GetOrientation'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetOrientation}')
        self.client_GetOrientation = self.create_client(
            covvi_interfaces.srv.GetOrientation,
            full_service_name_GetOrientation,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetOrientation}')
        while not self.client_GetOrientation.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetOrientation} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetOrientation}')
        
    @public
    def getOrientation(self) -> covvi_interfaces.msg.OrientationMsg:
        """Get hand orientation

        X Position
        Y Position
        Z Position
        """
        request = covvi_interfaces.srv.GetOrientation.Request()
        self.get_logger().info(f'Calling service GetOrientation asynchronously')
        future = self.client_GetOrientation.call_async(request)
        self.get_logger().info(f'Called service GetOrientation asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetOrientation is complete')
        response: covvi_interfaces.srv.GetOrientation.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    node = GetOrientationClientNode(service=service)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()