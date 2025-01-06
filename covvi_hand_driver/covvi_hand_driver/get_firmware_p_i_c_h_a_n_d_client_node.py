import sys
from time import sleep
from typing import Iterable, Any

import rclpy

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetFirmwarePICHANDClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', node_name: str = 'covvi_get_firmware_p_i_c_h_a_n_d_client_node', **kwargs):
        super().__init__(service=service, node_name=node_name, **kwargs)
        full_service_name_GetFirmwarePICHAND = self._get_full_ros2_name('GetFirmwarePICHAND', service=service)
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetFirmwarePICHAND}')
        self.client_GetFirmwarePICHAND = self.create_client(
            covvi_interfaces.srv.GetFirmwarePICHAND,
            full_service_name_GetFirmwarePICHAND,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetFirmwarePICHAND}')
        while not self.client_GetFirmwarePICHAND.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetFirmwarePICHAND} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetFirmwarePICHAND}')
        
    @public
    def getFirmware_PIC_HAND(self) -> covvi_interfaces.msg.HandFirmwarePicMsg:
        """"""
        request = covvi_interfaces.srv.GetFirmwarePICHAND.Request()
        self.get_logger().info(f'Calling service GetFirmwarePICHAND asynchronously')
        future = self.client_GetFirmwarePICHAND.call_async(request)
        self.get_logger().info(f'Called service GetFirmwarePICHAND asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetFirmwarePICHAND is complete')
        response: covvi_interfaces.srv.GetFirmwarePICHAND.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    rclpy.init(args=args)
    covvi_get_firmware_p_i_c_h_a_n_d_client_node = GetFirmwarePICHANDClientNode(service=service)
    rclpy.spin(covvi_get_firmware_p_i_c_h_a_n_d_client_node)
    covvi_get_firmware_p_i_c_h_a_n_d_client_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()