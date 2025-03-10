import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class SetRealtimeCfgClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_SetRealtimeCfg = f'{self.get_namespace()}/{service}/SetRealtimeCfg'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_SetRealtimeCfg}')
        self.client_SetRealtimeCfg = self.create_client(
            covvi_interfaces.srv.SetRealtimeCfg,
            full_service_name_SetRealtimeCfg,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_SetRealtimeCfg}')
        while not self.client_SetRealtimeCfg.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_SetRealtimeCfg} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_SetRealtimeCfg}')
        
    @public
    def setRealtimeCfg(self,
        digit_status:    bool = False,
        digit_posn:      bool = False,
        current_grip:    bool = False,
        electrode_value: bool = False,
        input_status:    bool = False,
        motor_current:   bool = False,
        digit_touch:     bool = False,
        digit_error:     bool = False,
        environmental:   bool = False,
        orientation:     bool = False,
        motor_limits:    bool = False,
    ) -> covvi_interfaces.msg.RealtimeCfg:
        """"""
        request = covvi_interfaces.srv.SetRealtimeCfg.Request()
        request.digit_status    = digit_status
        request.digit_posn      = digit_posn
        request.current_grip    = current_grip
        request.electrode_value = electrode_value
        request.input_status    = input_status
        request.motor_current   = motor_current
        request.digit_touch     = digit_touch
        request.digit_error     = digit_error
        request.environmental   = environmental
        request.orientation     = orientation
        request.motor_limits    = motor_limits
        self.get_logger().info(f'Calling service SetRealtimeCfg asynchronously')
        future = self.client_SetRealtimeCfg.call_async(request)
        self.get_logger().info(f'Called service SetRealtimeCfg asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service SetRealtimeCfg is complete')
        response: covvi_interfaces.srv.SetRealtimeCfg.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = SetRealtimeCfgClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()