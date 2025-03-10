import sys
from time import sleep
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg
from covvi_hand_driver.covvi_base_client_node import CovviBaseClientNode, public


class GetFirmwarePICECIClientNode(CovviBaseClientNode):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)
        full_service_name_GetFirmwarePICECI = f'{self.get_namespace()}/{service}/GetFirmwarePICECI'
        self.get_logger().info(f'Creating ROS2 Client:   {full_service_name_GetFirmwarePICECI}')
        self.client_GetFirmwarePICECI = self.create_client(
            covvi_interfaces.srv.GetFirmwarePICECI,
            full_service_name_GetFirmwarePICECI,
        )
        self.get_logger().info(f'Connecting ROS2 Client: {full_service_name_GetFirmwarePICECI}')
        while not self.client_GetFirmwarePICECI.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f'{full_service_name_GetFirmwarePICECI} service not available, waiting again...')
        self.get_logger().info(f'Connected ROS2 Client:  {full_service_name_GetFirmwarePICECI}')
        
    @public
    def getFirmware_PIC_ECI(self) -> covvi_interfaces.msg.EciFirmwarePicMsg:
        """"""
        request = covvi_interfaces.srv.GetFirmwarePICECI.Request()
        self.get_logger().info(f'Calling service GetFirmwarePICECI asynchronously')
        future = self.client_GetFirmwarePICECI.call_async(request)
        self.get_logger().info(f'Called service GetFirmwarePICECI asynchronously, waiting for response...')
        while not future.done():
            sleep(2**-6)
        self.get_logger().info(f'Service GetFirmwarePICECI is complete')
        response: covvi_interfaces.srv.GetFirmwarePICECI.Response = future.result()
        return response.result


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        rclpy.init(args=args)
        node = GetFirmwarePICECIClientNode(service=service)
        rclpy.spin(node)
        node.destroy_node()
        rclpy.shutdown()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()