import sys
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

from covvi_hand_driver.disable_all_realtime_cfg_client_node import DisableAllRealtimeCfgClientNode
from covvi_hand_driver.enable_all_realtime_cfg_client_node import EnableAllRealtimeCfgClientNode
from covvi_hand_driver.get_current_grip_client_node import GetCurrentGripClientNode
from covvi_hand_driver.get_device_identity_client_node import GetDeviceIdentityClientNode
from covvi_hand_driver.get_device_product_client_node import GetDeviceProductClientNode
from covvi_hand_driver.get_digit_config_client_node import GetDigitConfigClientNode
from covvi_hand_driver.get_digit_error_client_node import GetDigitErrorClientNode
from covvi_hand_driver.get_digit_posn_client_node import GetDigitPosnClientNode
from covvi_hand_driver.get_digit_posn_all_client_node import GetDigitPosnAllClientNode
from covvi_hand_driver.get_digit_status_client_node import GetDigitStatusClientNode
from covvi_hand_driver.get_digit_status_all_client_node import GetDigitStatusAllClientNode
from covvi_hand_driver.get_eci_connected_client_node import GetEciConnectedClientNode
from covvi_hand_driver.get_eci_device_class_type_client_node import GetEciDeviceClassTypeClientNode
from covvi_hand_driver.get_eci_error_client_node import GetEciErrorClientNode
from covvi_hand_driver.get_eci_manufacturer_i_d_client_node import GetEciManufacturerIDClientNode
from covvi_hand_driver.get_eci_power_on_client_node import GetEciPowerOnClientNode
from covvi_hand_driver.get_eci_product_i_d_client_node import GetEciProductIDClientNode
from covvi_hand_driver.get_eci_serial_number_client_node import GetEciSerialNumberClientNode
from covvi_hand_driver.get_environmental_client_node import GetEnvironmentalClientNode
from covvi_hand_driver.get_firmware_p_i_c_e_c_i_client_node import GetFirmwarePICECIClientNode
from covvi_hand_driver.get_firmware_p_i_c_h_a_n_d_client_node import GetFirmwarePICHANDClientNode
from covvi_hand_driver.get_grip_name_client_node import GetGripNameClientNode
from covvi_hand_driver.get_hand_connected_client_node import GetHandConnectedClientNode
from covvi_hand_driver.get_hand_device_class_type_client_node import GetHandDeviceClassTypeClientNode
from covvi_hand_driver.get_hand_error_client_node import GetHandErrorClientNode
from covvi_hand_driver.get_hand_manufacturer_i_d_client_node import GetHandManufacturerIDClientNode
from covvi_hand_driver.get_hand_power_on_client_node import GetHandPowerOnClientNode
from covvi_hand_driver.get_hand_product_i_d_client_node import GetHandProductIDClientNode
from covvi_hand_driver.get_hand_serial_number_client_node import GetHandSerialNumberClientNode
from covvi_hand_driver.get_hello_client_node import GetHelloClientNode
from covvi_hand_driver.get_motor_current_client_node import GetMotorCurrentClientNode
from covvi_hand_driver.get_motor_current_all_client_node import GetMotorCurrentAllClientNode
from covvi_hand_driver.get_motor_limits_client_node import GetMotorLimitsClientNode
from covvi_hand_driver.get_orientation_client_node import GetOrientationClientNode
from covvi_hand_driver.get_pinch_config_client_node import GetPinchConfigClientNode
from covvi_hand_driver.get_system_status_client_node import GetSystemStatusClientNode
from covvi_hand_driver.remove_user_grip_client_node import RemoveUserGripClientNode
from covvi_hand_driver.reset_realtime_cfg_client_node import ResetRealtimeCfgClientNode
from covvi_hand_driver.reset_user_grips_client_node import ResetUserGripsClientNode
from covvi_hand_driver.send_user_grip_client_node import SendUserGripClientNode
from covvi_hand_driver.set_current_grip_client_node import SetCurrentGripClientNode
from covvi_hand_driver.set_digit_move_client_node import SetDigitMoveClientNode
from covvi_hand_driver.set_digit_posn_client_node import SetDigitPosnClientNode
from covvi_hand_driver.set_digit_posn_stop_client_node import SetDigitPosnStopClientNode
from covvi_hand_driver.set_direct_control_close_client_node import SetDirectControlCloseClientNode
from covvi_hand_driver.set_direct_control_open_client_node import SetDirectControlOpenClientNode
from covvi_hand_driver.set_direct_control_stop_client_node import SetDirectControlStopClientNode
from covvi_hand_driver.set_hand_power_off_client_node import SetHandPowerOffClientNode
from covvi_hand_driver.set_hand_power_on_client_node import SetHandPowerOnClientNode
from covvi_hand_driver.set_realtime_cfg_client_node import SetRealtimeCfgClientNode

from covvi_hand_driver.digit_status_all_msg_subscriber_node import DigitStatusAllMsgSubscriberNode
from covvi_hand_driver.digit_posn_all_msg_subscriber_node import DigitPosnAllMsgSubscriberNode
from covvi_hand_driver.current_grip_msg_subscriber_node import CurrentGripMsgSubscriberNode
from covvi_hand_driver.electrode_value_msg_subscriber_node import ElectrodeValueMsgSubscriberNode
from covvi_hand_driver.input_status_msg_subscriber_node import InputStatusMsgSubscriberNode
from covvi_hand_driver.motor_current_all_msg_subscriber_node import MotorCurrentAllMsgSubscriberNode
from covvi_hand_driver.digit_touch_all_msg_subscriber_node import DigitTouchAllMsgSubscriberNode
from covvi_hand_driver.environmental_msg_subscriber_node import EnvironmentalMsgSubscriberNode
from covvi_hand_driver.system_status_msg_subscriber_node import SystemStatusMsgSubscriberNode
from covvi_hand_driver.orientation_msg_subscriber_node import OrientationMsgSubscriberNode
from covvi_hand_driver.motor_limits_msg_subscriber_node import MotorLimitsMsgSubscriberNode


class CovviClientNode(
    DisableAllRealtimeCfgClientNode,
    EnableAllRealtimeCfgClientNode,
    GetCurrentGripClientNode,
    GetDeviceIdentityClientNode,
    GetDeviceProductClientNode,
    GetDigitConfigClientNode,
    GetDigitErrorClientNode,
    GetDigitPosnClientNode,
    GetDigitPosnAllClientNode,
    GetDigitStatusClientNode,
    GetDigitStatusAllClientNode,
    GetEciConnectedClientNode,
    GetEciDeviceClassTypeClientNode,
    GetEciErrorClientNode,
    GetEciManufacturerIDClientNode,
    GetEciPowerOnClientNode,
    GetEciProductIDClientNode,
    GetEciSerialNumberClientNode,
    GetEnvironmentalClientNode,
    GetFirmwarePICECIClientNode,
    GetFirmwarePICHANDClientNode,
    GetGripNameClientNode,
    GetHandConnectedClientNode,
    GetHandDeviceClassTypeClientNode,
    GetHandErrorClientNode,
    GetHandManufacturerIDClientNode,
    GetHandPowerOnClientNode,
    GetHandProductIDClientNode,
    GetHandSerialNumberClientNode,
    GetHelloClientNode,
    GetMotorCurrentClientNode,
    GetMotorCurrentAllClientNode,
    GetMotorLimitsClientNode,
    GetOrientationClientNode,
    GetPinchConfigClientNode,
    GetSystemStatusClientNode,
    RemoveUserGripClientNode,
    ResetRealtimeCfgClientNode,
    ResetUserGripsClientNode,
    SendUserGripClientNode,
    SetCurrentGripClientNode,
    SetDigitMoveClientNode,
    SetDigitPosnClientNode,
    SetDigitPosnStopClientNode,
    SetDirectControlCloseClientNode,
    SetDirectControlOpenClientNode,
    SetDirectControlStopClientNode,
    SetHandPowerOffClientNode,
    SetHandPowerOnClientNode,
    SetRealtimeCfgClientNode,

    DigitStatusAllMsgSubscriberNode,
    DigitPosnAllMsgSubscriberNode,
    CurrentGripMsgSubscriberNode,
    ElectrodeValueMsgSubscriberNode,
    InputStatusMsgSubscriberNode,
    MotorCurrentAllMsgSubscriberNode,
    DigitTouchAllMsgSubscriberNode,
    EnvironmentalMsgSubscriberNode,
    SystemStatusMsgSubscriberNode,
    OrientationMsgSubscriberNode,
    MotorLimitsMsgSubscriberNode,
):
    def __init__(self, service: str = '', **kwargs):
        super().__init__(service=service, **kwargs)


def main(args: Iterable[Any] | None = None) -> None:
    _, service, *_ = sys.argv
    try:
        with rclpy.init(args=args):
            rclpy.spin(CovviClientNode(service=service))
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()