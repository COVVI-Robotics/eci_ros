import sys
from typing import Iterable, Any

import rclpy
from rclpy.executors import ExternalShutdownException

import covvi_interfaces.srv
import covvi_interfaces.msg

from covvi_hand_driver.covvi_base_node import CovviBaseNode

import eci
from eci import CovviInterface, FourOctetAddress


class CovviServerNode(CovviBaseNode):
    def __init__(self, host: FourOctetAddress | str = '', node_name: str = 'covvi_server_node', **kwargs):
        assert host
        super().__init__(node_name=node_name, **kwargs)
        self.host = FourOctetAddress(host)
        self.get_logger().info(f'Host: {host}')
        self.eci = CovviInterface(host)

        self.get_logger().info(f'Creating ROS2 Services')

        full_service_name_DisableAllRealtimeCfg = self._get_full_ros2_name('DisableAllRealtimeCfg')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_DisableAllRealtimeCfg}')
        self.service_DisableAllRealtimeCfg = self.create_service(covvi_interfaces.srv.DisableAllRealtimeCfg, full_service_name_DisableAllRealtimeCfg, self.serviceCallbackDisableAllRealtimeCfg)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_DisableAllRealtimeCfg}')

        full_service_name_EnableAllRealtimeCfg = self._get_full_ros2_name('EnableAllRealtimeCfg')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_EnableAllRealtimeCfg}')
        self.service_EnableAllRealtimeCfg = self.create_service(covvi_interfaces.srv.EnableAllRealtimeCfg, full_service_name_EnableAllRealtimeCfg, self.serviceCallbackEnableAllRealtimeCfg)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_EnableAllRealtimeCfg}')

        full_service_name_GetCurrentGrip = self._get_full_ros2_name('GetCurrentGrip')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetCurrentGrip}')
        self.service_GetCurrentGrip = self.create_service(covvi_interfaces.srv.GetCurrentGrip, full_service_name_GetCurrentGrip, self.serviceCallbackGetCurrentGrip)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetCurrentGrip}')

        full_service_name_GetDeviceIdentity = self._get_full_ros2_name('GetDeviceIdentity')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDeviceIdentity}')
        self.service_GetDeviceIdentity = self.create_service(covvi_interfaces.srv.GetDeviceIdentity, full_service_name_GetDeviceIdentity, self.serviceCallbackGetDeviceIdentity)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDeviceIdentity}')

        full_service_name_GetDeviceProduct = self._get_full_ros2_name('GetDeviceProduct')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDeviceProduct}')
        self.service_GetDeviceProduct = self.create_service(covvi_interfaces.srv.GetDeviceProduct, full_service_name_GetDeviceProduct, self.serviceCallbackGetDeviceProduct)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDeviceProduct}')

        full_service_name_GetDigitConfig = self._get_full_ros2_name('GetDigitConfig')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitConfig}')
        self.service_GetDigitConfig = self.create_service(covvi_interfaces.srv.GetDigitConfig, full_service_name_GetDigitConfig, self.serviceCallbackGetDigitConfig)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitConfig}')

        full_service_name_GetDigitError = self._get_full_ros2_name('GetDigitError')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitError}')
        self.service_GetDigitError = self.create_service(covvi_interfaces.srv.GetDigitError, full_service_name_GetDigitError, self.serviceCallbackGetDigitError)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitError}')

        full_service_name_GetDigitPosn = self._get_full_ros2_name('GetDigitPosn')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitPosn}')
        self.service_GetDigitPosn = self.create_service(covvi_interfaces.srv.GetDigitPosn, full_service_name_GetDigitPosn, self.serviceCallbackGetDigitPosn)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitPosn}')

        full_service_name_GetDigitPosnAll = self._get_full_ros2_name('GetDigitPosnAll')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitPosnAll}')
        self.service_GetDigitPosnAll = self.create_service(covvi_interfaces.srv.GetDigitPosnAll, full_service_name_GetDigitPosnAll, self.serviceCallbackGetDigitPosnAll)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitPosnAll}')

        full_service_name_GetDigitStatus = self._get_full_ros2_name('GetDigitStatus')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitStatus}')
        self.service_GetDigitStatus = self.create_service(covvi_interfaces.srv.GetDigitStatus, full_service_name_GetDigitStatus, self.serviceCallbackGetDigitStatus)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitStatus}')

        full_service_name_GetDigitStatusAll = self._get_full_ros2_name('GetDigitStatusAll')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetDigitStatusAll}')
        self.service_GetDigitStatusAll = self.create_service(covvi_interfaces.srv.GetDigitStatusAll, full_service_name_GetDigitStatusAll, self.serviceCallbackGetDigitStatusAll)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetDigitStatusAll}')

        full_service_name_GetEnvironmental = self._get_full_ros2_name('GetEnvironmental')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetEnvironmental}')
        self.service_GetEnvironmental = self.create_service(covvi_interfaces.srv.GetEnvironmental, full_service_name_GetEnvironmental, self.serviceCallbackGetEnvironmental)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetEnvironmental}')

        full_service_name_GetFirmwarePICECI = self._get_full_ros2_name('GetFirmwarePICECI')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetFirmwarePICECI}')
        self.service_GetFirmwarePICECI = self.create_service(covvi_interfaces.srv.GetFirmwarePICECI, full_service_name_GetFirmwarePICECI, self.serviceCallbackGetFirmwarePICECI)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetFirmwarePICECI}')

        full_service_name_GetFirmwarePICHAND = self._get_full_ros2_name('GetFirmwarePICHAND')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetFirmwarePICHAND}')
        self.service_GetFirmwarePICHAND = self.create_service(covvi_interfaces.srv.GetFirmwarePICHAND, full_service_name_GetFirmwarePICHAND, self.serviceCallbackGetFirmwarePICHAND)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetFirmwarePICHAND}')

        full_service_name_GetGripName = self._get_full_ros2_name('GetGripName')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetGripName}')
        self.service_GetGripName = self.create_service(covvi_interfaces.srv.GetGripName, full_service_name_GetGripName, self.serviceCallbackGetGripName)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetGripName}')

        full_service_name_GetHello = self._get_full_ros2_name('GetHello')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetHello}')
        self.service_GetHello = self.create_service(covvi_interfaces.srv.GetHello, full_service_name_GetHello, self.serviceCallbackGetHello)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetHello}')

        full_service_name_GetMotorCurrent = self._get_full_ros2_name('GetMotorCurrent')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetMotorCurrent}')
        self.service_GetMotorCurrent = self.create_service(covvi_interfaces.srv.GetMotorCurrent, full_service_name_GetMotorCurrent, self.serviceCallbackGetMotorCurrent)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetMotorCurrent}')

        full_service_name_GetMotorCurrentAll = self._get_full_ros2_name('GetMotorCurrentAll')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetMotorCurrentAll}')
        self.service_GetMotorCurrentAll = self.create_service(covvi_interfaces.srv.GetMotorCurrentAll, full_service_name_GetMotorCurrentAll, self.serviceCallbackGetMotorCurrentAll)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetMotorCurrentAll}')

        full_service_name_GetMotorLimits = self._get_full_ros2_name('GetMotorLimits')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetMotorLimits}')
        self.service_GetMotorLimits = self.create_service(covvi_interfaces.srv.GetMotorLimits, full_service_name_GetMotorLimits, self.serviceCallbackGetMotorLimits)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetMotorLimits}')

        full_service_name_GetOrientation = self._get_full_ros2_name('GetOrientation')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetOrientation}')
        self.service_GetOrientation = self.create_service(covvi_interfaces.srv.GetOrientation, full_service_name_GetOrientation, self.serviceCallbackGetOrientation)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetOrientation}')

        full_service_name_GetPinchConfig = self._get_full_ros2_name('GetPinchConfig')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetPinchConfig}')
        self.service_GetPinchConfig = self.create_service(covvi_interfaces.srv.GetPinchConfig, full_service_name_GetPinchConfig, self.serviceCallbackGetPinchConfig)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetPinchConfig}')

        full_service_name_GetSystemStatus = self._get_full_ros2_name('GetSystemStatus')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_GetSystemStatus}')
        self.service_GetSystemStatus = self.create_service(covvi_interfaces.srv.GetSystemStatus, full_service_name_GetSystemStatus, self.serviceCallbackGetSystemStatus)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_GetSystemStatus}')

        full_service_name_RemoveUserGrip = self._get_full_ros2_name('RemoveUserGrip')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_RemoveUserGrip}')
        self.service_RemoveUserGrip = self.create_service(covvi_interfaces.srv.RemoveUserGrip, full_service_name_RemoveUserGrip, self.serviceCallbackRemoveUserGrip)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_RemoveUserGrip}')

        full_service_name_ResetRealtimeCfg = self._get_full_ros2_name('ResetRealtimeCfg')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_ResetRealtimeCfg}')
        self.service_ResetRealtimeCfg = self.create_service(covvi_interfaces.srv.ResetRealtimeCfg, full_service_name_ResetRealtimeCfg, self.serviceCallbackResetRealtimeCfg)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_ResetRealtimeCfg}')

        full_service_name_ResetUserGrips = self._get_full_ros2_name('ResetUserGrips')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_ResetUserGrips}')
        self.service_ResetUserGrips = self.create_service(covvi_interfaces.srv.ResetUserGrips, full_service_name_ResetUserGrips, self.serviceCallbackResetUserGrips)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_ResetUserGrips}')

        full_service_name_SendUserGrip = self._get_full_ros2_name('SendUserGrip')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SendUserGrip}')
        self.service_SendUserGrip = self.create_service(covvi_interfaces.srv.SendUserGrip, full_service_name_SendUserGrip, self.serviceCallbackSendUserGrip)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SendUserGrip}')

        full_service_name_SetCurrentGrip = self._get_full_ros2_name('SetCurrentGrip')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetCurrentGrip}')
        self.service_SetCurrentGrip = self.create_service(covvi_interfaces.srv.SetCurrentGrip, full_service_name_SetCurrentGrip, self.serviceCallbackSetCurrentGrip)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetCurrentGrip}')

        full_service_name_SetDigitMove = self._get_full_ros2_name('SetDigitMove')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDigitMove}')
        self.service_SetDigitMove = self.create_service(covvi_interfaces.srv.SetDigitMove, full_service_name_SetDigitMove, self.serviceCallbackSetDigitMove)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDigitMove}')

        full_service_name_SetDigitPosn = self._get_full_ros2_name('SetDigitPosn')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDigitPosn}')
        self.service_SetDigitPosn = self.create_service(covvi_interfaces.srv.SetDigitPosn, full_service_name_SetDigitPosn, self.serviceCallbackSetDigitPosn)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDigitPosn}')

        full_service_name_SetDigitPosnStop = self._get_full_ros2_name('SetDigitPosnStop')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDigitPosnStop}')
        self.service_SetDigitPosnStop = self.create_service(covvi_interfaces.srv.SetDigitPosnStop, full_service_name_SetDigitPosnStop, self.serviceCallbackSetDigitPosnStop)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDigitPosnStop}')

        full_service_name_SetDirectControlClose = self._get_full_ros2_name('SetDirectControlClose')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDirectControlClose}')
        self.service_SetDirectControlClose = self.create_service(covvi_interfaces.srv.SetDirectControlClose, full_service_name_SetDirectControlClose, self.serviceCallbackSetDirectControlClose)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDirectControlClose}')

        full_service_name_SetDirectControlOpen = self._get_full_ros2_name('SetDirectControlOpen')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDirectControlOpen}')
        self.service_SetDirectControlOpen = self.create_service(covvi_interfaces.srv.SetDirectControlOpen, full_service_name_SetDirectControlOpen, self.serviceCallbackSetDirectControlOpen)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDirectControlOpen}')

        full_service_name_SetDirectControlStop = self._get_full_ros2_name('SetDirectControlStop')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetDirectControlStop}')
        self.service_SetDirectControlStop = self.create_service(covvi_interfaces.srv.SetDirectControlStop, full_service_name_SetDirectControlStop, self.serviceCallbackSetDirectControlStop)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetDirectControlStop}')

        full_service_name_SetHandPowerOff = self._get_full_ros2_name('SetHandPowerOff')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetHandPowerOff}')
        self.service_SetHandPowerOff = self.create_service(covvi_interfaces.srv.SetHandPowerOff, full_service_name_SetHandPowerOff, self.serviceCallbackSetHandPowerOff)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetHandPowerOff}')

        full_service_name_SetHandPowerOn = self._get_full_ros2_name('SetHandPowerOn')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetHandPowerOn}')
        self.service_SetHandPowerOn = self.create_service(covvi_interfaces.srv.SetHandPowerOn, full_service_name_SetHandPowerOn, self.serviceCallbackSetHandPowerOn)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetHandPowerOn}')

        full_service_name_SetRealtimeCfg = self._get_full_ros2_name('SetRealtimeCfg')
        self.get_logger().info(f'Creating ROS2 Service: {full_service_name_SetRealtimeCfg}')
        self.service_SetRealtimeCfg = self.create_service(covvi_interfaces.srv.SetRealtimeCfg, full_service_name_SetRealtimeCfg, self.serviceCallbackSetRealtimeCfg)
        self.get_logger().info(f'Created ROS2 Service:  {full_service_name_SetRealtimeCfg}')

        self.get_logger().info(f'Created ROS2 Services')

        self.get_logger().info(f'Creating ROS2 Topic Publishers')

        full_publisher_name_DigitStatusAllMsg = self._get_full_ros2_name('DigitStatusAllMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_DigitStatusAllMsg}')
        self.publisher_DigitStatusAllMsg = self.create_publisher(covvi_interfaces.msg.DigitStatusAllMsg, full_publisher_name_DigitStatusAllMsg, 10)
        self.eci.callbackDigitStatusAll = self.publisherCallbackDigitStatusAllMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_DigitStatusAllMsg}')

        full_publisher_name_DigitPosnAllMsg = self._get_full_ros2_name('DigitPosnAllMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_DigitPosnAllMsg}')
        self.publisher_DigitPosnAllMsg = self.create_publisher(covvi_interfaces.msg.DigitPosnAllMsg, full_publisher_name_DigitPosnAllMsg, 10)
        self.eci.callbackDigitPosnAll = self.publisherCallbackDigitPosnAllMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_DigitPosnAllMsg}')

        full_publisher_name_CurrentGripMsg = self._get_full_ros2_name('CurrentGripMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_CurrentGripMsg}')
        self.publisher_CurrentGripMsg = self.create_publisher(covvi_interfaces.msg.CurrentGripMsg, full_publisher_name_CurrentGripMsg, 10)
        self.eci.callbackCurrentGrip = self.publisherCallbackCurrentGripMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_CurrentGripMsg}')

        full_publisher_name_ElectrodeValueMsg = self._get_full_ros2_name('ElectrodeValueMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_ElectrodeValueMsg}')
        self.publisher_ElectrodeValueMsg = self.create_publisher(covvi_interfaces.msg.ElectrodeValueMsg, full_publisher_name_ElectrodeValueMsg, 10)
        self.eci.callbackElectrodeValue = self.publisherCallbackElectrodeValueMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_ElectrodeValueMsg}')

        full_publisher_name_InputStatusMsg = self._get_full_ros2_name('InputStatusMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_InputStatusMsg}')
        self.publisher_InputStatusMsg = self.create_publisher(covvi_interfaces.msg.InputStatusMsg, full_publisher_name_InputStatusMsg, 10)
        self.eci.callbackInputStatus = self.publisherCallbackInputStatusMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_InputStatusMsg}')

        full_publisher_name_MotorCurrentAllMsg = self._get_full_ros2_name('MotorCurrentAllMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_MotorCurrentAllMsg}')
        self.publisher_MotorCurrentAllMsg = self.create_publisher(covvi_interfaces.msg.MotorCurrentAllMsg, full_publisher_name_MotorCurrentAllMsg, 10)
        self.eci.callbackMotorCurrentAll = self.publisherCallbackMotorCurrentAllMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_MotorCurrentAllMsg}')

        full_publisher_name_DigitTouchAllMsg = self._get_full_ros2_name('DigitTouchAllMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_DigitTouchAllMsg}')
        self.publisher_DigitTouchAllMsg = self.create_publisher(covvi_interfaces.msg.DigitTouchAllMsg, full_publisher_name_DigitTouchAllMsg, 10)
        self.eci.callbackDigitTouchAll = self.publisherCallbackDigitTouchAllMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_DigitTouchAllMsg}')

        full_publisher_name_EnvironmentalMsg = self._get_full_ros2_name('EnvironmentalMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_EnvironmentalMsg}')
        self.publisher_EnvironmentalMsg = self.create_publisher(covvi_interfaces.msg.EnvironmentalMsg, full_publisher_name_EnvironmentalMsg, 10)
        self.eci.callbackEnvironmental = self.publisherCallbackEnvironmentalMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_EnvironmentalMsg}')

        full_publisher_name_SystemStatusMsg = self._get_full_ros2_name('SystemStatusMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_SystemStatusMsg}')
        self.publisher_SystemStatusMsg = self.create_publisher(covvi_interfaces.msg.SystemStatusMsg, full_publisher_name_SystemStatusMsg, 10)
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_SystemStatusMsg}')

        full_publisher_name_OrientationMsg = self._get_full_ros2_name('OrientationMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_OrientationMsg}')
        self.publisher_OrientationMsg = self.create_publisher(covvi_interfaces.msg.OrientationMsg, full_publisher_name_OrientationMsg, 10)
        self.eci.callbackOrientation = self.publisherCallbackOrientationMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_OrientationMsg}')

        full_publisher_name_MotorLimitsMsg = self._get_full_ros2_name('MotorLimitsMsg')
        self.get_logger().info(f'Creating ROS2 Topic Publisher: {full_publisher_name_MotorLimitsMsg}')
        self.publisher_MotorLimitsMsg = self.create_publisher(covvi_interfaces.msg.MotorLimitsMsg, full_publisher_name_MotorLimitsMsg, 10)
        self.eci.callbackMotorLimits = self.publisherCallbackMotorLimitsMsg
        self.get_logger().info(f'Created ROS2 Topic Publisher:  {full_publisher_name_MotorLimitsMsg}')

        self.get_logger().info(f'Created ROS2 Topic Publishers')

    def serviceCallbackDisableAllRealtimeCfg(self,
            request:  covvi_interfaces.srv.DisableAllRealtimeCfg.Request,
            response: covvi_interfaces.srv.DisableAllRealtimeCfg.Response,
        )          -> covvi_interfaces.srv.DisableAllRealtimeCfg.Response:
        """"""
        self.get_logger().info('Calling eci.disableAllRealtimeCfg synchronously')
        msg: eci.RealtimeCfg = self.eci.disableAllRealtimeCfg()
        self.get_logger().info('Called eci.disableAllRealtimeCfg synchronously')
        self.get_logger().info('Building disableAllRealtimeCfg response')
        response.result                 = covvi_interfaces.msg.RealtimeCfg()
        response.result.digit_status    = bool(msg.digit_status)
        response.result.digit_posn      = bool(msg.digit_posn)
        response.result.current_grip    = bool(msg.current_grip)
        response.result.electrode_value = bool(msg.electrode_value)
        response.result.input_status    = bool(msg.input_status)
        response.result.motor_current   = bool(msg.motor_current)
        response.result.digit_touch     = bool(msg.digit_touch)
        response.result.digit_error     = bool(msg.digit_error)
        response.result.environmental   = bool(msg.environmental)
        response.result.orientation     = bool(msg.orientation)
        response.result.motor_limits    = bool(msg.motor_limits)
        self.get_logger().info('Built disableAllRealtimeCfg response')
        self.get_logger().info('Sending disableAllRealtimeCfg response to client')
        return response

    def serviceCallbackEnableAllRealtimeCfg(self,
            request:  covvi_interfaces.srv.EnableAllRealtimeCfg.Request,
            response: covvi_interfaces.srv.EnableAllRealtimeCfg.Response,
        )          -> covvi_interfaces.srv.EnableAllRealtimeCfg.Response:
        """"""
        self.get_logger().info('Calling eci.enableAllRealtimeCfg synchronously')
        msg: eci.RealtimeCfg = self.eci.enableAllRealtimeCfg()
        self.get_logger().info('Called eci.enableAllRealtimeCfg synchronously')
        self.get_logger().info('Building enableAllRealtimeCfg response')
        response.result                 = covvi_interfaces.msg.RealtimeCfg()
        response.result.digit_status    = bool(msg.digit_status)
        response.result.digit_posn      = bool(msg.digit_posn)
        response.result.current_grip    = bool(msg.current_grip)
        response.result.electrode_value = bool(msg.electrode_value)
        response.result.input_status    = bool(msg.input_status)
        response.result.motor_current   = bool(msg.motor_current)
        response.result.digit_touch     = bool(msg.digit_touch)
        response.result.digit_error     = bool(msg.digit_error)
        response.result.environmental   = bool(msg.environmental)
        response.result.orientation     = bool(msg.orientation)
        response.result.motor_limits    = bool(msg.motor_limits)
        self.get_logger().info('Built enableAllRealtimeCfg response')
        self.get_logger().info('Sending enableAllRealtimeCfg response to client')
        return response

    def serviceCallbackGetCurrentGrip(self,
            request:  covvi_interfaces.srv.GetCurrentGrip.Request,
            response: covvi_interfaces.srv.GetCurrentGrip.Response,
        )          -> covvi_interfaces.srv.GetCurrentGrip.Response:
        """Get the current grip config"""
        self.get_logger().info('Calling eci.getCurrentGrip synchronously')
        msg: eci.CurrentGrip = self.eci.getCurrentGrip()
        self.get_logger().info('Called eci.getCurrentGrip synchronously')
        self.get_logger().info('Building getCurrentGrip response')
        response.result             = covvi_interfaces.msg.CurrentGrip()
        response.result.value       = covvi_interfaces.msg.CurrentGripID()
        response.result.value.value = eci.CurrentGripID(msg.value.value).value
        self.get_logger().info('Built getCurrentGrip response')
        self.get_logger().info('Sending getCurrentGrip response to client')
        return response

    def serviceCallbackGetDeviceIdentity(self,
            request:  covvi_interfaces.srv.GetDeviceIdentity.Request,
            response: covvi_interfaces.srv.GetDeviceIdentity.Response,
        )          -> covvi_interfaces.srv.GetDeviceIdentity.Response:
        """"""
        self.get_logger().info('Calling eci.getDeviceIdentity synchronously')
        msg: eci.DeviceIdentityMsg = self.eci.getDeviceIdentity()
        self.get_logger().info('Called eci.getDeviceIdentity synchronously')
        self.get_logger().info('Building getDeviceIdentity response')
        response.result                        = covvi_interfaces.msg.DeviceIdentityMsg()
        response.result.uid                    = str(msg.uid)
        response.result.dev_id                 = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value           = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type               = covvi_interfaces.msg.Command()
        response.result.cmd_type.value         = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value   = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id                 = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value           = eci.MessageID(msg.msg_id.value).value
        response.result.data_len               = int(msg.data_len)
        response.result.type                   = int(msg.type)
        response.result.wrist                  = bool(msg.wrist)
        response.result.glove                  = covvi_interfaces.msg.DeviceGlove()
        response.result.glove.value            = eci.DeviceGlove(msg.glove.value).value
        response.result.colour                 = covvi_interfaces.msg.DeviceColour()
        response.result.colour.value           = eci.DeviceColour(msg.colour.value).value
        response.result.language               = covvi_interfaces.msg.Language()
        response.result.language.value         = eci.Language(msg.language.value).value
        response.result.hw_version             = int(msg.hw_version)
        response.result.year_of_manufacture    = int(msg.year_of_manufacture)
        response.result.extended_warranty      = int(msg.extended_warranty)
        response.result.warranty_expires_month = int(msg.warranty_expires_month)
        response.result.warranty_expires_year  = int(msg.warranty_expires_year)
        self.get_logger().info('Built getDeviceIdentity response')
        self.get_logger().info('Sending getDeviceIdentity response to client')
        return response

    def serviceCallbackGetDeviceProduct(self,
            request:  covvi_interfaces.srv.GetDeviceProduct.Request,
            response: covvi_interfaces.srv.GetDeviceProduct.Response,
        )          -> covvi_interfaces.srv.GetDeviceProduct.Response:
        """"""
        self.get_logger().info('Calling eci.getDeviceProduct synchronously')
        msg: eci.DeviceProductMsg = self.eci.getDeviceProduct()
        self.get_logger().info('Called eci.getDeviceProduct synchronously')
        self.get_logger().info('Building getDeviceProduct response')
        response.result                        = covvi_interfaces.msg.DeviceProductMsg()
        response.result.uid                    = str(msg.uid)
        response.result.dev_id                 = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value           = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type               = covvi_interfaces.msg.Command()
        response.result.cmd_type.value         = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value   = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id                 = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value           = eci.MessageID(msg.msg_id.value).value
        response.result.data_len               = int(msg.data_len)
        response.result.manufacturer_id        = eci.Uint8(msg.manufacturer_id)
        response.result.product_id             = covvi_interfaces.msg.Product()
        response.result.product_id.value       = covvi_interfaces.msg.ProductString()
        response.result.product_id.value.value = eci.ProductString(msg.product_id.value.value).value
        self.get_logger().info('Built getDeviceProduct response')
        self.get_logger().info('Sending getDeviceProduct response to client')
        return response

    def serviceCallbackGetDigitConfig(self,
            request:  covvi_interfaces.srv.GetDigitConfig.Request,
            response: covvi_interfaces.srv.GetDigitConfig.Response,
        )          -> covvi_interfaces.srv.GetDigitConfig.Response:
        """Get digit limits"""
        self.get_logger().info('Setting up getDigitConfig call')
        digit = eci.Digit(request.digit.value)
        self.get_logger().info('getDigitConfig call has been setup')
        self.get_logger().info('Calling eci.getDigitConfig synchronously')
        msg: eci.DigitConfigMsg = self.eci.getDigitConfig(
            digit = digit,
        )
        self.get_logger().info('Called eci.getDigitConfig synchronously')
        self.get_logger().info('Building getDigitConfig response')
        response.result                      = covvi_interfaces.msg.DigitConfigMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.open_limit           = int(msg.open_limit)
        response.result.close_limit          = int(msg.close_limit)
        response.result.offset               = int(msg.offset)
        self.get_logger().info('Built getDigitConfig response')
        self.get_logger().info('Sending getDigitConfig response to client')
        return response

    def serviceCallbackGetDigitError(self,
            request:  covvi_interfaces.srv.GetDigitError.Request,
            response: covvi_interfaces.srv.GetDigitError.Response,
        )          -> covvi_interfaces.srv.GetDigitError.Response:
        """Get digit error flags"""
        self.get_logger().info('Setting up getDigitError call')
        digit = eci.Digit(request.digit.value)
        self.get_logger().info('getDigitError call has been setup')
        self.get_logger().info('Calling eci.getDigitError synchronously')
        msg: eci.DigitErrorMsg = self.eci.getDigitError(
            digit = digit,
        )
        self.get_logger().info('Called eci.getDigitError synchronously')
        self.get_logger().info('Building getDigitError response')
        response.result                      = covvi_interfaces.msg.DigitErrorMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.position             = bool(msg.position)
        response.result.limits               = bool(msg.limits)
        response.result.motor                = bool(msg.motor)
        response.result.hall                 = bool(msg.hall)
        self.get_logger().info('Built getDigitError response')
        self.get_logger().info('Sending getDigitError response to client')
        return response

    def serviceCallbackGetDigitPosn(self,
            request:  covvi_interfaces.srv.GetDigitPosn.Request,
            response: covvi_interfaces.srv.GetDigitPosn.Response,
        )          -> covvi_interfaces.srv.GetDigitPosn.Response:
        """Get the digit position"""
        self.get_logger().info('Setting up getDigitPosn call')
        digit = eci.Digit(request.digit.value)
        self.get_logger().info('getDigitPosn call has been setup')
        self.get_logger().info('Calling eci.getDigitPosn synchronously')
        msg: eci.DigitPosnMsg = self.eci.getDigitPosn(
            digit = digit,
        )
        self.get_logger().info('Called eci.getDigitPosn synchronously')
        self.get_logger().info('Building getDigitPosn response')
        response.result                      = covvi_interfaces.msg.DigitPosnMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.pos                  = int(msg.pos)
        self.get_logger().info('Built getDigitPosn response')
        self.get_logger().info('Sending getDigitPosn response to client')
        return response

    def serviceCallbackGetDigitPosnAll(self,
            request:  covvi_interfaces.srv.GetDigitPosnAll.Request,
            response: covvi_interfaces.srv.GetDigitPosnAll.Response,
        )          -> covvi_interfaces.srv.GetDigitPosnAll.Response:
        """Get all digit positions"""
        self.get_logger().info('Calling eci.getDigitPosn_all synchronously')
        msg: eci.DigitPosnAllMsg = self.eci.getDigitPosn_all()
        self.get_logger().info('Called eci.getDigitPosn_all synchronously')
        self.get_logger().info('Building getDigitPosn_all response')
        response.result                      = covvi_interfaces.msg.DigitPosnAllMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.thumb_pos            = int(msg.thumb_pos)
        response.result.index_pos            = int(msg.index_pos)
        response.result.middle_pos           = int(msg.middle_pos)
        response.result.ring_pos             = int(msg.ring_pos)
        response.result.little_pos           = int(msg.little_pos)
        response.result.rotate_pos           = int(msg.rotate_pos)
        self.get_logger().info('Built getDigitPosn_all response')
        self.get_logger().info('Sending getDigitPosn_all response to client')
        return response

    def serviceCallbackGetDigitStatus(self,
            request:  covvi_interfaces.srv.GetDigitStatus.Request,
            response: covvi_interfaces.srv.GetDigitStatus.Response,
        )          -> covvi_interfaces.srv.GetDigitStatus.Response:
        """Get the digit status flags"""
        self.get_logger().info('Setting up getDigitStatus call')
        digit = eci.Digit(request.digit.value)
        self.get_logger().info('getDigitStatus call has been setup')
        self.get_logger().info('Calling eci.getDigitStatus synchronously')
        msg: eci.DigitStatusMsg = self.eci.getDigitStatus(
            digit = digit,
        )
        self.get_logger().info('Called eci.getDigitStatus synchronously')
        self.get_logger().info('Building getDigitStatus response')
        response.result                      = covvi_interfaces.msg.DigitStatusMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.fault                = bool(msg.fault)
        response.result.gripping             = bool(msg.gripping)
        response.result.at_open              = bool(msg.at_open)
        response.result.at_posn              = bool(msg.at_posn)
        response.result.touch                = bool(msg.touch)
        response.result.stall                = bool(msg.stall)
        response.result.stopped              = bool(msg.stopped)
        response.result.active               = bool(msg.active)
        self.get_logger().info('Built getDigitStatus response')
        self.get_logger().info('Sending getDigitStatus response to client')
        return response

    def serviceCallbackGetDigitStatusAll(self,
            request:  covvi_interfaces.srv.GetDigitStatusAll.Request,
            response: covvi_interfaces.srv.GetDigitStatusAll.Response,
        )          -> covvi_interfaces.srv.GetDigitStatusAll.Response:
        """Get all digit status flags"""
        self.get_logger().info('Calling eci.getDigitStatus_all synchronously')
        msg: eci.DigitStatusAllMsg = self.eci.getDigitStatus_all()
        self.get_logger().info('Called eci.getDigitStatus_all synchronously')
        self.get_logger().info('Building getDigitStatus_all response')
        response.result                      = covvi_interfaces.msg.DigitStatusAllMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.thumb_fault          = bool(msg.thumb_fault)
        response.result.thumb_gripping       = bool(msg.thumb_gripping)
        response.result.thumb_at_open        = bool(msg.thumb_at_open)
        response.result.thumb_at_posn        = bool(msg.thumb_at_posn)
        response.result.thumb_touch          = bool(msg.thumb_touch)
        response.result.thumb_stall          = bool(msg.thumb_stall)
        response.result.thumb_stopped        = bool(msg.thumb_stopped)
        response.result.thumb_active         = bool(msg.thumb_active)
        response.result.index_fault          = bool(msg.index_fault)
        response.result.index_gripping       = bool(msg.index_gripping)
        response.result.index_at_open        = bool(msg.index_at_open)
        response.result.index_at_posn        = bool(msg.index_at_posn)
        response.result.index_touch          = bool(msg.index_touch)
        response.result.index_stall          = bool(msg.index_stall)
        response.result.index_stopped        = bool(msg.index_stopped)
        response.result.index_active         = bool(msg.index_active)
        response.result.middle_fault         = bool(msg.middle_fault)
        response.result.middle_gripping      = bool(msg.middle_gripping)
        response.result.middle_at_open       = bool(msg.middle_at_open)
        response.result.middle_at_posn       = bool(msg.middle_at_posn)
        response.result.middle_touch         = bool(msg.middle_touch)
        response.result.middle_stall         = bool(msg.middle_stall)
        response.result.middle_stopped       = bool(msg.middle_stopped)
        response.result.middle_active        = bool(msg.middle_active)
        response.result.ring_fault           = bool(msg.ring_fault)
        response.result.ring_gripping        = bool(msg.ring_gripping)
        response.result.ring_at_open         = bool(msg.ring_at_open)
        response.result.ring_at_posn         = bool(msg.ring_at_posn)
        response.result.ring_touch           = bool(msg.ring_touch)
        response.result.ring_stall           = bool(msg.ring_stall)
        response.result.ring_stopped         = bool(msg.ring_stopped)
        response.result.ring_active          = bool(msg.ring_active)
        response.result.little_fault         = bool(msg.little_fault)
        response.result.little_gripping      = bool(msg.little_gripping)
        response.result.little_at_open       = bool(msg.little_at_open)
        response.result.little_at_posn       = bool(msg.little_at_posn)
        response.result.little_touch         = bool(msg.little_touch)
        response.result.little_stall         = bool(msg.little_stall)
        response.result.little_stopped       = bool(msg.little_stopped)
        response.result.little_active        = bool(msg.little_active)
        response.result.rotate_fault         = bool(msg.rotate_fault)
        response.result.rotate_gripping      = bool(msg.rotate_gripping)
        response.result.rotate_at_open       = bool(msg.rotate_at_open)
        response.result.rotate_at_posn       = bool(msg.rotate_at_posn)
        response.result.rotate_touch         = bool(msg.rotate_touch)
        response.result.rotate_stall         = bool(msg.rotate_stall)
        response.result.rotate_stopped       = bool(msg.rotate_stopped)
        response.result.rotate_active        = bool(msg.rotate_active)
        self.get_logger().info('Built getDigitStatus_all response')
        self.get_logger().info('Sending getDigitStatus_all response to client')
        return response

    def serviceCallbackGetEnvironmental(self,
            request:  covvi_interfaces.srv.GetEnvironmental.Request,
            response: covvi_interfaces.srv.GetEnvironmental.Response,
        )          -> covvi_interfaces.srv.GetEnvironmental.Response:
        """Read temperature, battery voltage etc

        Temperature     (C)
        Humidity        (0-100%)
        Battery Voltage (mV)
        """
        self.get_logger().info('Calling eci.getEnvironmental synchronously')
        msg: eci.EnvironmentalMsg = self.eci.getEnvironmental()
        self.get_logger().info('Called eci.getEnvironmental synchronously')
        self.get_logger().info('Building getEnvironmental response')
        response.result                      = covvi_interfaces.msg.EnvironmentalMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.temperature          = int(msg.temperature)
        response.result.humidity             = int(msg.humidity)
        response.result.battery_voltage      = eci.Int16(msg.battery_voltage)
        self.get_logger().info('Built getEnvironmental response')
        self.get_logger().info('Sending getEnvironmental response to client')
        return response

    def serviceCallbackGetFirmwarePICECI(self,
            request:  covvi_interfaces.srv.GetFirmwarePICECI.Request,
            response: covvi_interfaces.srv.GetFirmwarePICECI.Response,
        )          -> covvi_interfaces.srv.GetFirmwarePICECI.Response:
        """"""
        self.get_logger().info('Calling eci.getFirmware_PIC_ECI synchronously')
        msg: eci.EciFirmwarePicMsg = self.eci.getFirmware_PIC_ECI()
        self.get_logger().info('Called eci.getFirmware_PIC_ECI synchronously')
        self.get_logger().info('Building getFirmware_PIC_ECI response')
        response.result                      = covvi_interfaces.msg.EciFirmwarePicMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.revision             = int(msg.revision)
        response.result.major                = int(msg.major)
        response.result.minor                = int(msg.minor)
        self.get_logger().info('Built getFirmware_PIC_ECI response')
        self.get_logger().info('Sending getFirmware_PIC_ECI response to client')
        return response

    def serviceCallbackGetFirmwarePICHAND(self,
            request:  covvi_interfaces.srv.GetFirmwarePICHAND.Request,
            response: covvi_interfaces.srv.GetFirmwarePICHAND.Response,
        )          -> covvi_interfaces.srv.GetFirmwarePICHAND.Response:
        """"""
        self.get_logger().info('Calling eci.getFirmware_PIC_HAND synchronously')
        msg: eci.HandFirmwarePicMsg = self.eci.getFirmware_PIC_HAND()
        self.get_logger().info('Called eci.getFirmware_PIC_HAND synchronously')
        self.get_logger().info('Building getFirmware_PIC_HAND response')
        response.result                      = covvi_interfaces.msg.HandFirmwarePicMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.revision             = int(msg.revision)
        response.result.major                = int(msg.major)
        response.result.minor                = int(msg.minor)
        self.get_logger().info('Built getFirmware_PIC_HAND response')
        self.get_logger().info('Sending getFirmware_PIC_HAND response to client')
        return response

    def serviceCallbackGetGripName(self,
            request:  covvi_interfaces.srv.GetGripName.Request,
            response: covvi_interfaces.srv.GetGripName.Response,
        )          -> covvi_interfaces.srv.GetGripName.Response:
        """Get user grip name"""
        self.get_logger().info('Setting up getGripName call')
        grip_name_index = eci.GripNameIndex(request.grip_name_index.value)
        self.get_logger().info('getGripName call has been setup')
        self.get_logger().info('Calling eci.getGripName synchronously')
        msg: eci.GripName = self.eci.getGripName(
            grip_name_index = grip_name_index,
        )
        self.get_logger().info('Called eci.getGripName synchronously')
        self.get_logger().info('Building getGripName response')
        response.result       = covvi_interfaces.msg.GripName()
        response.result.value = str(msg.value)
        self.get_logger().info('Built getGripName response')
        self.get_logger().info('Sending getGripName response to client')
        return response

    def serviceCallbackGetHello(self,
            request:  covvi_interfaces.srv.GetHello.Request,
            response: covvi_interfaces.srv.GetHello.Response,
        )          -> covvi_interfaces.srv.GetHello.Response:
        """"""
        self.get_logger().info('Calling eci.getHello synchronously')
        msg: eci.HelloMsg = self.eci.getHello()
        self.get_logger().info('Called eci.getHello synchronously')
        self.get_logger().info('Building getHello response')
        response.result                      = covvi_interfaces.msg.HelloMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        self.get_logger().info('Built getHello response')
        self.get_logger().info('Sending getHello response to client')
        return response

    def serviceCallbackGetMotorCurrent(self,
            request:  covvi_interfaces.srv.GetMotorCurrent.Request,
            response: covvi_interfaces.srv.GetMotorCurrent.Response,
        )          -> covvi_interfaces.srv.GetMotorCurrent.Response:
        """Get motor current

        Motor current is not available for rotation motor,
        The current value is in multiples of 16mA. e.g. 1 = 16mA, 64 = 1024mA
        """
        self.get_logger().info('Setting up getMotorCurrent call')
        digit = eci.Digit5(request.digit.value)
        self.get_logger().info('getMotorCurrent call has been setup')
        self.get_logger().info('Calling eci.getMotorCurrent synchronously')
        msg: eci.MotorCurrentMsg = self.eci.getMotorCurrent(
            digit = digit,
        )
        self.get_logger().info('Called eci.getMotorCurrent synchronously')
        self.get_logger().info('Building getMotorCurrent response')
        response.result                      = covvi_interfaces.msg.MotorCurrentMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.current              = int(msg.current)
        self.get_logger().info('Built getMotorCurrent response')
        self.get_logger().info('Sending getMotorCurrent response to client')
        return response

    def serviceCallbackGetMotorCurrentAll(self,
            request:  covvi_interfaces.srv.GetMotorCurrentAll.Request,
            response: covvi_interfaces.srv.GetMotorCurrentAll.Response,
        )          -> covvi_interfaces.srv.GetMotorCurrentAll.Response:
        """Get all motor currents

        Motor current is not available for rotation motor,
        The current value is in multiples of 16mA. e.g. 1 = 16mA, 64 = 1024mA
        """
        self.get_logger().info('Calling eci.getMotorCurrent_all synchronously')
        msg: eci.MotorCurrentAllMsg = self.eci.getMotorCurrent_all()
        self.get_logger().info('Called eci.getMotorCurrent_all synchronously')
        self.get_logger().info('Building getMotorCurrent_all response')
        response.result                      = covvi_interfaces.msg.MotorCurrentAllMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.thumb_current        = int(msg.thumb_current)
        response.result.index_current        = int(msg.index_current)
        response.result.middle_current       = int(msg.middle_current)
        response.result.ring_current         = int(msg.ring_current)
        response.result.little_current       = int(msg.little_current)
        self.get_logger().info('Built getMotorCurrent_all response')
        self.get_logger().info('Sending getMotorCurrent_all response to client')
        return response

    def serviceCallbackGetMotorLimits(self,
            request:  covvi_interfaces.srv.GetMotorLimits.Request,
            response: covvi_interfaces.srv.GetMotorLimits.Response,
        )          -> covvi_interfaces.srv.GetMotorLimits.Response:
        """Get motor limits"""
        self.get_logger().info('Calling eci.getMotorLimits synchronously')
        msg: eci.MotorLimitsMsg = self.eci.getMotorLimits()
        self.get_logger().info('Called eci.getMotorLimits synchronously')
        self.get_logger().info('Building getMotorLimits response')
        response.result                      = covvi_interfaces.msg.MotorLimitsMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.hand                 = bool(msg.hand)
        response.result.eci                  = bool(msg.eci)
        response.result.ltl                  = bool(msg.ltl)
        response.result.rng                  = bool(msg.rng)
        response.result.mid                  = bool(msg.mid)
        response.result.idx                  = bool(msg.idx)
        response.result.thb                  = bool(msg.thb)
        response.result.thumb_derate_value   = int(msg.thumb_derate_value)
        response.result.index_derate_value   = int(msg.index_derate_value)
        response.result.middle_derate_value  = int(msg.middle_derate_value)
        response.result.ring_derate_value    = int(msg.ring_derate_value)
        response.result.little_derate_value  = int(msg.little_derate_value)
        self.get_logger().info('Built getMotorLimits response')
        self.get_logger().info('Sending getMotorLimits response to client')
        return response

    def serviceCallbackGetOrientation(self,
            request:  covvi_interfaces.srv.GetOrientation.Request,
            response: covvi_interfaces.srv.GetOrientation.Response,
        )          -> covvi_interfaces.srv.GetOrientation.Response:
        """Get hand orientation

        X Position
        Y Position
        Z Position
        """
        self.get_logger().info('Calling eci.getOrientation synchronously')
        msg: eci.OrientationMsg = self.eci.getOrientation()
        self.get_logger().info('Called eci.getOrientation synchronously')
        self.get_logger().info('Building getOrientation response')
        response.result                      = covvi_interfaces.msg.OrientationMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.x                    = eci.Int16(msg.x)
        response.result.y                    = eci.Int16(msg.y)
        response.result.z                    = eci.Int16(msg.z)
        self.get_logger().info('Built getOrientation response')
        self.get_logger().info('Sending getOrientation response to client')
        return response

    def serviceCallbackGetPinchConfig(self,
            request:  covvi_interfaces.srv.GetPinchConfig.Request,
            response: covvi_interfaces.srv.GetPinchConfig.Response,
        )          -> covvi_interfaces.srv.GetPinchConfig.Response:
        """Get pinch points"""
        self.get_logger().info('Calling eci.getPinchConfig synchronously')
        msg: eci.PinchConfigMsg = self.eci.getPinchConfig()
        self.get_logger().info('Called eci.getPinchConfig synchronously')
        self.get_logger().info('Building getPinchConfig response')
        response.result                       = covvi_interfaces.msg.PinchConfigMsg()
        response.result.uid                   = str(msg.uid)
        response.result.dev_id                = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value          = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type              = covvi_interfaces.msg.Command()
        response.result.cmd_type.value        = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value  = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id                = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value          = eci.MessageID(msg.msg_id.value).value
        response.result.data_len              = int(msg.data_len)
        response.result.thumb_pos             = int(msg.thumb_pos)
        response.result.index_pos             = int(msg.index_pos)
        response.result.middle_pos            = int(msg.middle_pos)
        response.result.one_finger_rotate_pos = int(msg.one_finger_rotate_pos)
        response.result.two_finger_rotate_pos = int(msg.two_finger_rotate_pos)
        self.get_logger().info('Built getPinchConfig response')
        self.get_logger().info('Sending getPinchConfig response to client')
        return response

    def serviceCallbackGetSystemStatus(self,
            request:  covvi_interfaces.srv.GetSystemStatus.Request,
            response: covvi_interfaces.srv.GetSystemStatus.Response,
        )          -> covvi_interfaces.srv.GetSystemStatus.Response:
        """Read system status

        Critical error flags
        Non-fatal errors
        Bluetooth Status
        Change Notifications
        """
        self.get_logger().info('Calling eci.getSystemStatus synchronously')
        msg: eci.SystemStatusMsg = self.eci.getSystemStatus()
        self.get_logger().info('Called eci.getSystemStatus synchronously')
        self.get_logger().info('Building getSystemStatus response')
        response.result                      = covvi_interfaces.msg.SystemStatusMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.bluetooth_fault      = bool(msg.bluetooth_fault)
        response.result.spi_error            = bool(msg.spi_error)
        response.result.gateway_error        = bool(msg.gateway_error)
        response.result.humidity_limit       = bool(msg.humidity_limit)
        response.result.temperature_limit    = bool(msg.temperature_limit)
        response.result.bluetooth_status     = int(msg.bluetooth_status)
        response.result.change_notifications = int(msg.change_notifications)
        self.get_logger().info('Built getSystemStatus response')
        self.get_logger().info('Sending getSystemStatus response to client')
        return response

    def serviceCallbackRemoveUserGrip(self,
            request:  covvi_interfaces.srv.RemoveUserGrip.Request,
            response: covvi_interfaces.srv.RemoveUserGrip.Response,
        )          -> covvi_interfaces.srv.RemoveUserGrip.Response:
        """"""
        self.get_logger().info('Setting up removeUserGrip call')
        grip_name_index = eci.GripNameIndex(request.grip_name_index.value)
        self.get_logger().info('removeUserGrip call has been setup')
        self.get_logger().info('Calling eci.removeUserGrip synchronously')
        msg: eci.UserGripResMsg = self.eci.removeUserGrip(
            grip_name_index = grip_name_index,
        )
        self.get_logger().info('Called eci.removeUserGrip synchronously')
        self.get_logger().info('Building removeUserGrip response')
        response.result                       = covvi_interfaces.msg.UserGripResMsg()
        response.result.uid                   = str(msg.uid)
        response.result.dev_id                = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value          = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type              = covvi_interfaces.msg.Command()
        response.result.cmd_type.value        = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value  = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id                = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value          = eci.MessageID(msg.msg_id.value).value
        response.result.data_len              = int(msg.data_len)
        response.result.data_type             = covvi_interfaces.msg.BulkDataType()
        response.result.data_type.value       = eci.BulkDataType(msg.data_type.value).value
        response.result.grip_name_index       = covvi_interfaces.msg.GripNameIndex()
        response.result.grip_name_index.value = eci.GripNameIndex(msg.grip_name_index.value).value
        response.result.update_status         = covvi_interfaces.msg.UpdateStatus()
        response.result.update_status.value   = eci.UpdateStatus(msg.update_status.value).value
        self.get_logger().info('Built removeUserGrip response')
        self.get_logger().info('Sending removeUserGrip response to client')
        return response

    def serviceCallbackResetRealtimeCfg(self,
            request:  covvi_interfaces.srv.ResetRealtimeCfg.Request,
            response: covvi_interfaces.srv.ResetRealtimeCfg.Response,
        )          -> covvi_interfaces.srv.ResetRealtimeCfg.Response:
        """"""
        self.get_logger().info('Calling eci.resetRealtimeCfg synchronously')
        self.eci.resetRealtimeCfg()
        self.get_logger().info('Called eci.resetRealtimeCfg synchronously')
        self.get_logger().info('Building resetRealtimeCfg response')
        self.get_logger().info('Built resetRealtimeCfg response')
        self.get_logger().info('Sending resetRealtimeCfg response to client')
        return response

    def serviceCallbackResetUserGrips(self,
            request:  covvi_interfaces.srv.ResetUserGrips.Request,
            response: covvi_interfaces.srv.ResetUserGrips.Response,
        )          -> covvi_interfaces.srv.ResetUserGrips.Response:
        """"""
        self.get_logger().info('Calling eci.resetUserGrips synchronously')
        self.eci.resetUserGrips()
        self.get_logger().info('Called eci.resetUserGrips synchronously')
        self.get_logger().info('Building resetUserGrips response')
        self.get_logger().info('Built resetUserGrips response')
        self.get_logger().info('Sending resetUserGrips response to client')
        return response

    def serviceCallbackSendUserGrip(self,
            request:  covvi_interfaces.srv.SendUserGrip.Request,
            response: covvi_interfaces.srv.SendUserGrip.Response,
        )          -> covvi_interfaces.srv.SendUserGrip.Response:
        """"""
        self.get_logger().info('Setting up sendUserGrip call')
        grip_name_index = eci.GripNameIndex(request.grip_name_index.value)
        user_grip = eci.UserGripID(request.user_grip.value)
        self.get_logger().info('sendUserGrip call has been setup')
        self.get_logger().info('Calling eci.sendUserGrip synchronously')
        msg: eci.UserGripResMsg = self.eci.sendUserGrip(
            grip_name_index = grip_name_index,
            user_grip       = user_grip,
        )
        self.get_logger().info('Called eci.sendUserGrip synchronously')
        self.get_logger().info('Building sendUserGrip response')
        response.result                       = covvi_interfaces.msg.UserGripResMsg()
        response.result.uid                   = str(msg.uid)
        response.result.dev_id                = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value          = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type              = covvi_interfaces.msg.Command()
        response.result.cmd_type.value        = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value  = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id                = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value          = eci.MessageID(msg.msg_id.value).value
        response.result.data_len              = int(msg.data_len)
        response.result.data_type             = covvi_interfaces.msg.BulkDataType()
        response.result.data_type.value       = eci.BulkDataType(msg.data_type.value).value
        response.result.grip_name_index       = covvi_interfaces.msg.GripNameIndex()
        response.result.grip_name_index.value = eci.GripNameIndex(msg.grip_name_index.value).value
        response.result.update_status         = covvi_interfaces.msg.UpdateStatus()
        response.result.update_status.value   = eci.UpdateStatus(msg.update_status.value).value
        self.get_logger().info('Built sendUserGrip response')
        self.get_logger().info('Sending sendUserGrip response to client')
        return response

    def serviceCallbackSetCurrentGrip(self,
            request:  covvi_interfaces.srv.SetCurrentGrip.Request,
            response: covvi_interfaces.srv.SetCurrentGrip.Response,
        )          -> covvi_interfaces.srv.SetCurrentGrip.Response:
        """Set the current grip via the Grip ID"""
        self.get_logger().info('Setting up setCurrentGrip call')
        grip_id = eci.CurrentGripID(request.grip_id.value)
        self.get_logger().info('setCurrentGrip call has been setup')
        self.get_logger().info('Calling eci.setCurrentGrip synchronously')
        msg: eci.CurrentGripGripIdMsg = self.eci.setCurrentGrip(
            grip_id = grip_id,
        )
        self.get_logger().info('Called eci.setCurrentGrip synchronously')
        self.get_logger().info('Building setCurrentGrip response')
        response.result                      = covvi_interfaces.msg.CurrentGripGripIdMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.grip_id              = covvi_interfaces.msg.CurrentGripID()
        response.result.grip_id.value        = eci.CurrentGripID(msg.grip_id.value).value
        self.get_logger().info('Built setCurrentGrip response')
        self.get_logger().info('Sending setCurrentGrip response to client')
        return response

    def serviceCallbackSetDigitMove(self,
            request:  covvi_interfaces.srv.SetDigitMove.Request,
            response: covvi_interfaces.srv.SetDigitMove.Response,
        )          -> covvi_interfaces.srv.SetDigitMove.Response:
        """Command to move a single digit"""
        self.get_logger().info('Setting up setDigitMove call')
        digit = eci.Digit(request.digit.value)
        position = int(request.position)
        speed       = eci.Percentage()
        speed.value = int(request.speed.value)
        power       = eci.Percentage()
        power.value = int(request.power.value)
        limit       = eci.Percentage()
        limit.value = int(request.limit.value)
        self.get_logger().info('setDigitMove call has been setup')
        self.get_logger().info('Calling eci.setDigitMove synchronously')
        msg: eci.DigitMoveMsg = self.eci.setDigitMove(
            digit    = digit,
            position = position,
            speed    = speed,
            power    = power,
            limit    = limit,
        )
        self.get_logger().info('Called eci.setDigitMove synchronously')
        self.get_logger().info('Building setDigitMove response')
        response.result                      = covvi_interfaces.msg.DigitMoveMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.position             = int(msg.position)
        response.result.speed                = covvi_interfaces.msg.Percentage()
        response.result.speed.value          = int(msg.speed.value)
        response.result.power                = covvi_interfaces.msg.Percentage()
        response.result.power.value          = int(msg.power.value)
        response.result.limit                = covvi_interfaces.msg.Percentage()
        response.result.limit.value          = int(msg.limit.value)
        self.get_logger().info('Built setDigitMove response')
        self.get_logger().info('Sending setDigitMove response to client')
        return response

    def serviceCallbackSetDigitPosn(self,
            request:  covvi_interfaces.srv.SetDigitPosn.Request,
            response: covvi_interfaces.srv.SetDigitPosn.Response,
        )          -> covvi_interfaces.srv.SetDigitPosn.Response:
        """Set the digit position to move to and the movement speed for each digit and thumb rotation"""
        self.get_logger().info('Setting up setDigitPosn call')
        speed       = eci.Percentage()
        speed.value = int(request.speed.value)
        thumb = int(request.thumb)
        index = int(request.index)
        middle = int(request.middle)
        ring = int(request.ring)
        little = int(request.little)
        rotate = int(request.rotate)
        self.get_logger().info('setDigitPosn call has been setup')
        self.get_logger().info('Calling eci.setDigitPosn synchronously')
        msg: eci.DigitPosnSetMsg = self.eci.setDigitPosn(
            speed  = speed,
            thumb  = thumb,
            index  = index,
            middle = middle,
            ring   = ring,
            little = little,
            rotate = rotate,
        )
        self.get_logger().info('Called eci.setDigitPosn synchronously')
        self.get_logger().info('Building setDigitPosn response')
        response.result                      = covvi_interfaces.msg.DigitPosnSetMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.speed                = int(msg.speed)
        response.result.rotate               = bool(msg.rotate)
        response.result.little               = bool(msg.little)
        response.result.ring                 = bool(msg.ring)
        response.result.middle               = bool(msg.middle)
        response.result.index                = bool(msg.index)
        response.result.thumb                = bool(msg.thumb)
        response.result.thumb_pos            = int(msg.thumb_pos)
        response.result.index_pos            = int(msg.index_pos)
        response.result.middle_pos           = int(msg.middle_pos)
        response.result.ring_pos             = int(msg.ring_pos)
        response.result.little_pos           = int(msg.little_pos)
        response.result.rotate_pos           = int(msg.rotate_pos)
        self.get_logger().info('Built setDigitPosn response')
        self.get_logger().info('Sending setDigitPosn response to client')
        return response

    def serviceCallbackSetDigitPosnStop(self,
            request:  covvi_interfaces.srv.SetDigitPosnStop.Request,
            response: covvi_interfaces.srv.SetDigitPosnStop.Response,
        )          -> covvi_interfaces.srv.SetDigitPosnStop.Response:
        """Set the digit movement to stop"""
        self.get_logger().info('Calling eci.setDigitPosnStop synchronously')
        msg: eci.DigitPosnSetMsg = self.eci.setDigitPosnStop()
        self.get_logger().info('Called eci.setDigitPosnStop synchronously')
        self.get_logger().info('Building setDigitPosnStop response')
        response.result                      = covvi_interfaces.msg.DigitPosnSetMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.speed                = int(msg.speed)
        response.result.rotate               = bool(msg.rotate)
        response.result.little               = bool(msg.little)
        response.result.ring                 = bool(msg.ring)
        response.result.middle               = bool(msg.middle)
        response.result.index                = bool(msg.index)
        response.result.thumb                = bool(msg.thumb)
        response.result.thumb_pos            = int(msg.thumb_pos)
        response.result.index_pos            = int(msg.index_pos)
        response.result.middle_pos           = int(msg.middle_pos)
        response.result.ring_pos             = int(msg.ring_pos)
        response.result.little_pos           = int(msg.little_pos)
        response.result.rotate_pos           = int(msg.rotate_pos)
        self.get_logger().info('Built setDigitPosnStop response')
        self.get_logger().info('Sending setDigitPosnStop response to client')
        return response

    def serviceCallbackSetDirectControlClose(self,
            request:  covvi_interfaces.srv.SetDirectControlClose.Request,
            response: covvi_interfaces.srv.SetDirectControlClose.Response,
        )          -> covvi_interfaces.srv.SetDirectControlClose.Response:
        """"""
        self.get_logger().info('Setting up setDirectControlClose call')
        speed       = eci.Percentage()
        speed.value = int(request.speed.value)
        self.get_logger().info('setDirectControlClose call has been setup')
        self.get_logger().info('Calling eci.setDirectControlClose synchronously')
        msg: eci.DirectControlMsg = self.eci.setDirectControlClose(
            speed = speed,
        )
        self.get_logger().info('Called eci.setDirectControlClose synchronously')
        self.get_logger().info('Building setDirectControlClose response')
        response.result                      = covvi_interfaces.msg.DirectControlMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.command              = int(msg.command)
        response.result.speed                = covvi_interfaces.msg.Percentage()
        response.result.speed.value          = int(msg.speed.value)
        self.get_logger().info('Built setDirectControlClose response')
        self.get_logger().info('Sending setDirectControlClose response to client')
        return response

    def serviceCallbackSetDirectControlOpen(self,
            request:  covvi_interfaces.srv.SetDirectControlOpen.Request,
            response: covvi_interfaces.srv.SetDirectControlOpen.Response,
        )          -> covvi_interfaces.srv.SetDirectControlOpen.Response:
        """"""
        self.get_logger().info('Setting up setDirectControlOpen call')
        speed       = eci.Percentage()
        speed.value = int(request.speed.value)
        self.get_logger().info('setDirectControlOpen call has been setup')
        self.get_logger().info('Calling eci.setDirectControlOpen synchronously')
        msg: eci.DirectControlMsg = self.eci.setDirectControlOpen(
            speed = speed,
        )
        self.get_logger().info('Called eci.setDirectControlOpen synchronously')
        self.get_logger().info('Building setDirectControlOpen response')
        response.result                      = covvi_interfaces.msg.DirectControlMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.command              = int(msg.command)
        response.result.speed                = covvi_interfaces.msg.Percentage()
        response.result.speed.value          = int(msg.speed.value)
        self.get_logger().info('Built setDirectControlOpen response')
        self.get_logger().info('Sending setDirectControlOpen response to client')
        return response

    def serviceCallbackSetDirectControlStop(self,
            request:  covvi_interfaces.srv.SetDirectControlStop.Request,
            response: covvi_interfaces.srv.SetDirectControlStop.Response,
        )          -> covvi_interfaces.srv.SetDirectControlStop.Response:
        """"""
        self.get_logger().info('Calling eci.setDirectControlStop synchronously')
        msg: eci.DirectControlMsg = self.eci.setDirectControlStop()
        self.get_logger().info('Called eci.setDirectControlStop synchronously')
        self.get_logger().info('Building setDirectControlStop response')
        response.result                      = covvi_interfaces.msg.DirectControlMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.command              = int(msg.command)
        response.result.speed                = covvi_interfaces.msg.Percentage()
        response.result.speed.value          = int(msg.speed.value)
        self.get_logger().info('Built setDirectControlStop response')
        self.get_logger().info('Sending setDirectControlStop response to client')
        return response

    def serviceCallbackSetHandPowerOff(self,
            request:  covvi_interfaces.srv.SetHandPowerOff.Request,
            response: covvi_interfaces.srv.SetHandPowerOff.Response,
        )          -> covvi_interfaces.srv.SetHandPowerOff.Response:
        """Power off the hand"""
        self.get_logger().info('Calling eci.setHandPowerOff synchronously')
        msg: eci.HandPowerMsg = self.eci.setHandPowerOff()
        self.get_logger().info('Called eci.setHandPowerOff synchronously')
        self.get_logger().info('Building setHandPowerOff response')
        response.result                      = covvi_interfaces.msg.HandPowerMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.enable               = bool(msg.enable)
        self.get_logger().info('Built setHandPowerOff response')
        self.get_logger().info('Sending setHandPowerOff response to client')
        return response

    def serviceCallbackSetHandPowerOn(self,
            request:  covvi_interfaces.srv.SetHandPowerOn.Request,
            response: covvi_interfaces.srv.SetHandPowerOn.Response,
        )          -> covvi_interfaces.srv.SetHandPowerOn.Response:
        """Power on the hand"""
        self.get_logger().info('Calling eci.setHandPowerOn synchronously')
        msg: eci.HandPowerMsg = self.eci.setHandPowerOn()
        self.get_logger().info('Called eci.setHandPowerOn synchronously')
        self.get_logger().info('Building setHandPowerOn response')
        response.result                      = covvi_interfaces.msg.HandPowerMsg()
        response.result.uid                  = str(msg.uid)
        response.result.dev_id               = covvi_interfaces.msg.NetDevice()
        response.result.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        response.result.cmd_type             = covvi_interfaces.msg.Command()
        response.result.cmd_type.value       = covvi_interfaces.msg.CommandString()
        response.result.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        response.result.msg_id               = covvi_interfaces.msg.MessageID()
        response.result.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        response.result.data_len             = int(msg.data_len)
        response.result.enable               = bool(msg.enable)
        self.get_logger().info('Built setHandPowerOn response')
        self.get_logger().info('Sending setHandPowerOn response to client')
        return response

    def serviceCallbackSetRealtimeCfg(self,
            request:  covvi_interfaces.srv.SetRealtimeCfg.Request,
            response: covvi_interfaces.srv.SetRealtimeCfg.Response,
        )          -> covvi_interfaces.srv.SetRealtimeCfg.Response:
        """"""
        self.get_logger().info('Setting up setRealtimeCfg call')
        digit_status = bool(request.digit_status)
        digit_posn = bool(request.digit_posn)
        current_grip = bool(request.current_grip)
        electrode_value = bool(request.electrode_value)
        input_status = bool(request.input_status)
        motor_current = bool(request.motor_current)
        digit_touch = bool(request.digit_touch)
        digit_error = bool(request.digit_error)
        environmental = bool(request.environmental)
        orientation = bool(request.orientation)
        motor_limits = bool(request.motor_limits)
        self.get_logger().info('setRealtimeCfg call has been setup')
        self.get_logger().info('Calling eci.setRealtimeCfg synchronously')
        msg: eci.RealtimeCfg = self.eci.setRealtimeCfg(
            digit_status    = digit_status,
            digit_posn      = digit_posn,
            current_grip    = current_grip,
            electrode_value = electrode_value,
            input_status    = input_status,
            motor_current   = motor_current,
            digit_touch     = digit_touch,
            digit_error     = digit_error,
            environmental   = environmental,
            orientation     = orientation,
            motor_limits    = motor_limits,
        )
        self.get_logger().info('Called eci.setRealtimeCfg synchronously')
        self.get_logger().info('Building setRealtimeCfg response')
        response.result                 = covvi_interfaces.msg.RealtimeCfg()
        response.result.digit_status    = bool(msg.digit_status)
        response.result.digit_posn      = bool(msg.digit_posn)
        response.result.current_grip    = bool(msg.current_grip)
        response.result.electrode_value = bool(msg.electrode_value)
        response.result.input_status    = bool(msg.input_status)
        response.result.motor_current   = bool(msg.motor_current)
        response.result.digit_touch     = bool(msg.digit_touch)
        response.result.digit_error     = bool(msg.digit_error)
        response.result.environmental   = bool(msg.environmental)
        response.result.orientation     = bool(msg.orientation)
        response.result.motor_limits    = bool(msg.motor_limits)
        self.get_logger().info('Built setRealtimeCfg response')
        self.get_logger().info('Sending setRealtimeCfg response to client')
        return response

    def publisherCallbackDigitStatusAllMsg(self, msg: eci.DigitStatusAllMsg) -> None:
        self.get_logger().info('Building DigitStatusAllMsg')
        ros2_msg                      = covvi_interfaces.msg.DigitStatusAllMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.thumb_fault          = bool(msg.thumb_fault)
        ros2_msg.thumb_gripping       = bool(msg.thumb_gripping)
        ros2_msg.thumb_at_open        = bool(msg.thumb_at_open)
        ros2_msg.thumb_at_posn        = bool(msg.thumb_at_posn)
        ros2_msg.thumb_touch          = bool(msg.thumb_touch)
        ros2_msg.thumb_stall          = bool(msg.thumb_stall)
        ros2_msg.thumb_stopped        = bool(msg.thumb_stopped)
        ros2_msg.thumb_active         = bool(msg.thumb_active)
        ros2_msg.index_fault          = bool(msg.index_fault)
        ros2_msg.index_gripping       = bool(msg.index_gripping)
        ros2_msg.index_at_open        = bool(msg.index_at_open)
        ros2_msg.index_at_posn        = bool(msg.index_at_posn)
        ros2_msg.index_touch          = bool(msg.index_touch)
        ros2_msg.index_stall          = bool(msg.index_stall)
        ros2_msg.index_stopped        = bool(msg.index_stopped)
        ros2_msg.index_active         = bool(msg.index_active)
        ros2_msg.middle_fault         = bool(msg.middle_fault)
        ros2_msg.middle_gripping      = bool(msg.middle_gripping)
        ros2_msg.middle_at_open       = bool(msg.middle_at_open)
        ros2_msg.middle_at_posn       = bool(msg.middle_at_posn)
        ros2_msg.middle_touch         = bool(msg.middle_touch)
        ros2_msg.middle_stall         = bool(msg.middle_stall)
        ros2_msg.middle_stopped       = bool(msg.middle_stopped)
        ros2_msg.middle_active        = bool(msg.middle_active)
        ros2_msg.ring_fault           = bool(msg.ring_fault)
        ros2_msg.ring_gripping        = bool(msg.ring_gripping)
        ros2_msg.ring_at_open         = bool(msg.ring_at_open)
        ros2_msg.ring_at_posn         = bool(msg.ring_at_posn)
        ros2_msg.ring_touch           = bool(msg.ring_touch)
        ros2_msg.ring_stall           = bool(msg.ring_stall)
        ros2_msg.ring_stopped         = bool(msg.ring_stopped)
        ros2_msg.ring_active          = bool(msg.ring_active)
        ros2_msg.little_fault         = bool(msg.little_fault)
        ros2_msg.little_gripping      = bool(msg.little_gripping)
        ros2_msg.little_at_open       = bool(msg.little_at_open)
        ros2_msg.little_at_posn       = bool(msg.little_at_posn)
        ros2_msg.little_touch         = bool(msg.little_touch)
        ros2_msg.little_stall         = bool(msg.little_stall)
        ros2_msg.little_stopped       = bool(msg.little_stopped)
        ros2_msg.little_active        = bool(msg.little_active)
        ros2_msg.rotate_fault         = bool(msg.rotate_fault)
        ros2_msg.rotate_gripping      = bool(msg.rotate_gripping)
        ros2_msg.rotate_at_open       = bool(msg.rotate_at_open)
        ros2_msg.rotate_at_posn       = bool(msg.rotate_at_posn)
        ros2_msg.rotate_touch         = bool(msg.rotate_touch)
        ros2_msg.rotate_stall         = bool(msg.rotate_stall)
        ros2_msg.rotate_stopped       = bool(msg.rotate_stopped)
        ros2_msg.rotate_active        = bool(msg.rotate_active)
        self.get_logger().info('Publishing DigitStatusAllMsg')
        self.publisher_DigitStatusAllMsg.publish(ros2_msg)
        self.get_logger().info('Published DigitStatusAllMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackDigitPosnAllMsg(self, msg: eci.DigitPosnAllMsg) -> None:
        self.get_logger().info('Building DigitPosnAllMsg')
        ros2_msg                      = covvi_interfaces.msg.DigitPosnAllMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.thumb_pos            = int(msg.thumb_pos)
        ros2_msg.index_pos            = int(msg.index_pos)
        ros2_msg.middle_pos           = int(msg.middle_pos)
        ros2_msg.ring_pos             = int(msg.ring_pos)
        ros2_msg.little_pos           = int(msg.little_pos)
        ros2_msg.rotate_pos           = int(msg.rotate_pos)
        self.get_logger().info('Publishing DigitPosnAllMsg')
        self.publisher_DigitPosnAllMsg.publish(ros2_msg)
        self.get_logger().info('Published DigitPosnAllMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackCurrentGripMsg(self, msg: eci.CurrentGripMsg) -> None:
        self.get_logger().info('Building CurrentGripMsg')
        ros2_msg                      = covvi_interfaces.msg.CurrentGripMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.grip_id              = covvi_interfaces.msg.CurrentGripID()
        ros2_msg.grip_id.value        = eci.CurrentGripID(msg.grip_id.value).value
        ros2_msg.table                = covvi_interfaces.msg.Table()
        ros2_msg.table.value          = eci.Table(msg.table.value).value
        ros2_msg.table_index          = covvi_interfaces.msg.TableIndex()
        ros2_msg.table_index.value    = eci.TableIndex(msg.table_index.value).value
        self.get_logger().info('Publishing CurrentGripMsg')
        self.publisher_CurrentGripMsg.publish(ros2_msg)
        self.get_logger().info('Published CurrentGripMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackElectrodeValueMsg(self, msg: eci.ElectrodeValueMsg) -> None:
        self.get_logger().info('Building ElectrodeValueMsg')
        ros2_msg                      = covvi_interfaces.msg.ElectrodeValueMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.voltage              = eci.Int16(msg.voltage)
        self.get_logger().info('Publishing ElectrodeValueMsg')
        self.publisher_ElectrodeValueMsg.publish(ros2_msg)
        self.get_logger().info('Published ElectrodeValueMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackInputStatusMsg(self, msg: eci.InputStatusMsg) -> None:
        self.get_logger().info('Building InputStatusMsg')
        ros2_msg                      = covvi_interfaces.msg.InputStatusMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.supinate             = bool(msg.supinate)
        ros2_msg.pronate              = bool(msg.pronate)
        ros2_msg.back_button          = bool(msg.back_button)
        ros2_msg.little_tip           = bool(msg.little_tip)
        ros2_msg.ring_tip             = bool(msg.ring_tip)
        ros2_msg.middle_tip           = bool(msg.middle_tip)
        ros2_msg.index_tip            = bool(msg.index_tip)
        ros2_msg.thumb_tip            = bool(msg.thumb_tip)
        self.get_logger().info('Publishing InputStatusMsg')
        self.publisher_InputStatusMsg.publish(ros2_msg)
        self.get_logger().info('Published InputStatusMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackMotorCurrentAllMsg(self, msg: eci.MotorCurrentAllMsg) -> None:
        self.get_logger().info('Building MotorCurrentAllMsg')
        ros2_msg                      = covvi_interfaces.msg.MotorCurrentAllMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.thumb_current        = int(msg.thumb_current)
        ros2_msg.index_current        = int(msg.index_current)
        ros2_msg.middle_current       = int(msg.middle_current)
        ros2_msg.ring_current         = int(msg.ring_current)
        ros2_msg.little_current       = int(msg.little_current)
        self.get_logger().info('Publishing MotorCurrentAllMsg')
        self.publisher_MotorCurrentAllMsg.publish(ros2_msg)
        self.get_logger().info('Published MotorCurrentAllMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackDigitTouchAllMsg(self, msg: eci.DigitTouchAllMsg) -> None:
        self.get_logger().info('Building DigitTouchAllMsg')
        ros2_msg                      = covvi_interfaces.msg.DigitTouchAllMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.thumb_touch          = int(msg.thumb_touch)
        ros2_msg.index_touch          = int(msg.index_touch)
        ros2_msg.middle_touch         = int(msg.middle_touch)
        ros2_msg.ring_touch           = int(msg.ring_touch)
        ros2_msg.little_touch         = int(msg.little_touch)
        self.get_logger().info('Publishing DigitTouchAllMsg')
        self.publisher_DigitTouchAllMsg.publish(ros2_msg)
        self.get_logger().info('Published DigitTouchAllMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackEnvironmentalMsg(self, msg: eci.EnvironmentalMsg) -> None:
        self.get_logger().info('Building EnvironmentalMsg')
        ros2_msg                      = covvi_interfaces.msg.EnvironmentalMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.temperature          = int(msg.temperature)
        ros2_msg.humidity             = int(msg.humidity)
        ros2_msg.battery_voltage      = eci.Int16(msg.battery_voltage)
        self.get_logger().info('Publishing EnvironmentalMsg')
        self.publisher_EnvironmentalMsg.publish(ros2_msg)
        self.get_logger().info('Published EnvironmentalMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackSystemStatusMsg(self, msg: eci.SystemStatusMsg) -> None:
        self.get_logger().info('Building SystemStatusMsg')
        ros2_msg                      = covvi_interfaces.msg.SystemStatusMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.bluetooth_fault      = bool(msg.bluetooth_fault)
        ros2_msg.spi_error            = bool(msg.spi_error)
        ros2_msg.gateway_error        = bool(msg.gateway_error)
        ros2_msg.humidity_limit       = bool(msg.humidity_limit)
        ros2_msg.temperature_limit    = bool(msg.temperature_limit)
        ros2_msg.bluetooth_status     = int(msg.bluetooth_status)
        ros2_msg.change_notifications = int(msg.change_notifications)
        self.get_logger().info('Publishing SystemStatusMsg')
        self.publisher_SystemStatusMsg.publish(ros2_msg)
        self.get_logger().info('Published SystemStatusMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackOrientationMsg(self, msg: eci.OrientationMsg) -> None:
        self.get_logger().info('Building OrientationMsg')
        ros2_msg                      = covvi_interfaces.msg.OrientationMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.x                    = eci.Int16(msg.x)
        ros2_msg.y                    = eci.Int16(msg.y)
        ros2_msg.z                    = eci.Int16(msg.z)
        self.get_logger().info('Publishing OrientationMsg')
        self.publisher_OrientationMsg.publish(ros2_msg)
        self.get_logger().info('Published OrientationMsg')
        self.get_logger().info(str(ros2_msg))

    def publisherCallbackMotorLimitsMsg(self, msg: eci.MotorLimitsMsg) -> None:
        self.get_logger().info('Building MotorLimitsMsg')
        ros2_msg                      = covvi_interfaces.msg.MotorLimitsMsg()
        ros2_msg.uid                  = str(msg.uid)
        ros2_msg.dev_id               = covvi_interfaces.msg.NetDevice()
        ros2_msg.dev_id.value         = eci.NetDevice(msg.dev_id.value).value
        ros2_msg.cmd_type             = covvi_interfaces.msg.Command()
        ros2_msg.cmd_type.value       = covvi_interfaces.msg.CommandString()
        ros2_msg.cmd_type.value.value = eci.CommandString(msg.cmd_type.value.value).value
        ros2_msg.msg_id               = covvi_interfaces.msg.MessageID()
        ros2_msg.msg_id.value         = eci.MessageID(msg.msg_id.value).value
        ros2_msg.data_len             = int(msg.data_len)
        ros2_msg.hand                 = bool(msg.hand)
        ros2_msg.eci                  = bool(msg.eci)
        ros2_msg.ltl                  = bool(msg.ltl)
        ros2_msg.rng                  = bool(msg.rng)
        ros2_msg.mid                  = bool(msg.mid)
        ros2_msg.idx                  = bool(msg.idx)
        ros2_msg.thb                  = bool(msg.thb)
        ros2_msg.thumb_derate_value   = int(msg.thumb_derate_value)
        ros2_msg.index_derate_value   = int(msg.index_derate_value)
        ros2_msg.middle_derate_value  = int(msg.middle_derate_value)
        ros2_msg.ring_derate_value    = int(msg.ring_derate_value)
        ros2_msg.little_derate_value  = int(msg.little_derate_value)
        self.get_logger().info('Publishing MotorLimitsMsg')
        self.publisher_MotorLimitsMsg.publish(ros2_msg)
        self.get_logger().info('Published MotorLimitsMsg')
        self.get_logger().info(str(ros2_msg))

    def start_eci(self) -> None:
        self.get_logger().info(f'Connecting to the ECI {self.host}')
        self.eci.start()
        self.get_logger().info(f'Connected to the ECI {self.host}')
    
    def stop_eci(self) -> None:
        self.get_logger().info(f'Disconnecting to the ECI {self.host}')
        self.eci.stop()
        self.get_logger().info(f'Disconnected to the ECI {self.host}')


def main(args: Iterable[Any] | None = None) -> None:
    _, host, *_ = sys.argv
    try:
        with rclpy.init(args=args):
            covvi_server_node = CovviServerNode(host=host)
            covvi_server_node.start_eci()
            covvi_server_node.get_logger().info('Turning power on to the hand...')
            covvi_server_node.eci.setHandPowerOn()
            covvi_server_node.get_logger().info('Setting realtime config...')
            covvi_server_node.eci.setRealtimeCfg(environmental=True, orientation=True, digit_touch=True)
            covvi_server_node.get_logger().info('Spinning...')
            rclpy.spin(covvi_server_node)
            covvi_server_node.stop_eci()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == '__main__':
    main()