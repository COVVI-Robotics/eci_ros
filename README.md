# COVVI ROS2 Interface

### Connecting/Disconnecting the interface

Replace:
- ```$ECI_HOST``` with the IP address of the COVVI hand.
- ```$NAMESPACE``` with the namespace of your ROS2 application.
- ```$NAME```  with the name of the server node (one per ECI).


```bash
ros2 run covvi_hand_driver server $ECI_HOST --ros-args --remap __ns:=/$NAMESPACE --remap __name:=$NAME
```

As an example:

```bash
ros2 run covvi_hand_driver server 192.168.1.123 --ros-args --remap __ns:=/test --remap __name:=server_1
```


## Hand Power

### setHandPowerOn() - Turn the power on to the hand


```bash
ros2 service call /test/server_1/SetHandPowerOn covvi_interfaces/srv/SetHandPowerOn
```

### setHandPowerOff() - Turn the power off to the hand


```bash
ros2 service call /test/server_1/SetHandPowerOff covvi_interfaces/srv/SetHandPowerOff
```

Subsequent code snippets assume that ```/.../.../SetHandPowerOn``` has already been called and that the power to the hand is on.

## Discovery and device messages

### Hello

#### getHello() - Get a simple 'hello' response from the ECI


```bash
ros2 service call /test/server_1/GetHello covvi_interfaces/srv/GetHello
```

### Firmware

#### getFirmware_PIC() - Get the PIC Firmware version


```bash
ros2 service call /test/server_1/GetFirmwarePICECI covvi_interfaces/srv/GetFirmwarePICECI
# Requires power to the hand (blue LED on)
ros2 service call /test/server_1/GetFirmwarePICHAND covvi_interfaces/srv/GetFirmwarePICHAND
```

### DeviceIdentity

#### getDeviceIdentity() - Get device identity parameters


```bash
ros2 service call /test/server_1/GetDeviceIdentity covvi_interfaces/srv/GetDeviceIdentity
```

### DeviceProduct

#### getDeviceProduct() - Get product


```bash
ros2 service call /test/server_1/GetDeviceProduct covvi_interfaces/srv/GetDeviceProduct
```

## Real-time messages

### RealtimeCfg

#### setRealtimeCfg() - Set real-time update configuration

##### Turn all realtime packets on


```bash
ros2 service call /test/server_1/EnableAllRealtimeCfg covvi_interfaces/srv/EnableAllRealtimeCfg
```

##### Turn all realtime packets off


```bash
ros2 service call /test/server_1/DisableAllRealtimeCfg covvi_interfaces/srv/DisableAllRealtimeCfg
```


```bash
ros2 service call /test/server_1/SetRealtimeCfg covvi_interfaces/srv/SetRealtimeCfg \
"{
    digit_status:  False, digit_posn:    False, current_grip: False, electrode_value: False,
    input_status:  False, motor_current: False, digit_touch:  False, digit_error:     False,
    environmental: False, orientation:   False, motor_limits: False,
}"
```


```bash
ros2 service call /test/server_1/SetRealtimeCfg covvi_interfaces/srv/SetRealtimeCfg
```

#### resetRealtimeCfg() - Reset the realtime callbacks (and stop streaming realtime messages)


```bash
ros2 service call /test/server_1/ResetRealtimeCfg covvi_interfaces/srv/ResetRealtimeCfg
```

### DigitStatus

#### getDigitStatus_all() - Get all digit status flags


```bash
ros2 service call /test/server_1/GetDigitStatusAll covvi_interfaces/srv/GetDigitStatusAll
```

#### getDigitStatus() - Get all digit status flags individually


```bash
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 0}}" # THUMB
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 1}}" # INDEX
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 2}}" # MIDDLE
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 3}}" # RING
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 4}}" # LITTLE
ros2 service call /test/server_1/GetDigitStatus covvi_interfaces/srv/GetDigitStatus "{digit: {value: 5}}" # ROTATE
```

### DigitPosn

#### getDigitPosn_all() - Get all digit positions


```bash
ros2 service call /test/server_1/GetDigitPosnAll covvi_interfaces/srv/GetDigitPosnAll
```

#### getDigitPosn() - Get all digit positions individually


```bash
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 0}}" # THUMB
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 1}}" # INDEX
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 2}}" # MIDDLE
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 3}}" # RING
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 4}}" # LITTLE
ros2 service call /test/server_1/GetDigitPosn covvi_interfaces/srv/GetDigitPosn "{digit: {value: 5}}" # ROTATE
```

#### setDigitPosn() - Set all digit positions individually - Close the hand fully


```bash
ros2 service call /test/server_1/SetDigitPosn covvi_interfaces/srv/SetDigitPosn \
"{
    speed: {value: 50},
    thumb: 200, index: 200, middle: 200, ring: 200, little: 200, rotate: 200,
}"
```

#### setDigitPosn() - Set all digit positions individually - Open the hand fully


```bash
ros2 service call /test/server_1/SetDigitPosn covvi_interfaces/srv/SetDigitPosn \
"{
    speed: {value: 50},
    thumb: 40, index: 40, middle: 40, ring: 40, little: 40, rotate: 40,
}"
```

#### setDigitPosn() - Set all digit positions individually - Perform a thumbs up


```bash
ros2 service call /test/server_1/SetDigitPosn covvi_interfaces/srv/SetDigitPosn \
"{
    speed: {value: 100},
    thumb: 0, index: 0, middle: 0, ring: 0, little: 0, rotate: 0,
}"
sleep 1
ros2 service call /test/server_1/SetDigitPosn covvi_interfaces/srv/SetDigitPosn \
"{
    speed: {value: 100},
    thumb: 0, index: 200, middle: 200, ring: 200, little: 200, rotate: 0,
}"
```

### CurrentGrip

#### getCurrentGrip() - Get the current grip


```bash
ros2 service call /test/server_1/GetCurrentGrip covvi_interfaces/srv/GetCurrentGrip
```

#### setCurrentGrip(grip_id) - Set the current grip


```bash
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 1}}"
```

#### setCurrentGrip(grip_id) - Set the current grip to <current_grip_id>


```bash
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  1}}" # TRIPOD
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  2}}" # POWER
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  3}}" # TRIGGER
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  4}}" # PREC_OPEN
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  5}}" # PREC_CLOSED
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  6}}" # KEY
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  7}}" # FINGER
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  8}}" # CYLINDER
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value:  9}}" # COLUMN
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 10}}" # RELAXED
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 11}}" # GLOVE
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 12}}" # TAP
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 13}}" # GRAB
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 14}}" # TRIPOD_OPEN
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 15}}" # GN0
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 16}}" # GN1
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 17}}" # GN2
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 18}}" # GN3
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 19}}" # GN4
ros2 service call /test/server_1/SetCurrentGrip covvi_interfaces/srv/SetCurrentGrip "{grip_id: {value: 20}}" # GN5
```


```bash
ros2 service call /test/server_1/SetDirectControlClose covvi_interfaces/srv/SetDirectControlClose "{speed: {value: 100}}"
```


```bash
ros2 service call /test/server_1/SetDirectControlOpen covvi_interfaces/srv/SetDirectControlOpen "{speed: {value: 100}}"
```


```bash
ros2 service call /test/server_1/SetDirectControlStop covvi_interfaces/srv/SetDirectControlStop
```

### DirectControl

#### setDirectControlClose() - Close the whole hand


```bash
ros2 service call /test/server_1/SetDirectControlClose covvi_interfaces/srv/SetDirectControlClose "{speed: {value: 100}}"
```

#### setDirectControlOpen() - Open the whole hand


```bash
ros2 service call /test/server_1/SetDirectControlOpen covvi_interfaces/srv/SetDirectControlOpen "{speed: {value: 100}}"
```

### DigitMove

#### setDigitMove() - Command to move each digit individually - Open the whole hand


```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/SetDigitMove covvi_interfaces/srv/SetDigitMove \
        "{digit: {value: $i}, position: 40, speed: {value: 100}, power: {value: 0}, limit: {value: 0}}" ; \
done
```

#### setDigitMove() - Command to move each digit individually - Close the whole hand


```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/SetDigitMove covvi_interfaces/srv/SetDigitMove \
        "{digit: {value: $i}, position: 210, speed: {value: 100}, power: {value: 0}, limit: {value: 0}}" ; \
done
```

#### setDigitMove() - Open the index digit


```bash
ros2 service call /test/server_1/SetDigitMove covvi_interfaces/srv/SetDigitMove \
"{digit: {value: 1}, position: 44, speed: {value: 100}, power: {value: 20}, limit: {value: 0}}"
```

#### setDigitMove() - Close the index digit


```bash
ros2 service call /test/server_1/SetDigitMove covvi_interfaces/srv/SetDigitMove \
"{digit: {value: 1}, position: 210, speed: {value: 100}, power: {value: 20}, limit: {value: 0}}"
```

#### setDigitMove() - Command to move each digit individually - Set the digits to random positions


```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/SetDigitMove covvi_interfaces/srv/SetDigitMove \
        "{digit: {value: $i}, position: $((40 + $RANDOM % (200 - 40))), speed: {value: 100}, power: {value: 0}, limit: {value: 0}}" ; \
done
```

### MotorCurrent

#### getMotorCurrent_all() - Get the motor current of all Digits


```bash
ros2 service call /test/server_1/GetMotorCurrentAll covvi_interfaces/srv/GetMotorCurrentAll
```

#### getMotorCurrent() - Get the motor current of all Digits individually


```bash
for i in 0 1 2 3 4 ; do \
    ros2 service call /test/server_1/GetMotorCurrent covvi_interfaces/srv/GetMotorCurrent "{digit: {value: $i}}" ; \
done
```

### DigitError

#### getDigitError() - Get the digit error flags of all digits individually

```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/GetDigitError covvi_interfaces/srv/GetDigitError "{digit: {value: $i}}" ; \
done
```

## Digit configuration messages

### DigitConfig

#### getDigitConfig() - Get the limits of each digit individually


```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/GetDigitConfig covvi_interfaces/srv/GetDigitConfig "{digit: {value: $i}}" ; \
done
```

### PinchConfig

#### getPinchConfig() - Get the pinch points of each digit individually


```bash
ros2 service call /test/server_1/GetPinchConfig covvi_interfaces/srv/GetPinchConfig
```

## Grip configuration messages

### GripName

#### getGripName


```bash
for i in 0 1 2 3 4 5 ; do \
    ros2 service call /test/server_1/GetGripName covvi_interfaces/srv/GetGripName "{grip_name_index: {value: $i}}" ; \
done
```

## System and status messages

### Environmental

#### getEnvironmental() - Get the temperature, humidity, battery voltage values of the hand


```bash
ros2 service call /test/server_1/GetEnvironmental covvi_interfaces/srv/GetEnvironmental
```

### SystemStatus

#### getSystemStatus() - Get the system status


```bash
ros2 service call /test/server_1/GetSystemStatus covvi_interfaces/srv/GetSystemStatus
```

### Orientation

#### getOrientation() - Get the orientation of the hand


```bash
ros2 service call /test/server_1/GetOrientation covvi_interfaces/srv/GetOrientation
```

## Firmware update messages

### SendUserGrip

#### sendUserGrip(grip_index, grip_path) - Send a User Grip


```bash
ros2 service call /test/server_1/SendUserGrip covvi_interfaces/srv/SendUserGrip "{grip_name_index: {value: 0}, user_grip: {value: 0}}" # GN0, FIST
```

#### resetUserGrips() - Reset all the User Grips


```bash
ros2 service call /test/server_1/ResetUserGrips covvi_interfaces/srv/ResetUserGrips
```

### RemoveUserGrip

#### removeUserGrip(grip_index) - Remove a User Grip


```bash
ros2 service call /test/server_1/RemoveUserGrip covvi_interfaces/srv/RemoveUserGrip "{grip_name_index: {value: 0}}" # GN0
```
