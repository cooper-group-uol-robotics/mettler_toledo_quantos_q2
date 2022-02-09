# mettler_toledo_quantos_q2_driver
#### ROS Driver for Mettler Toledo Quantos dispensing system
#### Uses USB for serial communication
##### Written by Jakub Glowacki and Hatem Fakhruldeen

## How to Launch
The easiest way to launch the package is with roslaunch:
```
roslaunch mettler_toledo_quantos_q2_driver quantos_q2_driver.launch serial_port:=<port_name>
```
This will launch the driver for the quantos connected to the provided serial port. If no serial port argument is provided, the default port '/dev/ttyUSB0' will be used.

Alternatively, can be launched using rosrun:
```
rosrun mettler_toledo_quantos_q2_driver quantos_q2_driver <serial_port>
```
Similarly, if no serial port argument is provided, the serial port '/dev/ttyUSB0' is used.

## ROS Topics:
Todo

## How to send Commands:
Todo

## Possible commands:
Todo
