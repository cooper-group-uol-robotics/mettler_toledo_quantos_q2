# mettler_toledo_quantos_q2
#### ROS Driver for Mettler Toledo Quantos dispensing system
#### Uses USB for serial communication
##### Written by Jakub Glowacki

## How to Launch
The easiest way to launch the package is with roslaunch:
```
roslaunch ika_rct_digital IKAROS.launch
```

Alternatively, can be launched using rosrun:
```
rosrun ika_rct_digital IKADigitalROS
```

## ROS Topics:
IKA_Commands | For publishing commands to\
IKA_Readings | Periodically (every second) readings from the plate are published to this

## How to send Commands:
Commands are sent using the IKA_Commands topic. Each command has its own command ID which can be sent through the topic as an ika_command. An ika_command is just a simple integer corresponding to a command, as indicated by the list below. Additionally, some of the commands support a second field, ika_param, which allows you to set either the temperature of the heat plate or the speed of the stirrer. This is also an integer. If this is passed to a command that doesn't require it it will be ignored, if it isn't passed to a command that requires it it'll be assumed to be 0. An example command which turns on the hotplate and sets it to 100 degrees is like so:
```
rostopic pub -1 /IKA_Commands ika_rct_digital/IKACommand '{ika_command: 7, ika_param: 100}'

```

## Possible commands:
0 | Turns on the heat bed
1 | Turns off the heat bed
2 | Turns on the stirrer
3 | Turns off the stirrer
4 | Sets the stirrer RPM
5 | Sets the heater temperature in degrees celsius
6 | Turns on stirrer and sets it to specified RPM
7 | Turns on hotplate and sets it to specified temperature
8 | Turns off hotplate and stirrer and resets values to 0
