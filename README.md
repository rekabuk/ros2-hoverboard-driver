# ros2-hoverboard-driver
I couldn't get Victor's original code to run under ROS2, I noticed he had commented out hoverboad_node->write() in main.c. When I enabled this the serial port got jammed up, so I had to modify the driver to only send a message after receiving one. This worked fine with the firmware from EFeru. Thanks guys for all you help.
I managed to fine wheel_msg here https://github.com/Richard-Haes-Ellis/wheel_msgs
ROS2 driver for controlling hoverboard BLDC motors using UART. Hoverboard's mainboard must use this [firmware](https://github.com/EmanuelFeru/hoverboard-firmware-hack-FOC). ROS2 messages used for topic communication can be found in this [ROS2 package](https://github.com/Richard-Haes-Ellis/wheel_msgs).
