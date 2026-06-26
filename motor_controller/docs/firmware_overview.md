Last updated 2026-06-25

Firmware Overview
Project

Firmware project:

motor_controller

Main source file:

motor_controller/main/motor_controller.c
ESP-IDF

Current ESP-IDF version:

ESP-IDF v5.4.2
micro-ROS

The ESP32 uses micro_ros_espidf_component with custom UART transport.

Important configuration change:

RMW_UXRCE_TRANSPORT=custom

This was required so rmw_uros_set_custom_transport() links correctly.

Transport

UART0 / USB serial is used for micro-ROS transport between the ESP32 and Raspberry Pi.

Because UART0 is used by micro-ROS, printf() and idf.py monitor are not reliable during normal rover operation.

Debugging should use either:

ROS diagnostic topics
A second UART through a CP2102 USB-to-TTL adapter
Current Firmware Behavior

The ESP32 currently:

Starts a micro-ROS client.
Connects to the micro-ROS Agent on the Raspberry Pi.
Creates a ROS node.
Subscribes to /cmd_vel.
Calculates left and right wheel targets.
Publishes wheel target velocities.
Forces wheel targets to zero when /cmd_vel becomes stale.
Known Good Tags
ros2-cmdvel-working
wheel-targets-working

These tags represent known-good restore points.
EOF

