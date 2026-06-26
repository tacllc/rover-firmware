# Rover Firmware

ESP32 firmware for the ROS 2 autonomous rover.

## Hardware

* ESP32 DevKit V1
* Raspberry Pi 5 running ROS 2 Jazzy
* micro-ROS over USB Serial
* Cytron MDD10A dual motor driver
* Two CQRobot 37D geared DC motors with quadrature encoders

## Current Features

* micro-ROS custom UART transport
* ROS 2 `/cmd_vel` subscriber
* Differential drive kinematics
* Safety timeout (500 ms)
* Independent left/right wheel target publishers
* Motor output abstraction module
* PWM motor control
* Direction control
* Hardware verified on MDD10A

## Verified Operation

The complete control chain has been validated:

ROS 2 `/cmd_vel`
→ micro-ROS Agent
→ ESP32
→ Differential Drive Math
→ Wheel Target Publishers
→ Motor Output Module
→ Cytron MDD10A
→ Motors

Motor direction, PWM output, and safety timeout have all been confirmed on hardware.

## Next Milestones

* Encoder driver
* Closed-loop PID speed control
* Wheel velocity publishers
* Odometry
* Nav2 integration
