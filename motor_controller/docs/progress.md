# Development Progress

## Completed

### micro-ROS

✔ Custom UART transport

✔ Raspberry Pi 5 communication

✔ micro-ROS Agent integration

✔ ROS 2 node creation

## ROS Communication

✔ `/cmd_vel` subscriber

✔ Left wheel target publisher

✔ Right wheel target publisher

✔ Verified Best Effort QoS

## Motion Control

✔ Differential drive equations

✔ Wheel velocity targets (mm/s)

✔ Linear motion

✔ Reverse motion

✔ Steering calculations

## Safety

✔ 500 ms command timeout

✔ Automatic stop after communication loss

✔ Zero motor command on timeout

## Motor Driver

✔ Motor output module

✔ PWM generation

✔ Direction control

✔ Cytron MDD10A interface

✔ GPIO verification

✔ Hardware validation

## Hardware Tests

Verified:

* DIR outputs change correctly with forward/reverse commands.
* PWM duty cycle changes proportionally with requested speed.
* Motors respond correctly to ROS 2 `/cmd_vel`.
* Timeout returns motor outputs to zero after publisher stops.

## Git Milestones

* Initial firmware
* micro-ROS custom UART transport
* Verified `/cmd_vel` subscriber
* Differential drive math
* Wheel target publishers
* Motor driver operational

Current tag:

`wheel-targets-working`

## Next Objectives

1. Encoder interface
2. Wheel velocity measurement
3. PID speed controller
4. Wheel odometry
5. Nav2 integration
6. Autonomous navigation
