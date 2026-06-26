# Firmware Architecture

The firmware is organized into functional modules to separate ROS communication from hardware control.

## Current Modules

### motor_controller.c

Responsibilities:

* micro-ROS initialization
* `/cmd_vel` subscriber
* Differential drive calculations
* Safety timeout
* Wheel target publishers
* Calls Motor Output module

### motor_output.c

Responsibilities:

* Configure GPIO
* Configure LEDC PWM
* Control Cytron MDD10A
* Set motor direction
* Set PWM duty cycle

Public API:

```c
void motor_output_init(void);
void motor_output_update(int left_mmps, int right_mmps);
```

## Data Flow

ROS2 `/cmd_vel`

↓

cmd_vel_callback()

↓

Store latest linear.x / angular.z

↓

Timer callback

↓

Safety timeout check

↓

Differential drive math

↓

Left/right wheel target (mm/s)

↓

motor_output_update()

↓

PWM + Direction

↓

Cytron MDD10A

↓

Motors

## Current Safety Features

* 500 ms command timeout
* Automatic motor stop on lost communications
* Motor output abstraction
* Independent left/right wheel targets

## Planned Modules

encoder.c

* Read quadrature encoders
* Publish wheel velocity

pid_controller.c

* Closed-loop speed control

odometry.c

* Wheel odometry
* Robot pose estimation
