Last updated 2026-06-25

Motor Control
Current Status

The firmware currently calculates wheel target velocities but does not yet directly drive motors.

Current output:

/left_wheel_target_mmps
/right_wheel_target_mmps

These are diagnostic ROS topics used to verify the command pipeline.

Differential Drive Math

Inputs:

linear.x
angular.z
wheel_base

Calculations:

left_mps  = linear - (angular * WHEEL_BASE_M / 2.0f);
right_mps = linear + (angular * WHEEL_BASE_M / 2.0f);

Converted to millimeters per second:

left_mmps  = (int)(left_mps * 1000.0f);
right_mmps = (int)(right_mps * 1000.0f);
Verified Examples
Forward

Input:

linear.x = 1.25
angular.z = 0.0

Expected:

left  = 1250
right = 1250
Turn in Place

Input:

linear.x = 0.0
angular.z = 1.0
wheel_base = 0.30

Expected:

left  = -150
right = 150
Forward With Rotation

Input:

linear.x = 1.0
angular.z = 0.5
wheel_base = 0.30

Expected:

left  = 925
right = 1075
Next Steps
Add motor driver pin definitions.
Add PWM output.
Map target velocity to motor command.
Add encoder feedback.
Implement velocity PID control.
Publish encoder and motor status topics.
Feed odometry back to ROS 2.
EOF
