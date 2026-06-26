Last updated 2025-06-25

ROS Topics
Current Topics
/cmd_vel

Direction: Pi / ROS 2 → ESP32

Type:

geometry_msgs/msg/Twist

Purpose:

Commands desired rover motion.

Fields currently used:

linear.x   forward/reverse velocity in m/s
angular.z  yaw rotation rate in rad/s

Example:

ros2 topic pub -r 1 --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
/left_wheel_target_mmps

Direction: ESP32 → ROS 2

Type:

std_msgs/msg/Int32

Purpose:

Publishes calculated left wheel target velocity in millimeters per second.

Example expected value:

data: 925
/right_wheel_target_mmps

Direction: ESP32 → ROS 2

Type:

std_msgs/msg/Int32

Purpose:

Publishes calculated right wheel target velocity in millimeters per second.

Example expected value:

data: 1075
Verified Test Case

With:

linear.x = 1.0
angular.z = 0.5
wheel_base = 0.30

Expected:

left  = 925 mm/s
right = 1075 mm/s
Safety Behavior

If /cmd_vel stops publishing, the ESP32 command timeout forces wheel targets back to zero.

This prevents stale velocity commands from continuing to drive the rover.
