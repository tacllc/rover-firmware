Last updated 2026-06-25

Rover Bringup
Manual Bringup Sequence
1. Start Raspberry Pi

Boot the Raspberry Pi 5.

2. Start micro-ROS Agent

On the Pi:

cd ~/scripts
./launch_ros_agent.sh

The script should start something equivalent to:

source /opt/ros/jazzy/setup.bash
source ~/rover_ws/install/setup.bash
ros2 run micro_ros_agent micro_ros_agent serial -D /dev/ttyUSB0 -b 115200 -v4

Leave this terminal running.

3. Reset ESP32

Press the ESP32 reset button after the agent is running.

4. Verify Topics

In another Pi terminal:

source /opt/ros/jazzy/setup.bash
source ~/rover_ws/install/setup.bash
ros2 topic list

Expected topics include:

/cmd_vel
/left_wheel_target_mmps
/right_wheel_target_mmps
/parameter_events
/rosout
5. Publish Test Command
ros2 topic pub -r 1 --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 1.0}, angular: {z: 0.5}}"
6. Verify Wheel Targets

In separate terminals:

ros2 topic echo /left_wheel_target_mmps
ros2 topic echo /right_wheel_target_mmps

Expected:

left  ≈ 925
right ≈ 1075
7. Verify Timeout

Stop the /cmd_vel publisher.

Expected:

left  -> 0
right -> 0
Troubleshooting
Only /parameter_events and /rosout appear

Likely causes:

micro-ROS Agent is not running
ESP32 was not reset after agent started
wrong serial port
ESP32 firmware failed before node creation

Check:

ls /dev/ttyUSB* /dev/ttyACM*
ps aux | grep micro_ros_agent
/cmd_vel exists but callback does not fire

Use best-effort publishing:

ros2 topic pub -r 1 --qos-reliability best_effort /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"
printf() output is not visible

Expected. UART0 is used by micro-ROS transport.

Use ROS topics for diagnostics, or use a second UART with a CP2102 USB-to-TTL adapter.
EOF

