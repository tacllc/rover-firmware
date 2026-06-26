Last updated 2026-06-25

# Rover System Architecture 

## Overview This rover uses a Raspberry Pi 5 as the main ROS 2 computer and ESP32 microcontrollers for low-level hardware control. 

## Current Architecture ```

Raspberry Pi 5 
	├── Ubuntu Server 24.04 
	├── ROS 2 Jazzy 
	├── micro-ROS Agent 
	└── High-level rover logic 

USB Serial 

ESP32 Motor Controller 
	├── micro-ROS client 
	├── /cmd_vel subscriber 
	├── Differential drive wheel target calculation 
	├── Left wheel target publisher 
	├── Right wheel target publisher 
	└── Command timeout safety


Communication Flow
	/cmd_vel
  		ROS 2 Twist message
         ↓
	micro_ros_agent on Raspberry Pi
         ↓
	USB serial XRCE-DDS
         ↓
	ESP32 micro-ROS client
         ↓
	left/right wheel target calculation
	Hardware

Current known hardware:

Raspberry Pi 5 4GB model
Ubuntu Server 24.04
ESP32-WROOM motor controller
USB serial connection between Pi and ESP32
CP2102 USB-to-TTL adapter ordered for dedicated debug UART
Pi cooler installed
NVMe HAT installed, SSD pending
Design Principles
	Pi handles 
		ROS 2 / NAV 2
		Planning
		High-level behavior.
	ESP32 handles 
		Real-time-ish motor control and hardware I/O.

Serial debug should be separate from micro-ROS transport.

Motor commands must fail safe if /cmd_vel stops.


last updated 
2026-06-25
