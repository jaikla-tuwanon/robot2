#!/bin/bash

# Teleop Keyboard Launch Script for RBKairos
# Run this in a separate terminal after run_slam.sh is running

cd /home/tuwanon/week6_ws

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch teleop with topic remapping
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args --remap cmd_vel:=/mecanum_drive_controller/reference_unstamped
