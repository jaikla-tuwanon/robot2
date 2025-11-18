#!/bin/bash

# SLAM Gazebo Launch Script for RBKairos
# This script sets up all necessary environment variables and launches SLAM mapping

cd /home/tuwanon/week6_ws

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source install/setup.bash

# Set graphics environment for WSL (software rendering)
export LIBGL_ALWAYS_SOFTWARE=1
export MESA_GL_VERSION_OVERRIDE=3.3
export GALLIUM_DRIVER=llvmpipe

# Launch SLAM Gazebo
ros2 launch control_rbkairos slam_gazebo.launch.py
