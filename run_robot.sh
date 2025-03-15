#!/bin/bash

# Set error handling
set -e

# ROS 2 and workspace setup
echo "Setting up ROS 2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the launch file
ros2 run robot_state_publisher robot_state_publisher $(ros2 pkg prefix robot_2025_description)/share/robot_2025_description/urdf/camera_1.urdf &
ros2 run robot_state_publisher robot_state_publisher $(ros2 pkg prefix robot_2025_description)/share/robot_2025_description/urdf/camera_2.urdf &
ros2 run robot_state_publisher robot_state_publisher $(ros2 pkg prefix robot_2025_description)/share/robot_2025_description/urdf/robot_2025.urdf &
ros2 launch nk_vision nk_vision.launch.py 
sleep 10s
ros2 launch robot_2025_description robot.launch.yaml
