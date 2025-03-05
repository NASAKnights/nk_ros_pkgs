#!/bin/bash

# Set error handling
set -e

# ROS 2 and workspace setup
echo "Setting up ROS 2 environment..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Run the launch file
ros2 launch robot_2025_description robot.launch.yaml