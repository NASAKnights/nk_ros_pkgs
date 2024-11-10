#!/bin/bash
cd /home//home/nasa-knights/vision_logs
source /vision_ws/install/setup.bash
ros2 launch nk_vision nk_vision.launch.py > /vision_ws/log/recent.log