#!/bin/bash

# Start backend API
cd /opt/ros_vision/backend
uvicorn main:app --host 0.0.0.0 --port 8080 --reload &

# Serve frontend React build
cd /workspaces/isaac_ros-dev/src/web_gui/frontend
npm install
npm run dev -- --host 0.0.0.0 --port 3000