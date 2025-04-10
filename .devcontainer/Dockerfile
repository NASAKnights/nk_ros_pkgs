FROM isaac_ros_dev-aarch64

# Set up ROS 2 environment
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    ros-${ROS_DISTRO}-web-video-server \
    v4l-utils
RUN apt-get install -y ros-humble-pcl-conversions ros-humble-camera-ros
RUN pip install pynetworktables urllib3==1.26.15 requests-toolbelt==0.10.1 \
    fastapi uvicorn numpy
RUN apt-get update && apt-get install -y v4l-utils

RUN usermod -aG video root
# Copy scripts to persistent location
COPY entrypoint.sh /opt/ros_vision/scripts/entrypoint.sh
COPY run_robot.sh /opt/ros_vision/scripts/run_robot.sh
COPY start_web_dashboard.sh /opt/ros_vision/scripts/start_web_dashboard.sh
RUN chmod +x /opt/ros_vision/scripts/*.sh

# Build and move frontend
WORKDIR /tmp/frontend
COPY src/web_gui/frontend/package*.json ./
RUN npm install -g
# COPY src/web_gui/frontend/ .
# RUN npm run build
# Install serve globally
# RUN npm install -g serve
# RUN mkdir -p /opt/ros_vision/frontend
# RUN cp -r build /opt/ros_vision/frontend

# Copy backend code
RUN mkdir -p /opt/ros_vision/backend
COPY src/web_gui/backend /opt/ros_vision/backend

# Set final entrypoint
# ENTRYPOINT ["/opt/ros_vision/scripts/entrypoint.sh"]
