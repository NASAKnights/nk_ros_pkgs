FROM isaac_ros_dev-aarch64

# Set up ROS 2 environment
RUN apt-get update && apt-get install -y \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-image-transport \
    v4l-utils
RUN apt-get install -y ros-humble-pcl-conversions ros-humble-camera-ros
RUN pip install pynetworktables urllib3==1.26.15 requests-toolbelt==0.10.1
RUN apt-get update && apt-get install -y v4l-utils

RUN usermod -aG video root

RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
