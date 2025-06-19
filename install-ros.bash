#!/bin/bash

locale  # check for UTF-8

sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

locale  # verify settings


sudo apt update && sudo apt install curl -y
sudo apt install software-properties-common npm uvicorn python3-pip python3-fastapi -y
sudo add-apt-repository universe

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update

sudo apt upgrade -y

sudo apt install ros-jazzy-desktop -y

sudo apt install ros-jazzy-aruco ros-jazzy-usb-cam ros-jazzy-image-pipeline ros-jazzy-diagnostic-updater ros-jazzy-camera-ros ros-jazzy-web-video-server -y

pip install numpy==1.23.0 # Default version breaks with dependent packages

echo "source /opt/ros/jazzy/setup.bash" >> ~/.bash
