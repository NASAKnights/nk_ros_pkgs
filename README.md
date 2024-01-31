# NASA Knights ROS2 packages

## Installing and building

This project uses ROS2 Humble, and therefore needs to be running on Ubuntu 22.04

First make a workspace to store the code in, then clone the project to the src folder
```bash
mkdir vision_ws
cd vision_ws
git clone https://github.com/NASAKnights/nk_ros_pkgs.git -b humble --recurse-submodules
```

Install ROS2 Humble with the script in the top level folder of this project

```bash
cd nk_ros_pkgs
./install-ros.bash
```

Now you can build the project by running:

```bash
cd ..
colcon build --symlink-install
```

## Vision Pipeline

### Running

To run the pipeline you can use the following command:

```bash
source vision_ws/install/setup.bash # This will tell the system where to find the code that you just built is and needs to be run in every terminal
ros2 launch nk_vision nk_vision.launch.py
```

### Debugging

