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

### Initial Startup / Deploy

To run the pipeline on the Jetson Nano you can use the following process:

1. Open the repository in a regular vscode window
2. Connect to the Jetson via Ethernet
3. Run the "Deploy Code" Task by pressing Ctrl+Shift+P and searching for "Tasks"
4. Input the password when asked (it will ask multiple times)
5. If the process succeeds, then move on to configuring your Ethernet settings

### Configuring Ethernet settings on the Jetson Nano

1. Find your Ethernet interface using ifconfig (in our example it is `enP8p1s0`)
2. Run `sudo ip link set [interface] down` (E.g: `sudo ip link set enP8p1s0 down`)
3. Run `sudo ip addr add 10.1.22.10/255.255.255.0 dev [interface]`
4. Run `sudo ip link set [interface] up`
