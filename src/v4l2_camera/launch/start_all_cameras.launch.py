from launch import LaunchDescription
from launch_ros.actions import Node
import os
import subprocess
import re


def is_realsense(device_path):
    try:
        info = subprocess.check_output(
            ['udevadm', 'info', '--query=all', '--name', device_path]).decode()
        return 'RealSense' in info
    except:
        return False


def get_usb_port_path(device_path):
    try:
        output = subprocess.check_output(
            ['udevadm', 'info', '--name', device_path]).decode()
        for line in output.splitlines():
            if 'DEVPATH' in line:
                parts = line.split('/')
                usb_parts = [p for p in parts if '-' in p]
                port_path = '_'.join(usb_parts)
                return re.sub(r'[^a-zA-Z0-9_]', '_', port_path)  # sanitize
    except:
        pass
    return None


def generate_launch_description():
    nodes = []
    camera_index = 0

    for dev in sorted(os.listdir('/dev')):
        if not dev.startswith('video'):
            continue
        dev_path = f'/dev/{dev}'
        if is_realsense(dev_path):
            continue

        port_id = get_usb_port_path(dev_path)
        if not port_id:
            port_id = f"fallback_{camera_index}"

        node_name = f"camera_{port_id}"
        node_name = re.sub(r'[^a-zA-Z0-9_]', '_', node_name)  # final cleanup

        nodes.append(
            Node(
                package='v4l2_camera',
                executable='v4l2_camera_node',
                namespace=node_name,
                name=node_name,
                parameters=[{
                    'device': dev_path,
                    'device_id': port_id
                }]
            )
        )
        camera_index += 1

    return LaunchDescription(nodes)
