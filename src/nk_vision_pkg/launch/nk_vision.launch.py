# Copyright (c) 2008, Willow Garage, Inc.
# All rights reserved.
#
# Software License Agreement (BSD License 2.0)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions
# are met:
#
#  * Redistributions of source code must retain the above copyright
#    notice, this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above
#    copyright notice, this list of conditions and the following
#    disclaimer in the documentation and/or other materials provided
#    with the distribution.
#  * Neither the name of the Willow Garage nor the names of its
#    contributors may be used to endorse or promote products derived
#    from this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
# "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
# LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
# FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
# COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
# INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
# BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
# LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
# ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.
import os
import shutil

import glob
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

def get_next_bag_number(bag_folder):
    """
    Check the bag folder for existing bag recordings and return the next available number.
    """
    # Ensure the directory exists
    os.makedirs(bag_folder, exist_ok=True)

    # List existing bags with the pattern `bag_<num>`
    bag_files = glob.glob(os.path.join(bag_folder, "bag_*"))

    # Extract numbers from existing bag files
    existing_numbers = []
    for bag in bag_files:
        try:
            num = int(os.path.basename(bag).split('_')[-1])  # Extract last number
            existing_numbers.append(num)
        except ValueError:
            continue  # Ignore files that don't match the expected format

    # Get the next available number
    next_number = max(existing_numbers) + 1 if existing_numbers else 0
    return next_number

def copy_calibration_file():
    package_path = get_package_share_directory('nk_vision')

    # Define source and destination file paths
    source_file = os.path.join(package_path, 'config', 'arducam.yaml')
    for i in range(8):
        dest_file = f'/root/.ros/camera_info/ArducamOV9281USBCamera_Ardu__base_bus_0_usb_3610000_2_{i}_1_0_0c45_6366_1280x800.yaml'
        # Ensure the destination directory exists
        os.makedirs(os.path.dirname(dest_file), exist_ok=True)
        # Copy the file before launching the nodes
        shutil.copy(source_file, dest_file)
        print(f"Copied {source_file} to {dest_file}")

def generate_launch_description():
    # Ensure the directory exists
    copy_calibration_file()
    default_bag_folder = "vision_logs"
    next_bag_number = get_next_bag_number(os.path.join(get_package_share_directory('nk_vision'),default_bag_folder))
    bag_folder = os.path.join(get_package_share_directory('nk_vision'), default_bag_folder+f"/bag_{next_bag_number}")
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        # Node(
        #     package='camera_ros', executable='camera_node', output='screen',
        #     parameters=[{'width' : 1280,
        #                  'height' : 800,
        #                  'camera': 0,}],
        #     remappings=[('/camera/image_raw', '/camera_2/image_raw'),
        #                 ('/camera/camera_info', '/camera_2/camera_info')]
        #     ),
    
        Node(
            package='camera_ros', executable='camera_node', output='screen',
            parameters=[{ 'width' : 1280,
                         'height' : 800,
                         'camera': 0,}],
            remappings=[('/camera/image_raw', '/camera_1/image_raw'),
                        ('/camera/camera_info', '/camera_1/camera_info')]
            ),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(
                get_package_share_directory('nk_vision') + '/launch/aruco_tracker_1.launch.xml')),

        IncludeLaunchDescription(AnyLaunchDescriptionSource(
                get_package_share_directory('nk_vision') + '/launch/aruco_tracker_2.launch.xml')),

        # IncludeLaunchDescription(AnyLaunchDescriptionSource(
        #         get_package_share_directory('robot_2025_description') + '/launch/robot.launch.yaml')),
        
        IncludeLaunchDescription(AnyLaunchDescriptionSource(
                get_package_share_directory('frc_2025_field_description') + '/launch/main.launch.py')),

        Node(
            package='nk_vision', executable='tf2network_table.py', output='screen',
            parameters=[{'transfer_topics': ["world:base_link_1", "world:base_link_2", "base_link:branch"]}]),
        
        Node(package='nk_vision', executable='network_table2tf.py', output='screen',
            parameters=[{'transfer_topics': ["base_link"]}]),

        ExecuteProcess(
            cmd=['ros2', 'bag', 'record', 
                 '/tf',
                 '/camera_1/robot_description',
                 '/camera_2/robot_description',
                 '/robot_description',
                 '/joint_states',
                 '/tf',
                  '/field/robot_description',
                #   '/camera/realsense_camera/depth/color/points',
                  '-o',
                  bag_folder],
            output='screen'
        ),
        # Node(
        #     package='nk_vision', executable='stddev_network_table.py', output='screen',
        #     parameters=[{'transfer_topics': ["vision_stddev"]}])
    ])
