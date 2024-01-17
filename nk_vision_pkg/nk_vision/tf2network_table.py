#!/usr/bin/env python3


import argparse
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import tf2_ros
from scipy.spatial.transform.rotation import Rotation
import numpy.matlib as npm
from geometry_msgs.msg import PoseArray, Pose, Transform, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
import rclpy.time as time
from tf2_msgs.msg import TFMessage
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from networktables import NetworkTables
from networktables.entry import NetworkTableEntry
import ntcore
import logging
logging.basicConfig(level=logging.DEBUG)

TEAM = 122
NTABLE_NAME = "ROS2Bridge"

class TF2NetworkTable(Node):
    """  
    """
    def __init__(self):
        """
        Initialize TF2NetworkTable
        """
        
        # Read parameters and create n subscribers to the tf topics
        super().__init__('TF2NetworkTable')
        self.declare_parameter('transfer_topics')
        self.transfer_topics: list = self.get_parameter('transfer_topics').split()

        # TF setup 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.tf_callback,
            10)

        # Network Table
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        while not self.inst.isConnected():
            self.inst = ntcore.NetworkTableInstance.getDefault()
            table = self.inst.getTable(NTABLE_NAME)
            self.inst.startClient4("vision_client")
            self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
            self.inst.startDSClient()
            self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
            self.get_logger().info('Trying to connect to the robot', throttle_duration_sec = 1.0)
        
        self.pubs: list = []
        for topic in self.transfer_topics:
            self.pubs.append(table.getDoubleArrayTopic(topic).publish())

    def tf_callback(self, msg: TFMessage):
        for transform_message in msg.transforms:
            transform_message: TransformStamped
            if transform_message.child_frame_id in self.transfer_topics:
                translation = transform_message.transform.translation
                rotation = transform_message.transform.rotation
                current_time_ros = float(transform_message.header.stamp.nanosec) / 1000000000 \
                    + float(transform_message.header.stamp.nanosec)            
                current_time_robot = self.inst.getServerTimeOffset() + current_time_ros
                pose = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w, current_time_robot]
                self.pubs[self.transfer_topics.index(transform_message.child_frame_id)].set(pose)

   
def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
