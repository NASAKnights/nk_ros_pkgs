#!/usr/bin/env python3


import argparse
import rclpy
import time
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import numpy as np
import tf2_ros
from scipy.spatial.transform.rotation import Rotation
import numpy.matlib as npm
from geometry_msgs.msg import PoseArray, Pose, Transform, Quaternion
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
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
RATE = 50

class NetworkTable2TF(Node):
    """  
    """
    def __init__(self):
        """
        Initialize NetworkTable2TF
        """
        
        # Read parameters and create n subscribers to the tf topics
        super().__init__('NetworkTable2TF')
        self.declare_parameter('transfer_topics', [""])
        self.transfer_topics: list = self.get_parameter('transfer_topics').get_parameter_value().string_array_value

        # TF setup 
        self.tfBuffer = tf2_ros.Buffer()
        self.tf_broadcaster = TransformBroadcaster(self)

        # Network Table
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("pose_client")
        self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        table = self.inst.getTable(NTABLE_NAME)
        while not self.inst.isConnected():
            time.sleep(0.25)
            self.inst = ntcore.NetworkTableInstance.getDefault()
            table = self.inst.getTable(NTABLE_NAME)
            self.inst.startClient4("pose_client")
            self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
            self.inst.startDSClient()
            self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
            self.get_logger().info('Trying to connect to the robot', throttle_duration_sec = 1.0)
        
        self.subs = {}
        for topic in self.transfer_topics:
            self.subs[topic] = table.getDoubleArrayTopic(topic).subscribe([])

        self.timer = self.create_timer(1/RATE, self.transfer_data)

    def transfer_data(self):
        tfs = []
        for name, sub in self.subs.items():
            val = sub.get()
            if val != []:
                ros_time = val[7] - self.inst.getServerTimeOffset()
                ros_seconds = int(ros_time)
                ros_nanosec = int((ros_time - ros_seconds)*1e9)
                t = TransformStamped()
                t.child_frame_id = "world"
                t.header.stamp.sec = ros_seconds
                t.header.stamp.nanosec = ros_nanosec
                t.header.frame_id = name
                t.transform.translation.x = val[0]
                t.transform.translation.y = val[1]
                t.transform.translation.z = val[2]
                t.transform.rotation.x = val[3]
                t.transform.rotation.y = val[4]
                t.transform.rotation.z = val[5]
                t.transform.rotation.w = val[6]
                tfs.append(t)
        
        self.tf_broadcaster.sendTransform(tfs)
        
   
def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = NetworkTable2TF()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
