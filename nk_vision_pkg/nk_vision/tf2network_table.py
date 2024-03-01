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
import rclpy.time as rostime
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

class TF2NetworkTable(Node):
    """  
    """
    def __init__(self):
        """
        Initialize TF2NetworkTable
        """
        
        # Read parameters and create n subscribers to the tf topics
        super().__init__('TF2NetworkTable')
        self.declare_parameter('transfer_topics', [""])
        self.transfer_topics: list = self.get_parameter('transfer_topics').get_parameter_value().string_array_value

        # TF setup 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        # Network Table
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM) 
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        self.table = self.inst.getTable(NTABLE_NAME)
        while not self.inst.isConnected():
            time.sleep(0.25)
            self.reconnect()
        
        self.timer_callback = self.create_timer(
            1/RATE,
            self.read_external_measurements)

    def read_external_measurements(self):
        """ Reads the measurements from all sources defined in pose_sources
        """
        while not self.inst.isConnected():
            time.sleep(0.05)
            self.reconnect()
        for link in self.transfer_topics: 
            try:
                transform: TransformStamped = self.tfBuffer.lookup_transform('world', link, rostime.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                seconds, nanoseconds = transform.header.stamp.sec,transform.header.stamp.nanosec
                current_time_ros = float (nanoseconds) / 1e9 \
                    + float(seconds) 
                if (self.inst.getServerTimeOffset() != None):
                    current_time_robot = self.inst.getServerTimeOffset()/1e6 + current_time_ros
                else:
                    current_time_robot = 0
                
                pose = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w, current_time_robot]            
                self.pubs[self.transfer_topics.index(link)].set(pose)


            except:
                continue
   
    def reconnect(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable(NTABLE_NAME)
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM)
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        self.get_logger().info('Trying to connect to the robot', throttle_duration_sec = 1.0)
        self.pubs: list() = []
        for topic in self.transfer_topics:
            self.pubs.append(self.table.getDoubleArrayTopic(topic).publish())
            
        
def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
