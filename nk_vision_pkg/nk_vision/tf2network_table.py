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
        self.declare_parameter('transfer_topics', [""])
        self.transfer_topics: list = self.get_parameter('transfer_topics').get_parameter_value().string_array_value

        # TF setup 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        self.tf_subscriber = self.create_subscription(
            TFMessage,
            'tf',
            self.read_external_measurements,
            10)

        # Network Table
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        table = self.inst.getTable(NTABLE_NAME)
        # while not self.inst.isConnected():
        #     self.inst = ntcore.NetworkTableInstance.getDefault()
        #     table = self.inst.getTable(NTABLE_NAME)
        #     self.inst.startClient4("vision_client")
        #     self.inst.setServerTeam(TEAM) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        #     self.inst.startDSClient()
        #     self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        #     self.get_logger().info('Trying to connect to the robot', throttle_duration_sec = 1.0)
        
        self.pubs: list() = []
        # test = table.getDoubleArrayTopic('').publish()
        # test.set()
        for topic in self.transfer_topics:
            self.pubs.append(table.getDoubleArrayTopic(topic).publish())

    def read_external_measurements(self, msg: TFMessage):
        """ Reads the measurements from all sources defined in pose_sources
        """
        # self.get_logger().info(f'{self.transfer_topics}')
        for link in self.transfer_topics: 
            # self.get_logger().info(f'{link}')
            try:
                self.get_logger().info("help")
                transform: TransformStamped = self.tfBuffer.lookup_transform(link, 'world', time.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                self.get_logger().info('ahhh')
                seconds, nanoseconds = transform.header.stamp.sec,transform.header.stamp.nanosec
                current_time_ros = float (nanoseconds) / 1e9 \
                    + float(seconds)            
                current_time_robot = self.inst.getServerTimeOffset() + current_time_ros
                pose = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w, current_time_robot]
                
                self.pubs[self.transfer_topics.index(link)].set(pose)
                self.get_logger().info(f'{pose}')


            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue

    # def tf_callback(self, msg: TFMessage):
    #     for transform_message in msg.transforms:
    #         transform_message: TransformStamped
    #         self.get_logger().info(f'{self.transfer_topics}')
    #         if transform_message.child_frame_id in self.transfer_topics:
    #             translation = transform_message.transform.translation
    #             rotation = transform_message.transform.rotation
    #             current_time_ros = float(transform_message.header.stamp.nanosec) / 1e9 \
    #                 + float(transform_message.header.stamp.sec)            
    #             current_time_robot = self.inst.getServerTimeOffset() + current_time_ros
    #             pose = [translation.x, translation.y, translation.z, rotation.x, rotation.y, rotation.z, rotation.w, current_time_robot]
    #             self.pubs[self.transfer_topics.index(transform_message.child_frame_id)].set(pose)
    #             self.get_logger().info(f'tf Mess child frame {transform_message.child_frame_id}', throttle_duration_sec = 1.0)


   
def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
