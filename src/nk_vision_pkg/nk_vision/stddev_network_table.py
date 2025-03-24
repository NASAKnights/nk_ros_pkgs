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
import pandas as pd
logging.basicConfig(level=logging.DEBUG)

TEAM = 122
NTABLE_NAME = "ROS2Bridge"
RATE = 50


data = [
        [2, 5.165569711e-05, 0.0000618020, 0.0002046913], 
        [4, 0.0001285311218, 0.0017269060, 0.0011564079],
        [6, 0.0007173800092, 0.0003573570, 0.0049201348],
        [8, 0.0005020067207, 0.0001876331, 0.0035188319],
        [10, 0.0001207145544, 0.000495630, 0.0024328938],
        [15, 0.01184643065, 0.0017390639, 0.0168024665] 
        ]

STDDEV = pd.DataFrame(data, columns = ['Distance', 'X', 'Y', 'Z'])
#EXAMPLE
#Interpolated value = np.interp(3.4, STDDEV.Distance, STDDEV.X)
#Would return interpolated X value for a distance of 3.4 

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
        self.count = 0

    def find_closest_marker(self):
        """
        Finds the transform between the camera and the closest marker
        """
        # Loop through all transforms and find the camera-to-marker transforms
        distance = 0
        try:
            for x in range(36): # for 36 markers
                possible_marker = "marker_" + str(x)
                if self.tfBuffer.can_transform(possible_marker, "camera", rclpy.time.Time()): 
                    self.get_logger().warning(f"Made into if statement {possible_marker}")
                    try:
                        #make transform an object or something from camera_1 (the robot) to the april tag
                        frame = self.tfBuffer.lookup_transform(possible_marker, "camera", rclpy.time.Time())
                        # Calculate the distance to the marker
                        dx = frame.transform.translation.x
                        dy = frame.transform.translation.y
                        dz = frame.transform.translation.z

                        #self.get_logger().warning(f"dx: {dx}\ndy: {dy}\ndz: {dz}")
                        
                        distance = np.sqrt(dx**2 + dy**2 + dz**2)  # Euclidean distance
                        
                    except:
                        self.get_logger().warning("failed transform"*5)
                        continue  # If there's no transform, continue to the next one
                else:
                    self.get_logger().warning(f"Failed to make it through the if statement {x}")
        except:
            self.get_logger().warning("\n\n NO MARKER FOUND ", x, "\n\n")
            pass

        self.get_logger().warning(f"\n\n\n\n\n\n\nDistance: {distance}")    
        return distance
        
    def read_external_measurements(self):
        """ 
        Publish stddev of measurement based on camera distance to marker
        """
        while not self.inst.isConnected():
            time.sleep(0.05)
            self.reconnect()
        
        # find_marker_distance
        distance = self.find_closest_marker()

        # Take values from stddev table based on transform
        # interpolate the values from the table and find the stddev
        x_stddev = np.interp(distance, STDDEV.Distance, STDDEV.X)
        y_stddev = np.interp(distance, STDDEV.Distance, STDDEV.Y)
        z_stddev = np.interp(distance, STDDEV.Distance, STDDEV.Z)

        #publish the stdev out through the network table
        pose = [x_stddev, y_stddev, z_stddev]            
        self.table.getEntry("vision_stddev").setDoubleArray(pose)

    def reconnect(self):
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable(NTABLE_NAME)
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM)
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        self.get_logger().info('Trying to connect to the robot', throttle_duration_sec = 2.0)
        self.pub = self.table.getDoubleArrayTopic("vision_stddev").publish()
            
        
def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
