#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
import numpy as np
import tf2_ros
import pandas as pd
from networktables import NetworkTables
from aruco_opencv_msgs.msg import ArucoDetection

TEAM = 122
NTABLE_NAME = "ROS2Bridge"
RATE = 50
NT_SERVER = f"10.{TEAM // 100}.{TEAM % 100}.2"  # Typical FRC robot IP

data = [
        [0.6096, 0.005165569711, 0.00618020, 0.02046913], 
        [1.2192, 0.01285311218, 0.017269060, 0.2564079],
        [1.8288, 0.05173800092, 0.02573570, 0.49201348],
        [2.4384, 0.07020067207, 0.04876331, 0.7188319],
        [3.048, 0.1207145544, 0.0695630, 0.9328938],
        [4.572, 0.184643065, 0.15390639, 1.28024665],
        [6.096, 100.0, 100.0, 100.0] 
        ]

STDDEV = pd.DataFrame(data, columns = ['Distance', 'X', 'Y', 'Z'])
#EXAMPLE
#Interpolated value = np.interp(3.4, STDDEV.Distance, STDDEV.X)
#Would return interpolated X value for a distance of 3.4 

class StddevNetworkTable(Node):
    """  
    """
    def __init__(self):
        """
        Initialize StddevNetworkTable
        """
        
        # Read parameters and create n subscribers to the tf topics
        super().__init__('StddevNetworkTable')

        # TF setup 
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)
        # Network Table
        while not NetworkTables.isConnected():
            self.get_logger().info("Connecting", throttle_duration_sec=1)
            self.reconnect()
            
        self.subscription = self.create_subscription(
                    ArucoDetection,
                    '/aruco_detections',
                    self.read_external_measurements,
                    1)        

    def find_closest_marker(self, msg):
        """
        Finds the transform between the camera and the closest marker
        """
        # Loop through all transforms and find the closest camera-to-marker transforms
        closest = 999999

        for x in msg.markers:
            #make transform an object or something from camera_1 (the robot) to the april tag

            # Calculate the distance to the marker
            dx = x.pose.position.x
            dy = x.pose.position.y
            dz = x.pose.position.z

            #self.get_logger().warning(f"dx: {dx}\ndy: {dy}\ndz: {dz}")
            
            closest = min(np.sqrt(dx**2 + dy**2 + dz**2), closest) # Euclidean distance
        
        self.get_logger().info(f"the closest distance is {closest}", throttle_duration_sec = 2.0)
        return closest
      

    def read_external_measurements(self, msg):
        """ 
        Publish stddev of measurement based on camera distance to marker
        """
        if not NetworkTables.isConnected():
            self.reconnect()
            return
        
        # find_marker_distance
        distance = self.find_closest_marker(msg)

        # Take values from stddev table based on transform
        # interpolate the values from the table and find the stddev
        x_stddev = np.interp(distance, STDDEV.Distance, STDDEV.X)
        y_stddev = np.interp(distance, STDDEV.Distance, STDDEV.Y)
        z_stddev = np.interp(distance, STDDEV.Distance, STDDEV.Z)

        #publish the stdev out through the network table
        stddev = [x_stddev, y_stddev, z_stddev]            
        self.get_logger().info(f"The stddev is {stddev}", throttle_duration_sec = 2.0)
        self.table.getEntry("vision_stddev").setDoubleArray(stddev)

    def reconnect(self):
            """
            Reconnect to NetworkTables.
            """
            # Ensure NetworkTables is connected
            self.get_logger().warn("Waiting for NetworkTables connection...", throttle_duration_sec=1)
            NetworkTables.shutdown()
            NetworkTables.initialize(server=NT_SERVER)
            time.sleep(1)
            self.table = NetworkTables.getTable(NTABLE_NAME)
        
def main(args = None):
    rclpy.init(args = args)

    stddev_node = StddevNetworkTable()

    rclpy.spin(stddev_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
