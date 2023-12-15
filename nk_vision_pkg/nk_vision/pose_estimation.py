#!/usr/bin/env python3


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
from tf2_msgs.msg import _tf_message
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from networktables import NetworkTables
import ntcore


class PoseEstimationNode(Node):
    """ The pose estimation node will look at the robot odometry, and n pose estimates, 
        to approximate the location of the robot in the world coordinate frame.

        The initial approach will be to take the average of the n measurements and combine them to form
        a more 'true' estimate. Then incorporating the odometry.

        There are a few edge cases to handle that mean we cannot take a simple average.
            If a source of data is not currently publishing (no markers are detected by the camera)
            If a source has an absurd measurement, ie. outside the field, teleports by some large value, or is floating way off the ground
        
    """
    def __init__(self):
        """
        Initialize PoseEstimationNode
        """
        
        # Read parameters and create n subscribers to the tf topics

        self.pose_sources = ['base_link_1', 'base_link_2']
        self.camera = {'camera_1':False, 'camera_2':False}
        ## key is link, value is pose estimates
        self.pose_estimates = {}
        self.pose = Pose()

        self.rate = 20
        self.acceptable_timeout = 1/self.rate 
        
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)

        self.tf_subscriber = self.create_subscription(
            _tf_message,
            'tf',
            self.check_timestamp,
            10)

        # Read parameter and create subscriber to robot odometry topic (working)

        # Network Table
        inst = ntcore.NetworkTableInstance.getDefault()
        table = inst.getTable("datatable")
        inst.startClient4("example client")
        inst.setServerTeam(122) # where TEAM=190, 294, etc, or use inst.setServer("hostname") or similar
        self.network_table_pub = table.getDoubleArrayTopic("pose").publish()
                    
        # Set up tf publisher (publish pose: position and orientation) (initalize self.pose = pose) (done)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Set up timer callback to ensure PoseEstimate is published at 20Hz (done)
        timer_period = self.acceptable_timeout
        self.timer = self.create_timer(timer_period, self.find_avg_pose)
        
    

    def read_external_measurements(self):
        """ Reads the measurements from all sources defined in pose_sources
        """
        for link in self.pose_sources: 
            try:
                trans = self.tfBuffer.lookup_transform(link, 'world', time.Time())
                self.pose_estimates[link] = trans

            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                continue
        pass

    def publish_estimate(self):
        pass

    def find_avg_pose(self):
        """ Finds the average pose from all sources
        """
        self.find_avg_position()
        self.find_avg_orientation()
        self.pose.position = self.avg_position
        self.pose.orientation: Quaternion
        self.pose.orientation.x = self.avg_orientation[0]
        self.pose.orientation.y = self.avg_orientation[1]
        self.pose.orientation.z = self.avg_orientation[2]
        self.pose.orientation.w = self.avg_orientation[3]
        ## put in avg_pose to help constantly get new data
        t = TransformStamped()
        t.child_frame_id = "world"
        t.header.stamp = time.Time()
        t.header.frame_id = "base_link"
        t.transform.rotation = self.pose.orientation
        t.transform.translation = self.pose.position
        self.tf_broadcaster.sendTransform(t)
        pose = [self.pose.position.x, self.pose.position.y, self.pose.position.z, t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w]
        self.network_table_pub.set(pose)


    def check_Timestamp(self, msg: _tf_message.TFMessage):
        ## reset the dict to allow for clean data again
        self.pose_estimates = {}
        for transform_message in msg.transforms:
            transform_message: TransformStamped
            if transform_message.child_frame_id in self.camera.keys():
                ## check for timestamp with child frame to see if recent
                ## stored as int, nanosec convert into sec and float, then add sec and nanosec

                ## nanosec
                stamp_nanosecond = float(transform_message.header.stamp.nanosec) / 1000000000
                
                ## second add with nanosec
                stamp_second = transform_message.header.stamp.sec
                stamp_total_time = stamp_nanosecond + stamp_second
                time_since_last_message = time.Time() - stamp_total_time

                ## if time_since_last_message is <= self.acceptable timeout. Append to dict with child frame id and transform
                if time_since_last_message <= self.acceptable_timeout:
                    self.pose_estimates[transform_message.child_frame_id] = transform_message._transform
                
                ## if longer then skip
                elif time_since_last_message > self.acceptable_timeout:
                    pass


    def find_avg_position(self):
        """Find average position, by taking location from camera pose estimate we can find the average position.
        """
        positions = np.array()
        for pose, i in enumerate(self.pose_estimates):
            if self.is_valid_measurement(pose.header):
                positions[i,0] = pose.position.x
                positions[i,1] = pose.position.y
                positions[i,2] = pose.position.z
        self.avg_position = np.mean(positions, 0)


    def find_avg_orientation(self):
        orientations = np.array()
        for pose, i in enumerate(self.pose_estimates):
            orientations[i,:] = [pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w]

        self.avg_orientation: list[float] = self.weightedAverageQuaternions(orientations)
    

    def weightedAverageQuaternions(Q, w=[]):
        """ Average multiple quaternions with specific weights
            The weight vector w must be of the same length as the number of rows in the
            quaternion maxtrix Q

            Taken from https://github.com/christophhagen/averaging-quaternions/blob/master/averageQuaternions.py

        Args:
            Q (np.array): Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
            w (_type_): The weight vector w, it must be of the same length as the number of rows in the
            quaternion maxtrix Q

        Returns:
            np.array: the resultant average orienation
        """
        # Number of quaternions to average
        M = Q.shape[0]
        A = npm.zeros(shape=(4,4))
        weightSum = 0

        for i in range(0,M):
            q = Q[i,:]
            A = w[i] * np.outer(q,q) + A
            weightSum += w[i]

        # scale
        A = (1.0/weightSum) * A

        # compute eigenvalues and -vectors
        eigenValues, eigenVectors = np.linalg.eig(A)

        # Sort by largest eigenvalue
        eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]

        # return the real part of the largest eigenvector (has only real part)
        return np.real(eigenVectors[:,0].A1)
