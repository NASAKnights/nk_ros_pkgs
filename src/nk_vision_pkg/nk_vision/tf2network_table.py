#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
import tf2_ros
import rclpy.time as rostime
from networktables import NetworkTables


TEAM = 122
NTABLE_NAME = "ROS2Bridge"
RATE = 50
NT_SERVER = f"10.{TEAM // 100}.{TEAM % 100}.2"  # Typical FRC robot IP
TIME_TOPIC = "time"

class TF2NetworkTable(Node):
    def __init__(self):
        """
        Initialize TF2NetworkTable
        """
        super().__init__('TF2NetworkTable')

        # Read parameters and create n subscribers to the tf topics
        self.declare_parameter('transfer_topics', [""])
        self.transfer_topics = self.get_parameter('transfer_topics').get_parameter_value().string_array_value

        # TF setup
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        # Initialize NetworkTables
        NetworkTables.initialize(server=NT_SERVER)
        self.table = NetworkTables.getTable(NTABLE_NAME)
        self.topic_pairs = self.parse_transfer_topics(self.transfer_topics)

        self.reconnect()

        # Parse transfer topics
        self.pubs = self.create_publishers(self.topic_pairs)

        # Create a timer for periodic transform updates
        self.timer_callback = self.create_timer(1 / RATE, self.read_external_measurements)

    def parse_transfer_topics(self, transfer_topics):
        """
        Parse the transfer topics into parent-child pairs.
        """
        topic_pairs = []
        for topic in transfer_topics:
            try:
                parent, child = topic.split(':')
                topic_pairs.append((parent.strip(), child.strip()))
            except ValueError:
                self.get_logger().error(f"Invalid format for topic: {topic}. Expected format 'parent:child'", throttle_duration_sec=1)
        return topic_pairs

    def create_publishers(self, topic_pairs):
        """
        Create publishers for each pair in NetworkTables.
        """
        publishers = {}
        for _, child in topic_pairs:
            topic = f"{child}"
            publishers[child] = self.table.getEntry(topic)  # Use getEntry instead of ntcore publish
        return publishers

    def read_external_measurements(self):
        """
        Reads the measurements and publishes them to NetworkTables.
        """
        if not NetworkTables.isConnected() or not self.connected:
            self.get_logger().warn("Lost connection to NetworkTables, attempting to reconnect...", throttle_duration_sec=1)
            self.reconnect()
            return
        
        robot_time = self.table.getEntry(TIME_TOPIC).getDoubleArray([])
        if robot_time != []:
            self.time_offest = robot_time[0] - time.time()
        

        for parent, child in self.topic_pairs:
            try:
                transform = self.tfBuffer.lookup_transform(parent, child, rostime.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                seconds = transform.header.stamp.sec
                nanoseconds = transform.header.stamp.nanosec
                current_time_ros = (float(nanoseconds) / 1e9 + float(seconds)) + self.time_offest
                
                # No direct time offset handling in pynetworktables, so just use ROS time
                pose = [
                    translation.x, translation.y, translation.z,
                    rotation.x, rotation.y, rotation.z, rotation.w,
                    current_time_ros
                ]

                # Publish to NetworkTables
                if child in self.pubs:
                    self.pubs[child].setDoubleArray(pose)  # Use setDoubleArray instead of ntcore set

            except Exception as e:
                self.get_logger().warn(f"Failed to lookup transform for {parent} -> {child}: {e}", throttle_duration_sec=1)

    def reconnect(self):
        """
        Reconnect to NetworkTables.
        """
        # Ensure NetworkTables is connected
        self.get_logger().warn("Waiting for NetworkTables connection...", throttle_duration_sec=1)
        NetworkTables.shutdown()
        NetworkTables.initialize(server=NT_SERVER)
        self.table = NetworkTables.getTable(NTABLE_NAME)
        self.pubs = self.create_publishers(self.topic_pairs)
        robot_time = self.table.getEntry(TIME_TOPIC).getDoubleArray([])
        if robot_time == []:
            self.connected = False
        else:
            self.time_offest = robot_time[0] - time.time()
            self.connected = True
        

def main(args=None):
    rclpy.init(args=args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
