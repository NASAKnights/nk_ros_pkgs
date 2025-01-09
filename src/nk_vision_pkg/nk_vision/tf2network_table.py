#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
import ntcore
import logging
from geometry_msgs.msg import TransformStamped
import tf2_ros
import rclpy.time as rostime

logging.basicConfig(level=logging.DEBUG)

TEAM = 122
NTABLE_NAME = "ROS2Bridge"
RATE = 50


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

        # NetworkTable setup
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM)
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        self.table = self.inst.getTable(NTABLE_NAME)
        self.ensure_connection()

        # Parse transfer topics
        self.topic_pairs = self.parse_transfer_topics(self.transfer_topics)
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
                self.get_logger().error(f"Invalid format for topic: {topic}. Expected format 'parent:child'")
        return topic_pairs

    def create_publishers(self, topic_pairs):
        """
        Create publishers for each pair in NetworkTables.
        """
        publishers = {}
        for _, child in topic_pairs:
            topic = f"{child}"
            publishers[child] = self.table.getDoubleArrayTopic(topic).publish()
        return publishers

    def ensure_connection(self):
        """
        Ensure NetworkTable connection.
        """
        while not self.inst.isConnected():
            time.sleep(0.25)
            self.reconnect()

    def read_external_measurements(self):
        """
        Reads the measurements and publishes them to NetworkTables.
        """
        self.ensure_connection()
        for parent, child in self.topic_pairs:
            try:
                transform = self.tfBuffer.lookup_transform(parent, child, rostime.Time())
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                seconds = transform.header.stamp.sec
                nanoseconds = transform.header.stamp.nanosec
                current_time_ros = float(nanoseconds) / 1e9 + float(seconds)

                # Convert ROS time to robot time
                if self.inst.getServerTimeOffset() is not None:
                    current_time_robot = self.inst.getServerTimeOffset() / 1e6 + current_time_ros
                else:
                    current_time_robot = 0

                pose = [
                    translation.x, translation.y, translation.z,
                    rotation.x, rotation.y, rotation.z, rotation.w,
                    current_time_robot
                ]

                # Publish to NetworkTables
                if child in self.pubs:
                    self.pubs[child].set(pose)

            except Exception as e:
                self.get_logger().warn(f"Failed to lookup transform for {parent} -> {child}: {e}")

    def reconnect(self):
        """
        Reconnect to NetworkTables.
        """
        self.inst = ntcore.NetworkTableInstance.getDefault()
        self.table = self.inst.getTable(NTABLE_NAME)
        self.inst.startClient4("vision_client")
        self.inst.setServerTeam(TEAM)
        self.inst.startDSClient()
        self.inst.setServer("host", ntcore.NetworkTableInstance.kDefaultPort4)
        self.get_logger().info('Reconnecting to the robot', throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)

    tf2nt_node = TF2NetworkTable()

    rclpy.spin(tf2nt_node)


if __name__ == "__main__":
    main()
