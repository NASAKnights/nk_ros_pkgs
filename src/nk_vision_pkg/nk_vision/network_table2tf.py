#!/usr/bin/env python3

import rclpy
import time
from rclpy.node import Node
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from networktables import NetworkTables

TEAM = 122
NTABLE_NAME = "ROS2Bridge"
RATE = 10
NT_SERVER = f"10.{TEAM // 100}.{TEAM % 100}.2"  # Typical FRC robot IP
TIME_TOPIC = "time"

class NetworkTable2TF(Node):
    """ ROS 2 Node to transfer NetworkTables data to TF frames. """
    
    def __init__(self):
        super().__init__('NetworkTable2TF')
        
        # Read parameters for topics
        self.declare_parameter('transfer_topics', [""])
        self.transfer_topics = self.get_parameter('transfer_topics').get_parameter_value().string_array_value

        # TF setup
        self.tf_broadcaster = TransformBroadcaster(self)

        # Initialize NetworkTables
        NetworkTables.initialize(server=NT_SERVER)
        self.table = NetworkTables.getTable(NTABLE_NAME)
        self.poi_table = NetworkTables.getTable("POI")

        # Wait for NetworkTables connection
        self.reconnect()

        # ROS 2 timer for periodic updates
        self.timer = self.create_timer(1 / RATE, self.transfer_data)

    def transfer_data(self):
        """ Reads data from NetworkTables and publishes TF transforms """
        if not NetworkTables.isConnected():
            self.get_logger().warn("Lost connection to NetworkTables, attempting to reconnect...", throttle_duration_sec=1)
            self.reconnect()

        tfs = []
        for name, entry in self.subs.items():
            val = entry.getDoubleArray([])
            if val and len(val) >= 7:  # Ensure valid data length
                ros_time = val[7] + self.time_offest
                ros_seconds = int(ros_time)
                ros_nanosec = int((ros_time - ros_seconds) * 1e9)
                t = TransformStamped()
                t.header.frame_id = "world"
                t.child_frame_id = name
                t.header.stamp.sec = ros_seconds
                t.header.stamp.nanosec = ros_nanosec
                t.transform.translation.x = val[0]
                t.transform.translation.y = val[1]
                t.transform.translation.z = val[2]
                t.transform.rotation.x = val[3]
                t.transform.rotation.y = val[4]
                t.transform.rotation.z = val[5]
                t.transform.rotation.w = val[6]
                tfs.append(t)

        if tfs:
            self.tf_broadcaster.sendTransform(tfs)

    def reconnect(self):
        """
        Reconnect to NetworkTables.
        """
        # Ensure NetworkTables is connected
        self.get_logger().warn("Waiting for NetworkTables connection...", throttle_duration_sec=1)
        NetworkTables.initialize(server=NT_SERVER)
        self.table = NetworkTables.getTable(NTABLE_NAME)
        self.subs = {topic: self.table.getEntry(topic) for topic in self.transfer_topics}
        self.subs["reef"] = self.poi_table.getEntry("SmartDashboard/POI/POIREEF")
        self.time_offest = time.time() - self.table.getEntry(TIME_TOPIC).getDouble(0.0)


def main(args = None):
    rclpy.init(args = args)

    tf2nt_node = NetworkTable2TF()

    rclpy.spin(tf2nt_node)

if __name__ == "__main__":
    main()
