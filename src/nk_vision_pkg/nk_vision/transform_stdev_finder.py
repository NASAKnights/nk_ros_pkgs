#!/usr/bin/env python3



import math
import pandas as pd

from geometry_msgs.msg import Twist

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

from turtlesim.srv import Spawn


class TransformStDevFinder(Node):

    def __init__(self):
        super().__init__('transform_stdev_finder')
        #Creating empty data frames for later use
        transform = {
        "x": [],
        "y": [],
        "z": []
        }
        self.df_transform = pd.DataFrame(transform)

        # self.get_logger().info("strng 1")

        rotation = {
        "x": [],
        "y": [],
        "z": []
        }
        # self.get_logger().info("string 2")
        self.df_rotation = pd.DataFrame(rotation)
                

        # Declare and acquire `target_frame` parameter
        self.target_frame = self.declare_parameter(
            'target_frame', 'note').get_parameter_value().string_value
        # self.get_logger().info("string 3")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Call on_timer function every second
        self.timer = self.create_timer(.001, self.on_timer)
        # self.get_logger().info("string 4")

    def on_timer(self):
        # Store frame names in variables that will be used to
        # compute transformations
        from_frame_rel = self.target_frame
        to_frame_rel = 'rs_cam_1_link'
        self.get_logger().info("string 5")

                # Look up for the transformation between target_frame and turtle2 frames
                # and send velocity commands for turtle2 to reach target_frame
        try:
            t = self.tf_buffer.lookup_transform(
                to_frame_rel,
                from_frame_rel,
                rclpy.time.Time())

            # self.get_logger().info("string 6")
            # getting a list of new x, y, z transforms_
            self.get_logger().info(f"Transform: {t}")
            new_transform = {"x": t.transform.translation.x, "y": t.transform.translation.y, "z": t.transform.translation.z}
            new_rotation = {"x": t.transform.rotation.x, "y": t.transform.rotation.y, "z": t.transform.rotation.z}

            # df_new_transform = pd.DataFrame(new_transform)  # Make it a list of rows
            # df_new_rotation = pd.DataFrame(new_rotation)    # Make it a list of rows

            #converting lists into dataframes
            df_new_transform = pd.DataFrame([new_transform])
            df_new_rotation = pd.DataFrame([new_rotation])

            # converting the lists into dataframes and concating them to the end of the original dataframes
            
            self.df_transform = pd.concat([self.df_transform, df_new_transform], ignore_index=True)
            self.df_rotation = pd.concat([self.df_rotation, df_new_rotation], ignore_index=True)


            # self.get_logger().info("string 7")

        except TransformException as ex:
                self.get_logger().info(
                f'Could not transform {to_frame_rel} to {from_frame_rel}: {ex}')
                return

    def avg(self):
         pass

def main():
    rclpy.init()
    node = TransformStDevFinder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:

        print("\n\n\n\n")
        #converting to numpy arrays
        x_transform = node.df_transform['x'].to_numpy()
        y_transform = node.df_transform['y'].to_numpy()
        z_transform = node.df_transform['z'].to_numpy()

        z_rotation = node.df_rotation['z'].to_numpy()

        enorm = (x_transform.mean()**2 + y_transform.mean()**2) ** (1/2) # + z_transform.mean()**2) ** (1/2)

        print("Euclidean Norm: ", f'{enorm:.10f}')
        print("X Transform: ", f'{x_transform.std():.10}')
        print("Y Transform: ", f'{y_transform.std():.10f}')

        print("Z Rotaion: ", f'{z_rotation.std():.10f}')


        # print("\n\ndf_transform:\n", node.df_transform)
if __name__ == "__main__":
    main()