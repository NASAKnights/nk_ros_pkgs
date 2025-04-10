from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='v4l2_camera_node',
            parameters=[{
                'device': '/dev/video0',
                'width': 1280,
                'height': 800,
            }],
            output='screen',
        )
    ])
