from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to the DepthAI and RTAB-Map launch files
    depthai_launch_file = os.path.join(
        get_package_share_directory('depthai_ros_driver'),
        'launch',
        'depthai_stereo_inertial_node.launch.py'
    )
    rtabmap_launch_file = os.path.join(
        get_package_share_directory('rtabmap_ros'),
        'launch',
        'rtabmap.launch.py'
    )

    # Parameters for depthai_ros_driver
    depthai_params = {
        'camera_name': 'oakd',
        'stereo_fps': 30,  # FPS for the stereo images
        'imu_fps': 200,    # FPS for the IMU
        'stereo_resolution': '720p',  # Resolution of the stereo images
        'publish_topic': True         # Ensure topics are published
    }

    # Parameters for RTAB-Map
    rtabmap_params = {
        'args': '--delete_db_on_start',
        'stereo': 'true',
        'left_image_topic': '/oakd/left/image_rect',
        'right_image_topic': '/oakd/right/image_rect',
        'left_camera_info_topic': '/oakd/left/camera_info',
        'right_camera_info_topic': '/oakd/right/camera_info',
        'imu_topic': '/oakd/imu',
        'frame_id': 'oakd_frame',
        'approx_sync': 'true',
        'approx_sync_max_interval': '0.001',
        'wait_imu_to_init': 'true'
    }

    # Include DepthAI ROS driver launch file
    depthai_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(depthai_launch_file),
        launch_arguments=depthai_params.items(),
    )

    # Include RTAB-Map launch file
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rtabmap_launch_file),
        launch_arguments=rtabmap_params.items(),
    )

    return LaunchDescription([
        depthai_launch,
        rtabmap_launch
    ])
