from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            'input_topic',
            default_value='/camera/rs_cam_1/depth/color/points',
            description='Input PointCloud2 topic'
        ),
        DeclareLaunchArgument(
            'reference_ply_filepath',
            default_value='/path/to/your/file.ply',
            description='Path to the .ply file'
        ),
        ComposableNodeContainer(
            name='realsense_icp_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container_mt',  # Multithreaded container
            composable_node_descriptions=[
                ComposableNode(
                    package='realsense2_camera',
                    plugin='realsense2_camera::RealSenseNodeFactory',
                    name='realsense_camera',
                    parameters=[
                    ('camera_name', 'rs_cam_1'),
                    # ("serial_no", "\'036222071448\'"),
                    ('enable_color', 'true'),
                    ('rgb_camera.color_format', 'RGB8'),
                    ('rgb_camera.enable_auto_exposure', 'true'),
                    ('enable_depth', 'true'),
                    ('enable_infra', 'false'),
                    ('enable_infra1', 'false'),
                    ('enable_infra2', 'false'),
                    ('depth_module.depth_profile', '640,480,15'),
                    ('depth_module.depth_format', 'Z16'),
                    ('depth_module.infra_format', 'RGB8'),
                    ('depth_module.hdr_enabled', 'false'),
                    ('depth_module.enable_auto_exposure', 'true'),
                    ('enable_sync', 'true'),
                    ('enable_rgbd', 'true'),
                    # ('enable_gyro'),
                    # ('enable_accel'),
                    # ('clip_distance'),
                    ('publish_tf', 'true'),
                    ('tf_publish_rate', '10.0'),
                    ('pointcloud.enable', 'true'),
                    ('align_depth.enable', 'true'),
                    ('decimation_filter.enable', 'false'),
                    ('spatial_filter.enable', 'true'),
                    ('temporal_filter.enable', 'true'),
                    ('disparity_filter.enable', 'false'),
                    ('hole_filling_filter.enable', 'false'),
                    ('reconnect_timeout', '30.0' )
                    ]
                ),
                ComposableNode(
                    package='icp_obj_pose',  # Your package name
                    plugin='ICPNode',
                    name='icp_node',
                    parameters=[
                        {'input_topic': '/camera/depth/color/points'},  # Update topic name if necessary
                        {'reference_ply_filepath': LaunchConfiguration('reference_ply_filepath')}
                    ]
                )
            ],
            output='screen'
        )
    ])
