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
        DeclareLaunchArgument(
            'camera_frame',
            default_value='rs_cam_1_depth_frame',
            description='Camera frame ID'
        ),
        DeclareLaunchArgument(
            'object_name',
            default_value='note',
            description='Name of the object for TF entity'
        ),
        DeclareLaunchArgument(
            'voxel_size',
            default_value='0.025',
            description='Voxel size for point cloud downsampling'
        ),
        DeclareLaunchArgument(
            'input_pointcloud_topic',
            default_value='/camera/realsense_camera/depth/color/points',
            description='Topic for input point cloud'
        ),
        DeclareLaunchArgument(
            'input_image_topic',
            default_value='/camera/realsense_camera/color/image_raw',
            description='Topic for input image'
        ),
        DeclareLaunchArgument(
            'hue_min',
            default_value='0',
            description='Minimum hue for color filtering'
        ),
        DeclareLaunchArgument(
            'hue_max',
            default_value='25',
            description='Maximum hue for color filtering'
        ),
        DeclareLaunchArgument(
            'saturation_min',
            default_value='230',
            description='Minimum saturation for color filtering'
        ),
        DeclareLaunchArgument(
            'saturation_max',
            default_value='255',
            description='Maximum saturation for color filtering'
        ),
        DeclareLaunchArgument(
            'value_min',
            default_value='110',
            description='Minimum value for color filtering'
        ),
        DeclareLaunchArgument(
            'value_max',
            default_value='240',
            description='Maximum value for color filtering'
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
                        {'camera_name': 'rs_cam_1'},
                        {'rgb_camera.enable_auto_exposure': True},
                        {'pointcloud.enable': True},
                        {'align_depth.enable': True},
                        {'align_pointcloud.enable': True},
                        {'pointcloud.ordered_pc': True},
                        # stream_filter 2 allows for colored pointclouds
                        {'pointcloud.stream_filter': 2},
                        {'rgb_camera.profile': '640,480,30'},
                        {'depth_module.enable_auto_exposure': True},
                        {'enable_sync': True},
                        {'decimation_filter.enable': True},
                        {'spatial_filter.enable': True},
                        {'temporal_filter.enable': True},
                        {'disparity_filter.enable': False},
                        {'hole_filling_filter.enable': False},
                        {'reconnect_timeout': 30.0},
                        {'enable_gyro': False},
                        {'enable_accel': False},
                        # {'enable_color': True},
                        # {'rgb_camera.color_format': 'RGB8'},
                        # {'enable_depth': True},
                        # {'enable_infra': False},
                        # {'enable_infra1': False},
                        # {'enable_infra2': False},
                        # {'depth_module.profile': '1280,720,30'},
                        # {'rgb_camera.profile': '1280,720,30'},
                        # {'depth_module.profile': '640,480,30'},
                        # {'depth_module.depth_format': 'Z16'},
                        # {'depth_module.infra_format': 'RGB8'},
                        # {'depth_module.hdr_enabled': False},
                        # {'enable_rgbd': True},
                        # {'publish_tf': True},
                        # {'tf_publish_rate': 10.0},
                        # {'colorizer.enable': True},
                        # {"serial_no": "05022207040"},
                        # {'clip_distance': 1.0},
                    ]
                ),
                ComposableNode(
                    package='icp_obj_pose',  # Your package name
                    plugin='icp_object_pose::ICPNode',
                    name='icp_node',
                    parameters=[
                        {'camera_frame': LaunchConfiguration('camera_frame')},
                        {'object_name': LaunchConfiguration('object_name')},
                        {'voxel_size': LaunchConfiguration('voxel_size')},
                        {'input_pointcloud_topic': LaunchConfiguration('input_pointcloud_topic')},
                        {'input_image_topic': LaunchConfiguration('input_image_topic')},
                        {'hue_min': LaunchConfiguration('hue_min')},
                        {'hue_max': LaunchConfiguration('hue_max')},
                        {'saturation_min': LaunchConfiguration('saturation_min')},
                        {'saturation_max': LaunchConfiguration('saturation_max')},
                        {'value_min': LaunchConfiguration('value_min')},
                        {'value_max': LaunchConfiguration('value_max')},
                        {'reference_ply_filepath': LaunchConfiguration('reference_ply_filepath')}
                    ]
                )
            ],
            output='screen',
            arguments=['--ros-args', '--log-level', 'debug'],
            # prefix=['gdb -ex run --args']
        )
    ])
