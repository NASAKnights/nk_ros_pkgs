from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    return LaunchDescription([
        # Declare launch arguments
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
        
        # Node action for icp_node
        Node(
            package='icp_obj_pose',
            executable='icp_obj_pose',
            name='icp_node',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'reference_ply_filepath': LaunchConfiguration('reference_ply_filepath')
            }],
            remappings=[('/input_pointcloud', LaunchConfiguration('input_topic'))]
        ),
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            emulate_tty=True,
            name=LaunchConfiguration('rs_cam_1' ),
            parameters=[
                {'camera_name':'rs_cam_1'},
                # ("serial_no", '036222071448\'"},
                {'enable_color':'true'},
                {'rgb_camera.color_format':'RGB8'},
                {'rgb_camera.enable_auto_exposure':'true'},
                {'enable_depth':'true'},
                {'enable_infra':'false'},
                {'enable_infra1':'false'},
                {'enable_infra2':'false'},
                {'depth_module.depth_profile':'640,480,15'},
                {'depth_module.depth_format': 'Z16'},
                {'depth_module.infra_format':'RGB8'},
                {'depth_module.hdr_enabled':'false'},
                {'depth_module.enable_auto_exposure':'true'},
                {'enable_sync':'true'},
                {'enable_rgbd':'true'},
                # {'enable_gyro'},
                # {'enable_accel'},
                # {'clip_distance'},
                {'publish_tf':'true'},
                {'tf_publish_rate':'10.0'},
                {'pointcloud.enable':'true'},
                {'align_depth.enable':'true'},
                {'decimation_filter.enable':'false'},
                {'spatial_filter.enable':'true'},
                {'temporal_filter.enable':'true'},
                {'disparity_filter.enable':'false'},
                {'hole_filling_filter.enable':'false'},
                {'reconnect_timeout':'30.0'}
            ]
        )
    ])