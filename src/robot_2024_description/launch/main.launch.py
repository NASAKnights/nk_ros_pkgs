from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
import launch_ros.actions
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    
    urdf_file_name = get_package_share_directory('robot_2024_description') + '/urdf/robot_2024.urdf'
    with open(urdf_file_name, 'r') as infp:
        robot_desc = infp.read()

    urdf_file_name = get_package_share_directory('robot_2024_description') + '/urdf/camera_1.urdf'
    with open(urdf_file_name, 'r') as infp:
        robot_desc_1 = infp.read()

    urdf_file_name = get_package_share_directory('robot_2024_description') + '/urdf/camera_2.urdf'
    with open(urdf_file_name, 'r') as infp:
        robot_desc_2 = infp.read()


    return LaunchDescription([

        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true'),

        launch_ros.actions.Node(
            name='robot_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            remappings=[('robot_description', '/robot_description')],
        ),
        
        launch_ros.actions.Node(
            name='robot_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc_1}],
            remappings=[('robot_description', '/camera_1/robot_description')],
        ),

        launch_ros.actions.Node(
            name='robot_publisher',
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc_2}],
            remappings=[('robot_description', '/camera_2/robot_description')],
        ),
    ])
