from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='v4l2_camera',
            executable='calibration_node',
            name='calibration_node',
            parameters=[
                {'pattern_type': 'charuco'},  # or 'chessboard'
                {'chessboard_rows': 6},
                {'chessboard_cols': 9},
                {'square_size': 0.025},
                {'charuco_dict': 'DICT_5X5_1000'},
                {'charuco_squares_x': 5},
                {'charuco_squares_y': 7},
                {'charuco_square_length': 0.04},
                {'charuco_marker_length': 0.02},
                {'capture_topic': '/image_raw'},
                {'output_file': 'calibration.yaml'}
            ],
            output='screen'
        )
    ])
