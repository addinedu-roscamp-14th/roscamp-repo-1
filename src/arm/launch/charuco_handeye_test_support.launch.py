"""Launch TF and vision support for the interactive ChArUco test node."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start robot TF, saved Hand-Eye TF and the ChArUco detector."""
    arm_share = Path(get_package_share_directory('arm'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='jetcobot_eye_in_hand_charuco',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument(
            'board_frame_id', default_value='arm/charuco_test_target'
        ),
        DeclareLaunchArgument('legacy_pattern', default_value='true'),
        DeclareLaunchArgument(
            'max_reprojection_error_px', default_value='3.0'
        ),
    ]

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            'start_hardware_joint_publisher': 'false',
        }.items(),
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'handeye_publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
            'calibration_directory': LaunchConfiguration(
                'calibration_directory'
            ),
        }.items(),
    )
    charuco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'gripper_charuco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': 'arm/gripper_camera_optical_frame',
            'board_frame_id': LaunchConfiguration('board_frame_id'),
            'dictionary': 'DICT_4X4_50',
            'squares_x': '5',
            'squares_y': '5',
            'square_length_m': '0.020',
            'marker_length_m': '0.015',
            'legacy_pattern': LaunchConfiguration('legacy_pattern'),
            'detection_rate_hz': '5.0',
            'opencv_num_threads': '1',
            'minimum_charuco_corners': '6',
            'max_reprojection_error_px': LaunchConfiguration(
                'max_reprojection_error_px'
            ),
            'use_node_time_for_pose': 'false',
        }.items(),
    )
    return LaunchDescription(
        arguments + [robot_tf, handeye, charuco]
    )
