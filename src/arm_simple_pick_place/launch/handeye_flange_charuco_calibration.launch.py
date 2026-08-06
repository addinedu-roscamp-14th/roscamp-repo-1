"""Calibrate the wrist camera directly against the flange link."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Use arm/6_Link instead of arm/TCP as easy_handeye2 effector."""
    arm_share = Path(get_package_share_directory('arm'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'name',
            default_value='jetcobot_eye_in_hand_charuco_flange',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument(
            'robot_effector_frame', default_value='arm/6_Link'
        ),
    ]
    calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                arm_share
                / 'launch'
                / 'handeye_charuco_calibration.launch.py'
            )
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'dictionary': 'DICT_4X4_50',
            'squares_x': '5',
            'squares_y': '5',
            'square_length_m': '0.020',
            'marker_length_m': '0.015',
            'legacy_pattern': 'true',
            'name': LaunchConfiguration('name'),
            'calibration_directory': LaunchConfiguration(
                'calibration_directory'
            ),
            'robot_base_frame': 'arm/base_link',
            'robot_effector_frame': LaunchConfiguration(
                'robot_effector_frame'
            ),
            'tracking_base_frame':
                'arm/gripper_camera_optical_frame',
            'tracking_marker_frame': 'arm/handeye_target',
        }.items(),
    )
    return LaunchDescription(arguments + [calibration])
