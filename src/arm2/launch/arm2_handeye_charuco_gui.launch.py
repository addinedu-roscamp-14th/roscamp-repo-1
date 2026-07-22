"""Launch the laptop side of distributed ChArUco Hand-Eye calibration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start robot TF and easy_handeye2 without opening remote hardware."""
    arm_share = Path(get_package_share_directory('arm2'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    arguments = [
        DeclareLaunchArgument(
            'name', default_value='arm2_jetcobot_eye_in_hand_charuco'
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm2'
        ),
        DeclareLaunchArgument('robot_base_frame', default_value='arm2/base_link'),
        DeclareLaunchArgument('robot_effector_frame', default_value='arm2/TCP'),
        DeclareLaunchArgument(
            'tracking_base_frame',
            default_value='arm2/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='arm2/handeye_target'
        ),
    ]
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_robot_tf.launch.py')
        ),
        launch_arguments={
            'start_hardware_joint_publisher': 'false',
        }.items(),
    )
    easy_handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(handeye_share / 'launch' / 'calibrate.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': LaunchConfiguration('robot_base_frame'),
            'robot_effector_frame': LaunchConfiguration(
                'robot_effector_frame'
            ),
            'tracking_base_frame': LaunchConfiguration('tracking_base_frame'),
            'tracking_marker_frame': LaunchConfiguration(
                'tracking_marker_frame'
            ),
        }.items(),
    )
    return LaunchDescription(arguments + [
        SetEnvironmentVariable('PYTHONNOUSERSITE', '1'),
        SetEnvironmentVariable(
            'EASY_HANDEYE2_CALIBRATIONS_DIRECTORY',
            LaunchConfiguration('calibration_directory'),
        ),
        robot_model,
        easy_handeye,
    ])
