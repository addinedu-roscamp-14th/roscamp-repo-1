"""Launch physical MoveIt control and flange-referenced hand-eye calibration."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Start one serial owner, both TF trees, camera, tracker, and GUI."""
    arm_share = Path(get_package_share_directory('arm'))
    package_share = Path(
        get_package_share_directory('arm_simple_pick_place')
    )
    arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('trajectory_speed', default_value='15'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='15'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='2.5'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('joint_state_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
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
    ]
    hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'handeye_moveit_hardware.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'trajectory_speed': LaunchConfiguration('trajectory_speed'),
            'goal_correction_speed': LaunchConfiguration(
                'goal_correction_speed'
            ),
            'goal_tolerance_deg': LaunchConfiguration(
                'goal_tolerance_deg'
            ),
            'goal_timeout_sec': LaunchConfiguration('goal_timeout_sec'),
            'joint_state_rate_hz': LaunchConfiguration(
                'joint_state_rate_hz'
            ),
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    calibration = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(
                package_share
                / 'launch'
                / 'handeye_flange_charuco_calibration.launch.py'
            )
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'name': LaunchConfiguration('name'),
            'calibration_directory': LaunchConfiguration(
                'calibration_directory'
            ),
            'robot_effector_frame': 'arm/6_Link',
        }.items(),
    )
    return LaunchDescription(arguments + [hardware, calibration])
