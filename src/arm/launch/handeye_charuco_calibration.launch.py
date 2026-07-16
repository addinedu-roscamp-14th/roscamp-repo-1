"""Launch manual Eye-in-Hand calibration with a ChArUco board."""

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
    """Create the robot, ChArUco tracker and easy_handeye2 graph."""
    arm_share = Path(get_package_share_directory('arm'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument('squares_x', default_value='5'),
        DeclareLaunchArgument('squares_y', default_value='7'),
        DeclareLaunchArgument('square_length_m', default_value='0.025'),
        DeclareLaunchArgument('marker_length_m', default_value='0.018'),
        DeclareLaunchArgument(
            'minimum_charuco_corners', default_value='8'
        ),
        DeclareLaunchArgument(
            'max_reprojection_error_px', default_value='1.0'
        ),
        DeclareLaunchArgument(
            'name', default_value='jetcobot_eye_in_hand_charuco'
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument('robot_base_frame', default_value='arm/base_link'),
        DeclareLaunchArgument('robot_effector_frame', default_value='arm/TCP'),
        DeclareLaunchArgument(
            'tracking_base_frame',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'tracking_marker_frame', default_value='arm/handeye_target'
        ),
    ]
    robot_model = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            'start_hardware_joint_publisher': 'false',
        }.items(),
    )
    gripper_charuco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'gripper_charuco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('tracking_base_frame'),
            'board_frame_id': LaunchConfiguration('tracking_marker_frame'),
            'dictionary': LaunchConfiguration('dictionary'),
            'squares_x': LaunchConfiguration('squares_x'),
            'squares_y': LaunchConfiguration('squares_y'),
            'square_length_m': LaunchConfiguration('square_length_m'),
            'marker_length_m': LaunchConfiguration('marker_length_m'),
            'minimum_charuco_corners': LaunchConfiguration(
                'minimum_charuco_corners'
            ),
            'max_reprojection_error_px': LaunchConfiguration(
                'max_reprojection_error_px'
            ),
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
    use_system_opencv = SetEnvironmentVariable('PYTHONNOUSERSITE', '1')
    use_project_calibrations = SetEnvironmentVariable(
        'EASY_HANDEYE2_CALIBRATIONS_DIRECTORY',
        LaunchConfiguration('calibration_directory'),
    )
    return LaunchDescription(arguments + [
        use_system_opencv,
        use_project_calibrations,
        robot_model,
        gripper_charuco,
        easy_handeye,
    ])
