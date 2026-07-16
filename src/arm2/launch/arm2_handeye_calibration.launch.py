"""Launch JetCobot model TF, gripper ArUco tracking and easy_handeye2."""

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
    """Create the manual Eye-in-Hand calibration graph."""
    arm_share = Path(get_package_share_directory('arm2'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/arm_camera'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_arm_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument('name', default_value='arm2_jetcobot_eye_in_hand'),
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
    gripper_aruco = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'camera_frame_id': LaunchConfiguration('tracking_base_frame'),
            'marker_frame_id': LaunchConfiguration('tracking_marker_frame'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'dictionary': LaunchConfiguration('dictionary'),
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

    use_system_opencv = SetEnvironmentVariable(
        'PYTHONNOUSERSITE', '1'
    )

    return LaunchDescription(
        arguments + [
            use_system_opencv,
            robot_model,
            gripper_aruco,
            easy_handeye,
        ]
    )
