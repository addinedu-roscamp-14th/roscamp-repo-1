"""Continuously publish one ArUco target TF without robot manipulation."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_camera_info_url(context):
    """Convert a relative camera calibration path into a file URL."""
    value = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in value:
        value = Path(value).expanduser().resolve().as_uri()
    return [SetLaunchConfiguration('resolved_camera_info_url', value)]


def generate_launch_description():
    """Start robot TF, hand-eye TF, camera, and an always-on detector."""
    arm_share = Path(get_package_share_directory('arm'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))

    arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('target_id', default_value='2'),
        DeclareLaunchArgument('marker_size_m', default_value='0.020'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'target_frame_id', default_value='arm/target_marker'
        ),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='jetcobot_eye_in_hand_charuco_flange',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
    ]

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'start_hardware_joint_publisher': 'true',
        }.items(),
    )
    calibration_directory = SetEnvironmentVariable(
        'EASY_HANDEYE2_CALIBRATIONS_DIRECTORY',
        LaunchConfiguration('calibration_directory'),
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(handeye_share / 'launch' / 'publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
    )
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='arm/gripper_camera',
        name='continuous_tf_camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('video_device'),
            'image_size': [640, 480],
            'time_per_frame': [1, 10],
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'camera_info_url': LaunchConfiguration(
                'resolved_camera_info_url'
            ),
        }],
    )
    detector = Node(
        package='arm',
        executable='aruco_pose_publisher',
        namespace='arm',
        name='continuous_target_aruco_pose_publisher',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'marker_frame_id': LaunchConfiguration('target_frame_id'),
            'marker_id': ParameterValue(
                LaunchConfiguration('target_id'), value_type=int
            ),
            'marker_size_m': ParameterValue(
                LaunchConfiguration('marker_size_m'), value_type=float
            ),
            'dictionary': LaunchConfiguration('dictionary'),
            'use_node_time_for_pose': True,
        }],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        calibration_directory,
        robot_tf,
        handeye,
        camera,
        TimerAction(period=1.5, actions=[detector]),
    ])
