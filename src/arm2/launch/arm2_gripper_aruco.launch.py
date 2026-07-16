"""Launch the gripper camera and ArUco 6D pose publisher."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def resolve_camera_info_url(context):
    """Convert a relative camera calibration path to a file URL."""
    value = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in value:
        value = Path(value).expanduser().resolve().as_uri()
    return [SetLaunchConfiguration('resolved_camera_info_url', value)]


def generate_launch_description():
    """Create the gripper camera and marker tracking graph."""
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/arm_camera'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_arm_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm2/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'marker_frame_id', default_value='arm2/container_marker'
        ),
    ]

    camera = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'arm2', 'arm2_camera_device_supervisor',
            '--device', LaunchConfiguration('video_device'),
            '--poll-sec', '0.1',
            '--',
            'ros2', 'run', 'v4l2_camera', 'v4l2_camera_node',
            '--ros-args',
            '-r', '__ns:=/arm2/gripper_camera',
            '-r', '__node:=camera',
            '-p', ['video_device:=', LaunchConfiguration('video_device')],
            '-p', 'image_size:=[640,480]',
            '-p', 'time_per_frame:=[1,10]',
            '-p', 'pixel_format:=YUYV',
            '-p', 'output_encoding:=rgb8',
            '-p', ['camera_frame_id:=', LaunchConfiguration('camera_frame_id')],
            '-p', [
                'camera_info_url:=',
                LaunchConfiguration('resolved_camera_info_url'),
            ],
        ],
        output='screen',
    )

    detector = Node(
        package='arm2',
        executable='arm2_aruco_pose_publisher',
        name='aruco_pose_publisher',
        namespace='arm2',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'marker_frame_id': LaunchConfiguration('marker_frame_id'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'dictionary': LaunchConfiguration('dictionary'),
        }],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        camera,
        detector,
    ])
