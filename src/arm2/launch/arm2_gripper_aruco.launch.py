"""Launch the gripper camera and ArUco 6D pose publisher."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


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
            default_value='config/arm2/arm2_gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm2/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('secondary_marker_id', default_value='-1'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'marker_frame_id', default_value='arm2/container_marker'
        ),
        DeclareLaunchArgument(
            'secondary_marker_frame_id',
            default_value='arm2/stack_target_marker',
        ),
        DeclareLaunchArgument(
            'use_node_time_for_pose', default_value='true'
        ),
    ]

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='arm2/gripper_camera',
        name='camera',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
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
        package='arm2',
        executable='arm2_aruco_pose_publisher',
        name='arm2_aruco_pose_publisher',
        namespace='arm2',
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'marker_frame_id': LaunchConfiguration('marker_frame_id'),
            'marker_id': LaunchConfiguration('marker_id'),
            'secondary_marker_id': LaunchConfiguration(
                'secondary_marker_id'
            ),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'dictionary': LaunchConfiguration('dictionary'),
            'secondary_marker_frame_id': LaunchConfiguration(
                'secondary_marker_frame_id'
            ),
            'use_node_time_for_pose': ParameterValue(
                LaunchConfiguration('use_node_time_for_pose'),
                value_type=bool,
            ),
        }],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        camera,
        detector,
    ])
