"""Launch the gripper camera and ChArUco board pose publisher."""

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
    """Create the gripper camera and ChArUco tracking graph."""
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'board_frame_id', default_value='arm/handeye_target'
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
    ]

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='arm/gripper_camera',
        name='camera',
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
        executable='charuco_pose_publisher',
        name='charuco_pose_publisher',
        namespace='arm',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'board_frame_id': LaunchConfiguration('board_frame_id'),
            'dictionary': LaunchConfiguration('dictionary'),
            'squares_x': ParameterValue(
                LaunchConfiguration('squares_x'), value_type=int
            ),
            'squares_y': ParameterValue(
                LaunchConfiguration('squares_y'), value_type=int
            ),
            'square_length_m': ParameterValue(
                LaunchConfiguration('square_length_m'), value_type=float
            ),
            'marker_length_m': ParameterValue(
                LaunchConfiguration('marker_length_m'), value_type=float
            ),
            'minimum_charuco_corners': ParameterValue(
                LaunchConfiguration('minimum_charuco_corners'),
                value_type=int,
            ),
            'max_reprojection_error_px': ParameterValue(
                LaunchConfiguration('max_reprojection_error_px'),
                value_type=float,
            ),
        }],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        camera,
        detector,
    ])
