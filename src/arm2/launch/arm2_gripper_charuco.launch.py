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


def resolve_video_device(context):
    """Resolve the USB camera independently of its /dev/video number."""
    value = LaunchConfiguration('video_device').perform(context)
    if value != 'auto':
        return [SetLaunchConfiguration('resolved_video_device', value)]

    by_id = Path('/dev/v4l/by-id')
    stable = sorted(by_id.glob('*-video-index0')) if by_id.exists() else []
    if len(stable) == 1:
        return [SetLaunchConfiguration(
            'resolved_video_device', str(stable[0])
        )]

    candidates = []
    preferred = []
    for sysfs_device in sorted(Path('/sys/class/video4linux').glob('video*')):
        device = Path('/dev') / sysfs_device.name
        if not device.exists():
            continue
        try:
            index = (sysfs_device / 'index').read_text().strip()
            name = (sysfs_device / 'name').read_text().strip()
        except OSError:
            continue
        if index != '0':
            continue
        candidates.append(device)
        if 'USB 2.0 Camera' in name:
            preferred.append(device)

    selected = preferred if preferred else candidates
    if len(selected) != 1:
        devices = ', '.join(str(item) for item in selected) or 'none'
        raise RuntimeError(
            'Cannot uniquely auto-select the arm2 camera; candidates: '
            f'{devices}. Pass video_device:=/dev/videoN explicitly.'
        )
    return [SetLaunchConfiguration(
        'resolved_video_device', str(selected[0])
    )]


def generate_launch_description():
    """Create the gripper camera and ChArUco tracking graph."""
    arguments = [
        DeclareLaunchArgument('video_device', default_value='auto'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm2/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument(
            'board_frame_id', default_value='arm2/handeye_target'
        ),
        DeclareLaunchArgument('dictionary', default_value='DICT_4X4_50'),
        DeclareLaunchArgument('squares_x', default_value='11'),
        DeclareLaunchArgument('squares_y', default_value='8'),
        DeclareLaunchArgument('square_length_m', default_value='0.015'),
        DeclareLaunchArgument('marker_length_m', default_value='0.011'),
        DeclareLaunchArgument('legacy_pattern', default_value='true'),
        DeclareLaunchArgument('detection_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('opencv_num_threads', default_value='1'),
        DeclareLaunchArgument(
            'minimum_charuco_corners', default_value='24'
        ),
        DeclareLaunchArgument(
            'max_reprojection_error_px', default_value='1.0'
        ),
        DeclareLaunchArgument(
            'use_node_time_for_pose', default_value='false'
        ),
    ]

    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='arm2/gripper_camera',
        name='camera',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('resolved_video_device'),
            'image_size': [640, 480],
            'time_per_frame': [1, 10],
            'pixel_format': 'YUYV',
            'output_encoding': 'rgb8',
            # The arm2 USB camera does not expose manual exposure controls.
            # Minimize digital brightening so specular highlights on the
            # ChArUco board retain as much edge contrast as possible.
            'brightness': -20,
            'gamma': 72,
            'gain': 0,
            'backlight_compensation': 0,
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'camera_info_url': LaunchConfiguration(
                'resolved_camera_info_url'
            ),
        }],
    )
    detector = Node(
        package='arm2',
        executable='arm2_charuco_pose_publisher',
        name='arm2_charuco_pose_publisher',
        namespace='arm2',
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
            'legacy_pattern': ParameterValue(
                LaunchConfiguration('legacy_pattern'), value_type=bool
            ),
            'detection_rate_hz': ParameterValue(
                LaunchConfiguration('detection_rate_hz'), value_type=float
            ),
            'opencv_num_threads': ParameterValue(
                LaunchConfiguration('opencv_num_threads'), value_type=int
            ),
            'minimum_charuco_corners': ParameterValue(
                LaunchConfiguration('minimum_charuco_corners'),
                value_type=int,
            ),
            'max_reprojection_error_px': ParameterValue(
                LaunchConfiguration('max_reprojection_error_px'),
                value_type=float,
            ),
            'use_node_time_for_pose': ParameterValue(
                LaunchConfiguration('use_node_time_for_pose'),
                value_type=bool,
            ),
        }],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_video_device),
        OpaqueFunction(function=resolve_camera_info_url),
        camera,
        detector,
    ])
