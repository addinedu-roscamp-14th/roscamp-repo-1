"""Launch dual-ArUco physical pick and place without MoveIt."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_camera_info_url(context):
    """Convert a relative calibration YAML path to a file URL."""
    value = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in value:
        value = Path(value).expanduser().resolve().as_uri()
    return [SetLaunchConfiguration('resolved_camera_info_url', value)]


def generate_launch_description():
    """Create the existing arm stack with direct JetCobot motion."""
    arm_share = Path(get_package_share_directory('arm'))

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('pick_marker_id', default_value='0'),
        DeclareLaunchArgument('place_marker_id', default_value='7'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'use_node_time_for_pose', default_value='true'
        ),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='jetcobot_eye_in_hand_charuco_moveit',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value='config/arm/container_pick_place.yaml',
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('speed', default_value='5'),
    ]

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            # The coordinator owns the serial port and publishes the joints.
            'start_hardware_joint_publisher': 'false',
        }.items(),
    )
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
        executable='dual_aruco_pose_publisher',
        name='dual_aruco_pose_publisher',
        namespace='arm',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'pick_marker_id': ParameterValue(
                LaunchConfiguration('pick_marker_id'), value_type=int
            ),
            'place_marker_id': ParameterValue(
                LaunchConfiguration('place_marker_id'), value_type=int
            ),
            'pick_marker_frame': 'arm/pick_marker',
            'place_marker_frame': 'arm/place_marker',
            'marker_size_m': ParameterValue(
                LaunchConfiguration('marker_size_m'), value_type=float
            ),
            'dictionary': LaunchConfiguration('dictionary'),
            'use_node_time_for_pose': ParameterValue(
                LaunchConfiguration('use_node_time_for_pose'),
                value_type=bool,
            ),
        }],
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'handeye_publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
            'calibration_directory': LaunchConfiguration(
                'calibration_directory'
            ),
        }.items(),
    )
    coordinator = Node(
        package='arm',
        executable='container_pick_place_coordinator',
        name='container_pick_place_coordinator',
        namespace='arm',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'base_frame': 'arm/base_link',
                'execute_motion': True,
                'motion_backend': 'direct',
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': ParameterValue(
                    LaunchConfiguration('baud_rate'), value_type=int
                ),
                'speed': ParameterValue(
                    LaunchConfiguration('speed'), value_type=int
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        robot_tf,
        camera,
        handeye,
        TimerAction(period=1.5, actions=[detector]),
        coordinator,
    ])
