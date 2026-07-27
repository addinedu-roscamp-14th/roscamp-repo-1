"""Launch multi-layer container relocation with MoveIt and eye-in-hand ArUco."""

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
    """Create the physical relocation launch graph."""
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    relocation_share = Path(
        get_package_share_directory('arm_relocation')
    )

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
        DeclareLaunchArgument('pick_marker', default_value='1'),
        DeclareLaunchArgument('place_marker', default_value='2'),
        DeclareLaunchArgument('empty_marker', default_value='3'),
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
        DeclareLaunchArgument(
            'relocation_params_file',
            default_value=str(
                relocation_share
                / 'config'
                / 'container_pick_place_relocation.yaml'
            ),
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('trajectory_speed', default_value='70'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='30'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='3.2'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / 'real_planning.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    tcp_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='moveit_tcp_to_arm_tcp',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'TCP', '--child-frame-id', 'arm/TCP',
        ],
        output='screen',
    )
    bridge = Node(
        package='arm',
        executable='jetcobot_trajectory_bridge',
        name='jetcobot_trajectory_bridge',
        namespace='arm',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'speed': ParameterValue(
                LaunchConfiguration('trajectory_speed'), value_type=int
            ),
            'goal_tolerance_deg': ParameterValue(
                LaunchConfiguration('goal_tolerance_deg'), value_type=float
            ),
            'goal_timeout_sec': ParameterValue(
                LaunchConfiguration('goal_timeout_sec'), value_type=float
            ),
            'goal_correction_speed': ParameterValue(
                LaunchConfiguration('goal_correction_speed'), value_type=int
            ),
            'goal_correction_period_sec': 1.0,
            'gripper_speed': 50,
        }],
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
        package='arm_relocation',
        executable='relocation_aruco_pose_publisher',
        name='relocation_aruco_pose_publisher',
        namespace='arm',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
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
    coordinator = Node(
        package='arm_relocation',
        executable='container_pick_place_relocation',
        # Keep the existing node name so container_pick_place.yaml's fully
        # qualified selector (/arm/container_pick_place_coordinator) applies.
        name='container_pick_place_coordinator',
        namespace='arm',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            LaunchConfiguration('relocation_params_file'),
            {
                'base_frame': 'base_link',
                'execute_motion': True,
                'motion_backend': 'moveit',
                'pick_marker_id': ParameterValue(
                    LaunchConfiguration('pick_marker'), value_type=int
                ),
                'place_marker_id': ParameterValue(
                    LaunchConfiguration('place_marker'), value_type=int
                ),
                'empty_marker_id': ParameterValue(
                    LaunchConfiguration('empty_marker'), value_type=int
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        calibration_directory,
        moveit,
        tcp_alias,
        bridge,
        camera,
        handeye,
        TimerAction(period=1.5, actions=[detector]),
        coordinator,
    ])
