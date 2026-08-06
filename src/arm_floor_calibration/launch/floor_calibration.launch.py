"""Launch camera/TF/ArUco support and the non-moving calibration collector."""

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


def resolve_paths(context):
    """Resolve user paths before ROS nodes consume launch substitutions."""
    camera_url = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in camera_url:
        camera_url = Path(camera_url).expanduser().resolve().as_uri()
    output_file = Path(
        LaunchConfiguration('output_file').perform(context)
    ).expanduser().resolve()
    return [
        SetLaunchConfiguration('resolved_camera_info_url', camera_url),
        SetLaunchConfiguration('resolved_output_file', str(output_file)),
    ]


def generate_launch_description():
    """Create the non-moving floor calibration launch graph."""
    package_share = Path(
        get_package_share_directory('arm_floor_calibration')
    )
    arm_share = Path(get_package_share_directory('arm'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
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
        DeclareLaunchArgument('target_id', default_value='2'),
        DeclareLaunchArgument('marker_size_m', default_value='0.020'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='jetcobot_eye_in_hand_charuco_flange',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                package_share / 'config' / 'floor_calibration.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'output_file',
            default_value='calibration_results/floor_calibration.yaml',
        ),
        DeclareLaunchArgument('controller_x_m', default_value='0.0184'),
        DeclareLaunchArgument('controller_y_m', default_value='0.0000'),
        DeclareLaunchArgument('controller_z_m', default_value='-0.0019'),
        DeclareLaunchArgument(
            'controller_roll_rad', default_value='-1.5707963'
        ),
        DeclareLaunchArgument(
            'controller_pitch_rad', default_value='-0.7853982'
        ),
        DeclareLaunchArgument(
            'controller_yaw_rad', default_value='-1.5707963'
        ),
    ]

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            # manual_jog owns the serial port and publishes /arm/joint_states.
            'start_hardware_joint_publisher': 'false',
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
            'name': LaunchConfiguration('calibration_name')
        }.items(),
    )
    controller_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='floor_calibration_controller_frame',
        arguments=[
            '--x', LaunchConfiguration('controller_x_m'),
            '--y', LaunchConfiguration('controller_y_m'),
            '--z', LaunchConfiguration('controller_z_m'),
            '--roll', LaunchConfiguration('controller_roll_rad'),
            '--pitch', LaunchConfiguration('controller_pitch_rad'),
            '--yaw', LaunchConfiguration('controller_yaw_rad'),
            '--frame-id', 'arm/6_Link',
            '--child-frame-id', 'arm/controller_coords',
        ],
    )
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        namespace='arm/gripper_camera',
        name='floor_calibration_camera',
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
        name='floor_calibration_aruco',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'marker_frame_id': 'arm/target_marker',
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
    calibrator = Node(
        package='arm_floor_calibration',
        executable='floor_calibrator',
        namespace='arm',
        name='floor_calibrator',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {'output_file': LaunchConfiguration('resolved_output_file')},
        ],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_paths),
        calibration_directory,
        robot_tf,
        controller_frame,
        handeye,
        camera,
        TimerAction(period=1.5, actions=[detector, calibrator]),
    ])
