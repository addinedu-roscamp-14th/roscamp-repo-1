"""Launch direct container Pick/Place without MoveIt."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    OpaqueFunction,
    SetLaunchConfiguration,
    TimerAction,
)
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_runtime(context):
    """Resolve local files and create the calibrated camera transform."""
    camera_url = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in camera_url:
        camera_url = Path(camera_url).expanduser().resolve().as_uri()
    calibration_file = Path(
        LaunchConfiguration('calibration_file').perform(context)
    ).expanduser().resolve()
    handeye_file = Path(
        LaunchConfiguration('handeye_calibration_file').perform(context)
    ).expanduser().resolve()
    with handeye_file.open(encoding='utf-8') as stream:
        handeye = yaml.safe_load(stream)
    parameters = handeye['parameters']
    transform = handeye['transform']
    translation = transform['translation']
    rotation = transform['rotation']
    if parameters['calibration_type'] != 'eye_in_hand':
        raise ValueError('Only eye_in_hand calibration is supported')
    handeye_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pick_place_handeye_transform',
        arguments=[
            '--x', str(translation['x']),
            '--y', str(translation['y']),
            '--z', str(translation['z']),
            '--qx', str(rotation['x']),
            '--qy', str(rotation['y']),
            '--qz', str(rotation['z']),
            '--qw', str(rotation['w']),
            '--frame-id', parameters['robot_effector_frame'],
            '--child-frame-id', parameters['tracking_base_frame'],
        ],
    )
    return [
        SetLaunchConfiguration('resolved_camera_info_url', camera_url),
        SetLaunchConfiguration(
            'resolved_calibration_file', str(calibration_file)
        ),
        handeye_node,
    ]


def generate_launch_description():
    """Create camera, TF, gated detector, and direct coordinator graph."""
    package_share = Path(
        get_package_share_directory('arm_pick_place')
    )
    robot_description = (
        package_share / 'urdf' / 'jetcobot_kinematics.urdf'
    ).read_text(encoding='utf-8')
    arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=str(
                package_share / 'config' / 'gripper_camera_info.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('marker_size_m', default_value='0.020'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'handeye_calibration_file',
            default_value=str(
                package_share
                / 'config'
                / 'jetcobot_eye_in_hand_charuco_flange.calib'
            ),
        ),
        DeclareLaunchArgument(
            'calibration_file',
            default_value=str(
                package_share / 'config' / 'floor_calibration.yaml'
            ),
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                package_share / 'config' / 'container_pick_place.yaml'
            ),
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

    # The coordinator owns /dev/ttyUSB0 and publishes measured joints.
    robot_tf = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='arm',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': 'arm/',
        }],
        remappings=[('joint_states', '/arm/joint_states')],
    )
    controller_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='pick_place_controller_frame',
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
        name='pick_place_camera',
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
        package='arm_pick_place',
        executable='gated_pick_place_aruco',
        namespace='arm',
        name='pick_place_aruco',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'pick_marker_id': 2,
            'place_marker_id': 8,
            'pick_marker_frame': 'arm/pick_marker',
            'place_marker_frame': 'arm/place_marker',
            'marker_size_m': ParameterValue(
                LaunchConfiguration('marker_size_m'), value_type=float
            ),
            'dictionary': LaunchConfiguration('dictionary'),
            'use_node_time_for_pose': True,
        }],
    )
    coordinator = Node(
        package='arm_pick_place',
        executable='container_pick_place',
        namespace='arm',
        name='pick_place',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'calibration_file': LaunchConfiguration(
                    'resolved_calibration_file'
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_runtime),
        robot_tf,
        controller_frame,
        camera,
        TimerAction(period=1.5, actions=[detector, coordinator]),
    ])
