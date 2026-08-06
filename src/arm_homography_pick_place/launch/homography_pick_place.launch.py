"""Launch direct homography Pick/Place without MoveIt."""

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
    """Resolve camera, hand-eye and calibration paths before node startup."""
    camera_url = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in camera_url:
        camera_url = Path(camera_url).expanduser().resolve().as_uri()
    calibration_file = Path(
        LaunchConfiguration('calibration_file').perform(context)
    ).expanduser().resolve()
    return [
        SetLaunchConfiguration('resolved_camera_info_url', camera_url),
        SetLaunchConfiguration(
            'resolved_calibration_file', str(calibration_file)
        ),
    ]


def generate_launch_description():
    """Create camera, TF, gated detector, and direct coordinator graph."""
    package_share = Path(
        get_package_share_directory('arm_homography_pick_place')
    )
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
        DeclareLaunchArgument('pick_id', default_value='2'),
        DeclareLaunchArgument('place_id', default_value='8'),
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
            'calibration_file',
            default_value='calibration_results/floor_calibration.yaml',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=str(
                package_share / 'config' / 'homography_pick_place.yaml'
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
    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
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
        name='homography_pick_place_controller_frame',
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
        name='homography_pick_place_camera',
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
        package='arm_homography_pick_place',
        executable='gated_pick_place_aruco',
        namespace='arm',
        name='homography_pick_place_aruco',
        output='screen',
        parameters=[{
            'camera_frame_id': LaunchConfiguration('camera_frame_id'),
            'pick_marker_id': ParameterValue(
                LaunchConfiguration('pick_id'), value_type=int
            ),
            'place_marker_id': ParameterValue(
                LaunchConfiguration('place_id'), value_type=int
            ),
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
        package='arm_homography_pick_place',
        executable='homography_pick_place',
        namespace='arm',
        name='homography_pick_place',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'serial_port': LaunchConfiguration('serial_port'),
                'pick_marker_id': ParameterValue(
                    LaunchConfiguration('pick_id'), value_type=int
                ),
                'place_marker_id': ParameterValue(
                    LaunchConfiguration('place_id'), value_type=int
                ),
                'calibration_file': LaunchConfiguration(
                    'resolved_calibration_file'
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_paths),
        calibration_directory,
        robot_tf,
        controller_frame,
        handeye,
        camera,
        TimerAction(period=1.5, actions=[detector, coordinator]),
    ])
