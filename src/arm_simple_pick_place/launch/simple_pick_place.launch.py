"""Launch simple pick/place with either direct or MoveIt motion."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    SetLaunchConfiguration,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def resolve_camera_info_url(context):
    """Convert a relative camera calibration path into a file URL."""
    value = LaunchConfiguration('camera_info_url').perform(context)
    if '://' not in value:
        value = Path(value).expanduser().resolve().as_uri()
    return [SetLaunchConfiguration('resolved_camera_info_url', value)]


def generate_launch_description():
    """Create camera, calibrated TF, detector, and selected motion backend."""
    package_share = Path(
        get_package_share_directory('arm_simple_pick_place')
    )
    arm_share = Path(get_package_share_directory('arm'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    is_moveit = PythonExpression([
        "'", LaunchConfiguration('motion_backend'), "' == 'moveit'"
    ])
    selected_base_frame = PythonExpression([
        "'base_link' if '", LaunchConfiguration('motion_backend'),
        "' == 'moveit' else 'arm/base_link'"
    ])
    arguments = [
        DeclareLaunchArgument('motion_backend', default_value='direct'),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument(
            'camera_frame_id',
            default_value='arm/gripper_camera_optical_frame',
        ),
        DeclareLaunchArgument('pick_marker_id', default_value='2'),
        DeclareLaunchArgument('place_marker_id', default_value='7'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
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
                package_share / 'config' / 'simple_pick_place.yaml'
            ),
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument(
            'controller_frame', default_value='arm/controller_coords'
        ),
        # Measured from three stationary poses by comparing get_coords()
        # against base_link -> 6_Link. Values are 6_Link-local.
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

    direct_robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        condition=UnlessCondition(is_moveit),
        launch_arguments={
            'start_hardware_joint_publisher': 'false',
        }.items(),
    )
    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / 'real_planning.launch.py')
        ),
        condition=IfCondition(is_moveit),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    bridge = Node(
        package='arm',
        executable='jetcobot_trajectory_bridge',
        namespace='arm',
        name='jetcobot_trajectory_bridge',
        condition=IfCondition(is_moveit),
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'joint_states_topic': '/joint_states',
        }],
    )
    # Saved flange calibration uses prefixed arm frames. These zero transforms
    # connect that calibrated tree to MoveIt's unprefixed robot model.
    base_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='moveit_base_to_calibration_base',
        condition=IfCondition(is_moveit),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'base_link',
            '--child-frame-id', 'arm/base_link',
        ],
    )
    flange_alias = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='moveit_flange_to_calibration_flange',
        condition=IfCondition(is_moveit),
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', '6_Link',
            '--child-frame-id', 'arm/6_Link',
        ],
    )
    controller_frame = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='six_link_to_controller_coords',
        arguments=[
            '--x', LaunchConfiguration('controller_x_m'),
            '--y', LaunchConfiguration('controller_y_m'),
            '--z', LaunchConfiguration('controller_z_m'),
            '--roll', LaunchConfiguration('controller_roll_rad'),
            '--pitch', LaunchConfiguration('controller_pitch_rad'),
            '--yaw', LaunchConfiguration('controller_yaw_rad'),
            '--frame-id', 'arm/6_Link',
            '--child-frame-id', LaunchConfiguration('controller_frame'),
        ],
        output='screen',
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
        package='arm_simple_pick_place',
        executable='gated_dual_aruco_pose_publisher',
        namespace='arm',
        name='simple_dual_aruco_pose_publisher',
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
            'use_node_time_for_pose': True,
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
        package='arm_simple_pick_place',
        executable='simple_pick_place',
        namespace='arm',
        name='simple_pick_place',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'motion_backend': LaunchConfiguration('motion_backend'),
                'serial_port': LaunchConfiguration('serial_port'),
                'base_frame': selected_base_frame,
                'command_frame': LaunchConfiguration('controller_frame'),
            },
        ],
    )
    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_camera_info_url),
        calibration_directory,
        direct_robot_tf,
        moveit,
        bridge,
        base_alias,
        flange_alias,
        controller_frame,
        camera,
        handeye,
        detector,
        coordinator,
    ])
