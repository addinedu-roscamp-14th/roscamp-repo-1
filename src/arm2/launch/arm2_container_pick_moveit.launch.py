"""Launch MoveIt-based ArUco container picking on physical JetCobot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetLaunchConfiguration,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def prefer_stable_usb_names(context):
    """Replace volatile legacy device paths with installed udev aliases."""
    actions = []
    video_device = LaunchConfiguration('video_device').perform(context)
    if video_device.startswith('/dev/video') and Path('/dev/arm_camera').exists():
        actions.append(SetLaunchConfiguration('video_device', '/dev/arm_camera'))
    serial_port = LaunchConfiguration('serial_port').perform(context)
    if serial_port.startswith('/dev/ttyUSB') and Path('/dev/jetcobot').exists():
        actions.append(SetLaunchConfiguration('serial_port', '/dev/jetcobot'))
    camera_info = LaunchConfiguration('camera_info_url').perform(context)
    if camera_info == 'config/arm2/arm2_arm_camera_info.yaml':
        actions.append(SetLaunchConfiguration(
            'camera_info_url', 'config/arm2/arm2_arm_camera_info.yaml'
        ))
    marker_size = LaunchConfiguration('marker_size_m').perform(context)
    if marker_size in ('0.015', '0.02', '0.025'):
        actions.append(SetLaunchConfiguration('marker_size_m', '0.026'))
    return actions


def generate_launch_description():
    """Create the physical MoveIt pick graph without fake controllers."""
    arm_share = Path(get_package_share_directory('arm2'))
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    default_params = 'config/arm2/arm2_container_pick.yaml'

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/arm_camera'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_arm_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument(
            'calibration_name', default_value='arm2_jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument(
            'startup_pose_file',
            default_value='config/arm2/arm2_startup_pose.yaml',
        ),
        DeclareLaunchArgument(
            'grasp_calibration_file',
            default_value='config/arm2/arm2_container_grasp_teach.yaml',
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/jetcobot'),
        DeclareLaunchArgument('trajectory_speed', default_value='100'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='50'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='3.0'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('max_start_error_deg', default_value='45.0'),
        DeclareLaunchArgument('startup_move_enabled', default_value='true'),
        DeclareLaunchArgument('startup_speed', default_value='100'),
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
            '--frame-id', 'TCP', '--child-frame-id', 'arm2/TCP',
        ],
        output='screen',
    )
    bridge = Node(
        package='arm2',
        executable='arm2_jetcobot_trajectory_bridge',
        name='jetcobot_trajectory_bridge',
        namespace='arm2',
        output='screen',
        parameters=[LaunchConfiguration('startup_pose_file'), {
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
            'max_start_error_deg': ParameterValue(
                LaunchConfiguration('max_start_error_deg'), value_type=float
            ),
            'goal_correction_speed': ParameterValue(
                LaunchConfiguration('goal_correction_speed'), value_type=int
            ),
            'goal_correction_period_sec': 1.0,
            'startup_move_enabled': ParameterValue(
                LaunchConfiguration('startup_move_enabled'), value_type=bool
            ),
            'startup_speed': ParameterValue(
                LaunchConfiguration('startup_speed'), value_type=int
            ),
            'startup_tolerance_deg': ParameterValue(
                LaunchConfiguration('goal_tolerance_deg'), value_type=float
            ),
            'startup_timeout_sec': 20.0,
        }],
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'marker_frame_id': 'arm2/container_marker',
        }.items(),
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_handeye_publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
    )
    coordinator = Node(
        package='arm2',
        executable='arm2_container_pick_coordinator',
        name='container_pick_coordinator',
        namespace='arm2',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            LaunchConfiguration('grasp_calibration_file'),
            LaunchConfiguration('startup_pose_file'),
            {
                'base_frame': 'base_link',
                'execute_motion': True,
                'motion_backend': 'moveit',
            },
        ],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=prefer_stable_usb_names),
        moveit,
        tcp_alias,
        bridge,
        camera,
        handeye,
        coordinator,
    ])
