"""Launch MoveIt-based ArUco container picking on physical JetCobot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the physical MoveIt pick graph without fake controllers."""
    arm_share = Path(get_package_share_directory('arm'))
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    default_params = 'config/arm/container_pick.yaml'

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
        DeclareLaunchArgument(
            'calibration_name', default_value='jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('trajectory_speed', default_value='100'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='50'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='3.0'),
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
        }],
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'marker_frame_id': 'arm/container_marker',
        }.items(),
    )
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'handeye_publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
        }.items(),
    )
    coordinator = Node(
        package='arm',
        executable='container_pick_coordinator',
        name='container_pick_coordinator',
        namespace='arm',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
            {
                'base_frame': 'base_link',
                'execute_motion': True,
                'motion_backend': 'moveit',
            },
        ],
    )

    return LaunchDescription(arguments + [
        moveit,
        tcp_alias,
        bridge,
        camera,
        handeye,
        coordinator,
    ])
