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


def resolve_params_file(context):
    """Resolve and validate the coordinator YAML before nodes are started."""
    configured = LaunchConfiguration('params_file').perform(context)
    path = Path(configured).expanduser()
    if not path.is_absolute():
        path = (Path.cwd() / path).resolve()
    if not path.is_file():
        raise FileNotFoundError(f'arm2 params file not found: {path}')
    return [SetLaunchConfiguration('resolved_params_file', str(path))]


def generate_launch_description():
    """Create the physical MoveIt pick graph without fake controllers."""
    arm_share = Path(get_package_share_directory('arm2'))
    default_params = 'config/arm2/arm2_container_pick.yaml'

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/arm_camera'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('stack_marker_id', default_value='11'),
        DeclareLaunchArgument('marker_size_m', default_value='0.020'),
        DeclareLaunchArgument(
            'stack_container_height_m', default_value='0.035'
        ),
        DeclareLaunchArgument(
            'use_node_time_for_pose', default_value='true'
        ),
        DeclareLaunchArgument(
            'calibration_name', default_value='arm2_jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('serial_port', default_value='/dev/jetcobot'),
        DeclareLaunchArgument('trajectory_speed', default_value='100'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='50'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='2.5'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_moveit.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    bridge = Node(
        package='arm2',
        executable='arm2_jetcobot_trajectory_bridge',
        name='arm2_jetcobot_trajectory_bridge',
        namespace='arm2',
        output='screen',
        sigterm_timeout='20.0',
        sigkill_timeout='5.0',
        parameters=[
            LaunchConfiguration('resolved_params_file'),
            {
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
                    LaunchConfiguration('goal_correction_speed'),
                    value_type=int,
                ),
                'goal_correction_period_sec': 1.0,
                'gripper_speed': 50,
            },
        ],
    )
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'marker_id': LaunchConfiguration('marker_id'),
            'secondary_marker_id': LaunchConfiguration('stack_marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'marker_frame_id': 'arm2/container_marker',
            'use_node_time_for_pose': LaunchConfiguration(
                'use_node_time_for_pose'
            ),
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
        name='arm2_container_pick_coordinator',
        namespace='arm2',
        output='screen',
        parameters=[
            LaunchConfiguration('resolved_params_file'),
            {
                'base_frame': 'arm2/base_link',
                'moveit_ee_link': 'arm2/TCP',
                'execute_motion': True,
                'motion_backend': 'moveit',
                'stack_container_height_m': ParameterValue(
                    LaunchConfiguration('stack_container_height_m'),
                    value_type=float,
                ),
            },
        ],
    )

    return LaunchDescription(arguments + [
        OpaqueFunction(function=resolve_params_file),
        moveit,
        bridge,
        camera,
        handeye,
        coordinator,
    ])
