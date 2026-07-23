"""Launch laptop-side MoveIt planning for a remote physical JetCobot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start planning without opening a local serial or video device."""
    arm_share = Path(get_package_share_directory('arm'))
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    arguments = [
        DeclareLaunchArgument(
            'calibration_name', default_value='jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm'
        ),
        DeclareLaunchArgument(
            'params_file', default_value='config/arm/container_pick.yaml'
        ),
        DeclareLaunchArgument(
            'stack_container_height_m', default_value='0.035'
        ),
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
                'stack_container_height_m': ParameterValue(
                    LaunchConfiguration('stack_container_height_m'),
                    value_type=float,
                ),
            },
        ],
    )
    return LaunchDescription(arguments + [
        moveit,
        tcp_alias,
        handeye,
        coordinator,
    ])
