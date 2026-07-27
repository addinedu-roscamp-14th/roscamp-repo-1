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
    arm_share = Path(get_package_share_directory('arm2'))
    arguments = [
        DeclareLaunchArgument(
            'calibration_name', default_value='arm2_jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm2'
        ),
        DeclareLaunchArgument(
            'params_file', default_value='config/arm2/arm2_container_pick.yaml'
        ),
        DeclareLaunchArgument(
            'stack_container_height_m', default_value='0.035'
        ),
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
    handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_handeye_publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('calibration_name'),
            'calibration_directory': LaunchConfiguration(
                'calibration_directory'
            ),
        }.items(),
    )
    coordinator = Node(
        package='arm2',
        executable='arm2_container_pick_coordinator',
        name='arm2_container_pick_coordinator',
        namespace='arm2',
        output='screen',
        parameters=[
            LaunchConfiguration('params_file'),
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
        moveit,
        handeye,
        coordinator,
    ])
