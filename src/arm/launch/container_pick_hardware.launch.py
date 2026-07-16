"""Launch remote JetCobot hardware and gripper-camera marker tracking."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Expose Raspberry Pi camera and robot ports as launch arguments."""
    arm_share = Path(get_package_share_directory('arm'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('trajectory_speed', default_value='100'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='100'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='2.5'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument(
            'use_node_time_for_pose', default_value='true'
        ),
    ]
    bridge = Node(
        package='arm',
        executable='jetcobot_trajectory_bridge',
        name='jetcobot_trajectory_bridge',
        namespace='arm',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': ParameterValue(
                LaunchConfiguration('baud_rate'), value_type=int
            ),
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
            'use_node_time_for_pose': LaunchConfiguration(
                'use_node_time_for_pose'
            ),
        }.items(),
    )
    return LaunchDescription(arguments + [bridge, camera])
