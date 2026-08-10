"""Launch MoveIt-driven Eye-in-Hand calibration for arm2."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Run MoveIt, physical control, marker tracking and easy_handeye2."""
    arm_share = Path(get_package_share_directory('arm2'))
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'name', default_value='arm2_jetcobot_eye_in_hand'
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm2'
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('trajectory_speed', default_value='30'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='30'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='2.5'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='20.0'),
        DeclareLaunchArgument(
            'auto_rotation_delta_deg', default_value='25.0'
        ),
        DeclareLaunchArgument(
            'auto_translation_delta_m', default_value='0.10'
        ),
        DeclareLaunchArgument(
            'auto_velocity_scale', default_value='0.50'
        ),
        DeclareLaunchArgument(
            'auto_acceleration_scale', default_value='0.50'
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
    bridge = Node(
        package='arm2',
        executable='arm2_jetcobot_trajectory_bridge',
        name='arm2_jetcobot_trajectory_bridge',
        namespace='arm2',
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
            str(arm_share / 'launch' / 'arm2_gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
            'dictionary': LaunchConfiguration('dictionary'),
            'marker_frame_id': 'arm2/handeye_target',
            'secondary_marker_id': '-1',
            'use_node_time_for_pose': 'true',
        }.items(),
    )
    easy_handeye = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(handeye_share / 'launch' / 'calibrate.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
            'calibration_type': 'eye_in_hand',
            'robot_base_frame': 'arm2/base_link',
            'robot_effector_frame': 'arm2/TCP',
            'tracking_base_frame': 'arm2/gripper_camera_optical_frame',
            'tracking_marker_frame': 'arm2/handeye_target',
        }.items(),
    )
    auto_sampler = Node(
        package='arm2',
        executable='arm2_auto_handeye_sampler',
        name='arm2_auto_handeye_sampler',
        namespace='arm2',
        output='screen',
        parameters=[{
            'rotation_delta_deg': ParameterValue(
                LaunchConfiguration('auto_rotation_delta_deg'),
                value_type=float,
            ),
            'translation_delta_m': ParameterValue(
                LaunchConfiguration('auto_translation_delta_m'),
                value_type=float,
            ),
            'velocity_scale': ParameterValue(
                LaunchConfiguration('auto_velocity_scale'),
                value_type=float,
            ),
            'acceleration_scale': ParameterValue(
                LaunchConfiguration('auto_acceleration_scale'),
                value_type=float,
            ),
        }],
    )
    environment = [
        SetEnvironmentVariable(
            'EASY_HANDEYE2_CALIBRATIONS_DIRECTORY',
            LaunchConfiguration('calibration_directory'),
        ),
    ]
    return LaunchDescription(
        arguments
        + environment
        + [moveit, bridge, camera, easy_handeye, auto_sampler]
    )
