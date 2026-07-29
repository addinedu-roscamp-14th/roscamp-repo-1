"""Launch container tracking, target generation, and guarded picking."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create the complete container pick graph."""
    arm_share = Path(get_package_share_directory('arm'))
    default_params = 'config/arm/container_pick.yaml'

    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video4'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm/gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.020'),
        DeclareLaunchArgument('calibration_name', default_value='jetcobot_eye_in_hand'),
        DeclareLaunchArgument('params_file', default_value=default_params),
        DeclareLaunchArgument('execute_motion', default_value='false'),
    ]

    hardware_joint_publisher = PythonExpression([
        "'", LaunchConfiguration('execute_motion'), "'.lower() not in ('true', '1')"
    ])
    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'robot_tf.launch.py')
        ),
        launch_arguments={
            'start_hardware_joint_publisher': hardware_joint_publisher,
        }.items(),
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
                'execute_motion': ParameterValue(
                    LaunchConfiguration('execute_motion'), value_type=bool
                )
            },
        ],
    )

    return LaunchDescription(
        arguments + [robot_tf, camera, handeye, coordinator]
    )
