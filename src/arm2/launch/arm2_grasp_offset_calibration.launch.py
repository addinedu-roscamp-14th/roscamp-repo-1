"""Launch arm2 TF and vision nodes for manually taught grasp offsets."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Run offset calibration without opening the robot serial port."""
    arm_share = Path(get_package_share_directory('arm2'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_gripper_camera_info.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.015'),
        DeclareLaunchArgument('dictionary', default_value='DICT_5X5_50'),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='arm2_jetcobot_eye_in_hand',
        ),
        DeclareLaunchArgument(
            'calibration_directory', default_value='config/arm2'
        ),
        DeclareLaunchArgument(
            'output_yaml',
            default_value='config/arm2/arm2_container_pick.yaml',
        ),
        DeclareLaunchArgument('max_offset_std_m', default_value='0.006'),
    ]

    robot_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm_share / 'launch' / 'arm2_robot_tf.launch.py')
        ),
        launch_arguments={
            'start_hardware_joint_publisher': 'false',
        }.items(),
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
            'secondary_marker_id': '-1',
            'use_node_time_for_pose': 'true',
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
    calibrator = Node(
        package='arm2',
        executable='arm2_grasp_offset_calibrator',
        name='arm2_grasp_offset_calibrator',
        output='screen',
        parameters=[{
            'output_yaml': LaunchConfiguration('output_yaml'),
            'max_offset_std_m': ParameterValue(
                LaunchConfiguration('max_offset_std_m'),
                value_type=float,
            ),
        }],
    )
    return LaunchDescription(
        arguments + [robot_tf, camera, handeye, calibrator]
    )
