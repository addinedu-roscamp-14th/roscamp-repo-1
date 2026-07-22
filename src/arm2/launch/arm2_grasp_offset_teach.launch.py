"""Launch TF and marker tracking for manually teaching a grasp offset."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Track the container while manual_jog exclusively owns the robot."""
    arm_share = Path(get_package_share_directory('arm2'))
    arguments = [
        DeclareLaunchArgument('video_device', default_value='auto'),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value=str(
                Path.home()
                / 'poter_ws/config/arm2/arm2_gripper_camera_info.yaml'
            ),
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument(
            'calibration_name',
            default_value='arm2_jetcobot_eye_in_hand_charuco',
        ),
        DeclareLaunchArgument(
            'calibration_directory',
            default_value=str(Path.home() / 'poter_ws/config/arm2'),
        ),
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
            'marker_frame_id': 'arm2/container_marker',
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
    return LaunchDescription(arguments + [robot_tf, camera, handeye])
