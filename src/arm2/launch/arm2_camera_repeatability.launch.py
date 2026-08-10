"""Launch the arm2 camera, ArUco detector, and repeatability monitor."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Create a camera-only repeatability test graph."""
    arm2_share = Path(get_package_share_directory('arm2'))
    arguments = [
        DeclareLaunchArgument(
            'video_device', default_value='/dev/arm_camera'
        ),
        DeclareLaunchArgument(
            'camera_info_url',
            default_value='config/arm2/arm2_gripper_camera_info_v3.yaml',
        ),
        DeclareLaunchArgument('marker_id', default_value='0'),
        DeclareLaunchArgument('marker_size_m', default_value='0.026'),
        DeclareLaunchArgument('marker_size_mm', default_value='26.0'),
        DeclareLaunchArgument('sample_count', default_value='200'),
        DeclareLaunchArgument('reference_u_px', default_value='300.350'),
        DeclareLaunchArgument('reference_v_px', default_value='293.686'),
    ]
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(arm2_share / 'launch' / 'arm2_gripper_aruco.launch.py')
        ),
        launch_arguments={
            'video_device': LaunchConfiguration('video_device'),
            'camera_info_url': LaunchConfiguration('camera_info_url'),
            'marker_id': LaunchConfiguration('marker_id'),
            'marker_size_m': LaunchConfiguration('marker_size_m'),
        }.items(),
    )
    monitor = Node(
        package='arm2',
        executable='arm2_camera_repeatability_monitor',
        namespace='arm2',
        output='screen',
        parameters=[{
            'marker_id': ParameterValue(
                LaunchConfiguration('marker_id'), value_type=int
            ),
            'marker_size_mm': ParameterValue(
                LaunchConfiguration('marker_size_mm'), value_type=float
            ),
            'sample_count': ParameterValue(
                LaunchConfiguration('sample_count'), value_type=int
            ),
            'reference_u_px': ParameterValue(
                LaunchConfiguration('reference_u_px'), value_type=float
            ),
            'reference_v_px': ParameterValue(
                LaunchConfiguration('reference_v_px'), value_type=float
            ),
        }],
        remappings=[],
    )
    return LaunchDescription(arguments + [camera, monitor])
