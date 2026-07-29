"""Launch the top-down annotated-image FastAPI stream."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Configure the stream node from a workspace-relative YAML file."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='config/dashboard/dashboard.yaml',
        ),
        DeclareLaunchArgument('slam_map_topic', default_value='/map'),
        DeclareLaunchArgument('slam_scan_topic', default_value='/scan'),
        DeclareLaunchArgument('slam_pose_topic', default_value=''),
        DeclareLaunchArgument('slam_enable_scan', default_value='true'),
        DeclareLaunchArgument(
            'slam_base_frame',
            default_value='base_footprint',
        ),
        Node(
            package='dashboard',
            executable='dashboard_stream_node',
            name='dashboard_stream_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'slam_map_topic': ParameterValue(
                        LaunchConfiguration('slam_map_topic'),
                        value_type=str,
                    ),
                    'slam_scan_topic': ParameterValue(
                        LaunchConfiguration('slam_scan_topic'),
                        value_type=str,
                    ),
                    'slam_pose_topic': ParameterValue(
                        LaunchConfiguration('slam_pose_topic'),
                        value_type=str,
                    ),
                    'slam_enable_scan': ParameterValue(
                        LaunchConfiguration('slam_enable_scan'),
                        value_type=bool,
                    ),
                    'slam_base_frame': ParameterValue(
                        LaunchConfiguration('slam_base_frame'),
                        value_type=str,
                    ),
                },
            ],
        ),
    ])
