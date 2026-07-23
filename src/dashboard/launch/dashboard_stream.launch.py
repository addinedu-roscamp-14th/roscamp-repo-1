"""Launch the top-down annotated-image FastAPI stream."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Configure the stream node from a workspace-relative YAML file."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='config/dashboard/dashboard.yaml',
        ),
        Node(
            package='dashboard',
            executable='dashboard_stream_node',
            name='dashboard_stream_node',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
