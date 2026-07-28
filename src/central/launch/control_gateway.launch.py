"""Launch the HTTP-to-ROS central-control gateway."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Configure the gateway from a workspace-relative YAML file."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value='config/central/control_gateway.yaml',
        ),
        DeclareLaunchArgument(
            'host',
            default_value='127.0.0.1',
        ),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        Node(
            package='central',
            executable='control_gateway',
            name='central_control_gateway',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'host': LaunchConfiguration('host'),
                    'api_token': LaunchConfiguration('api_token'),
                },
            ],
        ),
    ])
