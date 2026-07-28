"""Central camera, API, and two-vehicle fleet dispatcher."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workspace = LaunchConfiguration('workspace')
    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'poter_ws',
            ]),
        ),
        DeclareLaunchArgument('video_device', default_value='/dev/video2'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument('start_dashboard_api', default_value='true'),
        DeclareLaunchArgument('control_host', default_value='0.0.0.0'),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('porter_bringup'),
                    'launch',
                    'central_laptop.launch.py',
                ])
            ),
            launch_arguments={
                'workspace': workspace,
                'video_device': LaunchConfiguration('video_device'),
                'start_camera': LaunchConfiguration('start_camera'),
                'start_yolo': LaunchConfiguration('start_yolo'),
                'start_dashboard_api': LaunchConfiguration(
                    'start_dashboard_api'
                ),
                'dashboard_slam_map_topic': '/agv1/map',
                'dashboard_slam_scan_topic': '/agv1/scan',
                'dashboard_slam_base_frame': 'agv1/base_footprint',
                'start_nav2': 'false',
                'start_navigation_control': 'false',
                'use_rviz': 'false',
            }.items(),
        ),
        Node(
            package='central',
            executable='fleet_dispatcher',
            name='fleet_dispatcher',
            output='screen',
        ),
        Node(
            package='central',
            executable='camera_to_map_bridge',
            name='camera_to_map_bridge',
            output='screen',
            parameters=[{
                'calibration_yaml': PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'camera_map_calibration.yaml',
                ]),
                'waypoint_mode': False,
            }],
        ),
        Node(
            package='central',
            executable='control_gateway',
            name='central_control_gateway',
            output='screen',
            parameters=[
                PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'control_gateway.yaml',
                ]),
                {
                    'host': LaunchConfiguration('control_host'),
                    'api_token': LaunchConfiguration('api_token'),
                },
            ],
        ),
    ])
