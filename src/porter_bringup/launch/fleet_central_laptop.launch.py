"""Central camera, API, and two-vehicle fleet dispatcher."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
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
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('control_host', default_value='0.0.0.0'),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        GroupAction(
            scoped=True,
            actions=[
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
            ],
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('drive'),
                    'launch',
                    'multi_vehicle_nav.launch.py',
                ])
            ),
            launch_arguments={
                'vehicle_id': 'agv1',
                'workspace': workspace,
                'use_composition': 'False',
                'start_navigation': 'False',
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('drive'),
                    'launch',
                    'multi_vehicle_nav.launch.py',
                ])
            ),
            launch_arguments={
                'vehicle_id': 'agv2',
                'workspace': workspace,
                'use_composition': 'False',
                'start_navigation': 'False',
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
        Node(
            package='rviz2',
            executable='rviz2',
            name='fleet_rviz',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_rviz')),
            arguments=[
                '-d',
                PathJoinSubstitution([
                    FindPackageShare('porter_bringup'),
                    'rviz',
                    'fleet_nav.rviz',
                ]),
            ],
        ),
    ])
