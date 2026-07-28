"""Hardware bringup (and optionally Nav2) for one Pinky AGV.

Nav2 (map_server/amcl) now runs centrally on the fleet laptop via
fleet_central_laptop.launch.py, so 'start_nav2' defaults to false to avoid
launching duplicate/conflicting map_server, amcl, etc. on the vehicle itself.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    workspace = LaunchConfiguration('workspace')
    vehicle_id = LaunchConfiguration('vehicle_id')
    return LaunchDescription([
        DeclareLaunchArgument(
            'vehicle_id',
            description='Vehicle ID: agv1 or agv2',
            choices=['agv1', 'agv2'],
        ),
        DeclareLaunchArgument(
            'workspace',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'), 'poter_ws',
            ]),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                workspace, 'config', 'SLAM', 'current_map.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=PathJoinSubstitution([
                workspace, 'config', 'SLAM', 'keepout_mask.yaml',
            ]),
        ),
        DeclareLaunchArgument('lidar_serial_port', default_value='/dev/ttyS0'),
        DeclareLaunchArgument(
            'motor_serial_port', default_value='/dev/ttyAMA5'
        ),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_nav2', default_value='false'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('pinky'),
                    'launch',
                    'multi_vehicle_bringup.launch.py',
                ])
            ),
            launch_arguments={
                'vehicle_id': vehicle_id,
                'lidar_serial_port': LaunchConfiguration('lidar_serial_port'),
                'motor_serial_port': LaunchConfiguration('motor_serial_port'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
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
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            launch_arguments={
                'vehicle_id': vehicle_id,
                'workspace': workspace,
                'map': LaunchConfiguration('map'),
                'keepout_mask': LaunchConfiguration('keepout_mask'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }.items(),
        ),
    ])
