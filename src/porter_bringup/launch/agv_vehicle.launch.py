"""Hardware and namespaced Nav2 bringup for one Pinky AGV.

Each vehicle runs its own localization and Nav2 stack. The fleet laptop only
visualizes both vehicles and dispatches goals to their namespaced actions.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def _configure_discovery(context):
    server = LaunchConfiguration('discovery_server').perform(context).strip()
    if not server:
        return []
    return [SetEnvironmentVariable('ROS_DISCOVERY_SERVER', server)]


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
        DeclareLaunchArgument(
            'scan_max_age_sec',
            default_value='0.5',
            description='Maximum LaserScan age accepted by the vehicle',
        ),
        DeclareLaunchArgument(
            'discovery_server',
            default_value=EnvironmentVariable(
                'ROS_DISCOVERY_SERVER',
                default_value='',
            ),
            description='Fast DDS server, for example 10.0.0.2:11811',
        ),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument(
            'start_other_robot_obstacle',
            default_value='true',
            description='Add the peer AGV pose to this local costmap',
        ),
        DeclareLaunchArgument(
            'other_robot_obstacle_radius', default_value='0.13'
        ),
        DeclareLaunchArgument(
            'other_robot_point_spacing', default_value='0.02'
        ),
        DeclareLaunchArgument(
            'shared_pose_publish_rate', default_value='10.0'
        ),
        DeclareLaunchArgument(
            'other_robot_publish_rate', default_value='10.0'
        ),
        DeclareLaunchArgument(
            'other_robot_pose_timeout', default_value='1.0'
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='true',
            description='Run Nav2 in a component container',
        ),
        DeclareLaunchArgument(
            'nav2_start_delay',
            default_value='8.0',
            description='Wait for hardware TF and sensor topics before Nav2',
        ),
        DeclareLaunchArgument(
            'start_parking_supervisor',
            default_value='true',
            description='Run the vehicle auto-parking action server',
        ),
        DeclareLaunchArgument(
            'parking_spots_yaml',
            default_value=PathJoinSubstitution([
                FindPackageShare('drive'), 'params', 'parking_spots.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'parking_supervisor_start_delay',
            default_value='35.0',
            description='Start parking after the Nav2 component load burst',
        ),
        OpaqueFunction(function=_configure_discovery),
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
                'scan_max_age_sec': LaunchConfiguration('scan_max_age_sec'),
            }.items(),
        ),
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            condition=IfCondition(LaunchConfiguration('start_nav2')),
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(
                        PathJoinSubstitution([
                            FindPackageShare('drive'),
                            'launch',
                            'multi_vehicle_nav.launch.py',
                        ])
                    ),
                    launch_arguments={
                        'vehicle_id': vehicle_id,
                        'workspace': workspace,
                        'map': LaunchConfiguration('map'),
                        'keepout_mask': LaunchConfiguration('keepout_mask'),
                        'use_sim_time': LaunchConfiguration('use_sim_time'),
                        'use_composition': LaunchConfiguration(
                            'use_composition'
                        ),
                        'start_other_robot_obstacle': LaunchConfiguration(
                            'start_other_robot_obstacle'
                        ),
                        'other_robot_obstacle_radius': LaunchConfiguration(
                            'other_robot_obstacle_radius'
                        ),
                        'other_robot_point_spacing': LaunchConfiguration(
                            'other_robot_point_spacing'
                        ),
                        'shared_pose_publish_rate': LaunchConfiguration(
                            'shared_pose_publish_rate'
                        ),
                        'other_robot_publish_rate': LaunchConfiguration(
                            'other_robot_publish_rate'
                        ),
                        'other_robot_pose_timeout': LaunchConfiguration(
                            'other_robot_pose_timeout'
                        ),
                        # The top-level vehicle launch starts parking_new
                        # after Nav2 has finished loading on the Raspberry Pi.
                        'start_parking_supervisor': 'false',
                    }.items(),
                ),
            ],
        ),
        TimerAction(
            period=LaunchConfiguration('parking_supervisor_start_delay'),
            condition=IfCondition(
                LaunchConfiguration('start_parking_supervisor')
            ),
            actions=[
                Node(
                    package='drive',
                    executable='parking_new',
                    name='parking_supervisor',
                    namespace=vehicle_id,
                    output='screen',
                    parameters=[{
                        'parking_spots_yaml': LaunchConfiguration(
                            'parking_spots_yaml'
                        ),
                        'cmd_vel_topic': 'cmd_vel_parking',
                    }],
                ),
            ],
        ),
    ])
