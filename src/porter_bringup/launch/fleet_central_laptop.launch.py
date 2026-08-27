"""Central camera, API, and two-vehicle fleet dispatcher."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _configure_discovery(context):
    enabled = LaunchConfiguration('start_discovery_server').perform(context)
    if enabled.strip().lower() in ('true', '1', 'yes', 'on'):
        port = LaunchConfiguration('discovery_server_port').perform(context)
        return [
            SetEnvironmentVariable(
                'ROS_DISCOVERY_SERVER',
                f'127.0.0.1:{port}',
            )
        ]
    return [UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER')]


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
        DeclareLaunchArgument(
            'start_collision_supervisor',
            default_value='true',
            description='Enable top-down YOLO inter-vehicle collision holds',
        ),
        DeclareLaunchArgument('start_dashboard_api', default_value='true'),
        DeclareLaunchArgument(
            'start_arm_dispatcher',
            default_value='true',
            description='Start the central ARM1/ARM2 command dispatcher',
        ),
        DeclareLaunchArgument(
            'start_port_autonomy',
            default_value='true',
            description='Enable vessel ROI events and autonomous ARM2 scan',
        ),
        DeclareLaunchArgument(
            'dashboard_enable_scan',
            default_value='false',
            description='Stream AGV1 LaserScan to the central dashboard',
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('control_host', default_value='0.0.0.0'),
        DeclareLaunchArgument(
            'b1_waiting_distance_m',
            default_value='0.25',
            description='Distance behind B-1 used by a queued vehicle',
        ),
        DeclareLaunchArgument(
            'b1_waiting_camera_down_offset_m',
            default_value='0.11',
            description='Additional camera-down offset for the B-1 waiting pose',
        ),
        DeclareLaunchArgument(
            'a_zone_waiting_distance_m',
            default_value='0.20',
            description='Distance behind the A zone used by a queued vehicle',
        ),
        DeclareLaunchArgument(
            'a_zone_waiting_camera_down_offset_m',
            default_value='0.05',
            description='Additional camera-down offset for the A waiting pose',
        ),
        DeclareLaunchArgument(
            'b1_exit_forward_distance_m',
            default_value='0.30',
            description='Forward distance after the B-1 left exit turn',
        ),
        DeclareLaunchArgument(
            'b1_exit_forward_speed_mps',
            default_value='0.05',
            description='Straight-line speed after the B-1 left exit turn',
        ),
        DeclareLaunchArgument(
            'b1_exit_behavior_timeout_sec',
            default_value='20.0',
            description='Timeout for each B-1 exit behavior action',
        ),
        DeclareLaunchArgument(
            'b1_exit_manual_turn',
            default_value='false',
            description=(
                'Bypass Nav2 costmap checking for the B-1 turn; diagnostic '
                'fallback only'
            ),
        ),
        DeclareLaunchArgument(
            'b1_exit_turn_speed_rps',
            default_value='0.25',
            description='Maximum angular speed for the B-1 left turn',
        ),
        DeclareLaunchArgument(
            'b1_exit_open_loop',
            default_value='true',
            description=(
                'Use a low-speed safety-gated crawl for the short B-1 exit leg'
            ),
        ),
        DeclareLaunchArgument(
            'b1_exit_detection_radius_m',
            default_value='0.35',
            description='Radius used to recognize a vehicle at B-1 before exit',
        ),
        DeclareLaunchArgument(
            'b1_zone_map_x',
            default_value='1.294',
            description='B-1 stop position X in the shared map frame',
        ),
        DeclareLaunchArgument(
            'b1_zone_map_y',
            default_value='-0.087',
            description='B-1 stop position Y in the shared map frame',
        ),
        DeclareLaunchArgument(
            'b1_exit_turn_tolerance_deg',
            default_value='5.0',
            description='Required AMCL yaw accuracy before B-1 forward motion',
        ),
        DeclareLaunchArgument(
            'auto_park_idle_sec',
            default_value='0.0',
            description=(
                'Idle seconds before automatic parking; 0 disables it so '
                'only a new parking request can start parking'
            ),
        ),
        DeclareLaunchArgument(
            'park_exit_forward_distance_m',
            default_value='0.20',
            description='Mandatory straight distance before leaving a parking spot',
        ),
        DeclareLaunchArgument(
            'park_exit_forward_speed_mps',
            default_value='0.05',
            description='Straight-line speed while leaving a parking spot',
        ),
        DeclareLaunchArgument(
            'discovery_server_port',
            default_value='11811',
        ),
        DeclareLaunchArgument(
            'start_discovery_server',
            default_value='false',
            description='Use false for the domain-separated Zenoh topology',
        ),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        ExecuteProcess(
            cmd=[
                'fastdds',
                'discovery',
                '-i',
                '0',
                '-l',
                '0.0.0.0',
                '-p',
                LaunchConfiguration('discovery_server_port'),
            ],
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_discovery_server')
            ),
        ),
        OpaqueFunction(function=_configure_discovery),
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
                        'dashboard_slam_pose_topic': '/agv1/amcl_pose',
                        'dashboard_slam_secondary_scan_topic': '/agv2/scan',
                        'dashboard_slam_secondary_pose_topic': (
                            '/agv2/amcl_pose'
                        ),
                        'dashboard_slam_enable_scan': LaunchConfiguration(
                            'dashboard_enable_scan'
                        ),
                        'dashboard_slam_base_frame': 'agv1/base_footprint',
                        'dashboard_slam_secondary_base_frame': (
                            'agv2/base_footprint'
                        ),
                        'start_nav2': 'false',
                        'start_navigation_control': 'false',
                        'use_rviz': 'false',
                    }.items(),
                ),
            ],
        ),
        Node(
            package='central',
            executable='fleet_dispatcher',
            name='fleet_dispatcher',
            output='screen',
            parameters=[{
                'subscribe_odom_fallback': False,
                'b1_exit_left_turn_deg': 90.0,
                'b1_exit_forward_distance_m': ParameterValue(
                    LaunchConfiguration('b1_exit_forward_distance_m'),
                    value_type=float,
                ),
                'b1_exit_forward_speed_mps': ParameterValue(
                    LaunchConfiguration('b1_exit_forward_speed_mps'),
                    value_type=float,
                ),
                'b1_exit_behavior_timeout_sec': ParameterValue(
                    LaunchConfiguration('b1_exit_behavior_timeout_sec'),
                    value_type=float,
                ),
                'b1_exit_manual_turn': ParameterValue(
                    LaunchConfiguration('b1_exit_manual_turn'),
                    value_type=bool,
                ),
                'b1_exit_turn_speed_rps': ParameterValue(
                    LaunchConfiguration('b1_exit_turn_speed_rps'),
                    value_type=float,
                ),
                'b1_exit_open_loop': ParameterValue(
                    LaunchConfiguration('b1_exit_open_loop'),
                    value_type=bool,
                ),
                'b1_exit_detection_radius_m': ParameterValue(
                    LaunchConfiguration('b1_exit_detection_radius_m'),
                    value_type=float,
                ),
                'b1_zone_map_x': ParameterValue(
                    LaunchConfiguration('b1_zone_map_x'),
                    value_type=float,
                ),
                'b1_zone_map_y': ParameterValue(
                    LaunchConfiguration('b1_zone_map_y'),
                    value_type=float,
                ),
                'b1_exit_turn_tolerance_deg': ParameterValue(
                    LaunchConfiguration('b1_exit_turn_tolerance_deg'),
                    value_type=float,
                ),
                'auto_park_idle_sec': ParameterValue(
                    LaunchConfiguration('auto_park_idle_sec'),
                    value_type=float,
                ),
                'park_exit_forward_distance_m': ParameterValue(
                    LaunchConfiguration('park_exit_forward_distance_m'),
                    value_type=float,
                ),
                'park_exit_forward_speed_mps': ParameterValue(
                    LaunchConfiguration('park_exit_forward_speed_mps'),
                    value_type=float,
                ),
                'zone_release_hysteresis_m': 0.05,
            }],
        ),
        Node(
            package='central',
            executable='arm_dispatcher',
            name='arm_dispatcher',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_arm_dispatcher')
            ),
            parameters=[
                PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'arm_dispatcher.yaml',
                ]),
            ],
        ),
        Node(
            package='central',
            executable='inventory_sync',
            name='inventory_sync',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_arm_dispatcher')
            ),
        ),
        Node(
            package='central',
            executable='port_event_detector',
            name='port_event_detector',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_port_autonomy')
            ),
            parameters=[
                PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'port_autonomy.yaml',
                ]),
            ],
        ),
        Node(
            package='central',
            executable='autonomy_orchestrator',
            name='autonomy_orchestrator',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_port_autonomy')
            ),
            parameters=[
                PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'port_autonomy.yaml',
                ]),
            ],
        ),
        Node(
            package='central',
            executable='fleet_collision_supervisor',
            name='fleet_collision_supervisor',
            output='screen',
            condition=IfCondition(
                LaunchConfiguration('start_collision_supervisor')
            ),
            parameters=[
                PathJoinSubstitution([
                    workspace,
                    'config',
                    'central',
                    'fleet_collision_supervisor.yaml',
                ]),
                {
                    'calibration_yaml': PathJoinSubstitution([
                        workspace,
                        'config',
                        'central',
                        'camera_map_calibration.yaml',
                    ]),
                },
            ],
        ),
        Node(
            package='central',
            executable='map_relay',
            name='map_relay',
            output='screen',
        ),
        *[
            Node(
                package='central',
                executable='costmap_parameter_proxy',
                name=f'{vehicle_id}_{scope}_costmap_tuning',
                output='screen',
                parameters=[{
                    'vehicle_id': vehicle_id,
                    'scope': scope,
                }],
            )
            for vehicle_id in ('agv1', 'agv2')
            for scope in ('global', 'local')
        ],
        Node(
            package='drive',
            executable='target_map_pose_to_nav_goal',
            name='agv1_manual_goal_to_nav',
            output='screen',
            parameters=[{
                'target_topic': '/agv1/goal_pose',
                'action_name': '/agv1/navigate_to_pose',
                'default_frame_id': 'map',
            }],
        ),
        Node(
            package='drive',
            executable='target_map_pose_to_nav_goal',
            name='agv2_manual_goal_to_nav',
            output='screen',
            parameters=[{
                'target_topic': '/agv2/goal_pose',
                'action_name': '/agv2/navigate_to_pose',
                'default_frame_id': 'map',
            }],
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
                'b1_waiting_distance_m': ParameterValue(
                    LaunchConfiguration('b1_waiting_distance_m'),
                    value_type=float,
                ),
                'b1_waiting_camera_down_offset_m': ParameterValue(
                    LaunchConfiguration('b1_waiting_camera_down_offset_m'),
                    value_type=float,
                ),
                'a_zone_waiting_distance_m': ParameterValue(
                    LaunchConfiguration('a_zone_waiting_distance_m'),
                    value_type=float,
                ),
                'a_zone_waiting_camera_down_offset_m': ParameterValue(
                    LaunchConfiguration(
                        'a_zone_waiting_camera_down_offset_m'
                    ),
                    value_type=float,
                ),
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
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fleet_map_anchor',
            output='screen',
            arguments=[
                '--x', '0',
                '--y', '0',
                '--z', '0',
                '--roll', '0',
                '--pitch', '0',
                '--yaw', '0',
                '--frame-id', 'map',
                '--child-frame-id', 'fleet_map_anchor',
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
