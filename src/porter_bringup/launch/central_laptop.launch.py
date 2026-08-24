"""Launch the complete ROS control stack on the central laptop."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start camera, perception, Nav2, streaming, and control bridges."""
    workspace = LaunchConfiguration('workspace')
    video_device = LaunchConfiguration('video_device')
    camera_info_file = LaunchConfiguration('camera_info_file')
    map_file = LaunchConfiguration('map')
    keepout_mask = LaunchConfiguration('keepout_mask')
    yolo_weights = LaunchConfiguration('yolo_weights')
    yolo_obb_weights = LaunchConfiguration('yolo_obb_weights')
    calibration_yaml = LaunchConfiguration('calibration_yaml')
    b1_camera_left_offset_m = LaunchConfiguration(
        'b1_camera_left_offset_m'
    )
    b1_camera_down_offset_m = LaunchConfiguration(
        'b1_camera_down_offset_m'
    )
    b1_waiting_distance_m = LaunchConfiguration(
        'b1_waiting_distance_m'
    )
    b1_waiting_camera_down_offset_m = LaunchConfiguration(
        'b1_waiting_camera_down_offset_m'
    )
    a_zone_waiting_distance_m = LaunchConfiguration(
        'a_zone_waiting_distance_m'
    )
    a_zone_waiting_camera_down_offset_m = LaunchConfiguration(
        'a_zone_waiting_camera_down_offset_m'
    )
    dashboard_params = LaunchConfiguration('dashboard_params_file')
    dashboard_slam_map_topic = LaunchConfiguration(
        'dashboard_slam_map_topic'
    )
    dashboard_slam_scan_topic = LaunchConfiguration(
        'dashboard_slam_scan_topic'
    )
    dashboard_slam_pose_topic = LaunchConfiguration(
        'dashboard_slam_pose_topic'
    )
    dashboard_slam_enable_scan = LaunchConfiguration(
        'dashboard_slam_enable_scan'
    )
    dashboard_slam_base_frame = LaunchConfiguration(
        'dashboard_slam_base_frame'
    )
    nav2_params = LaunchConfiguration('nav2_params_file')
    control_params = LaunchConfiguration('control_params_file')
    control_host = LaunchConfiguration('control_host')
    api_token = LaunchConfiguration('api_token')
    use_rviz = LaunchConfiguration('use_rviz')
    start_camera = LaunchConfiguration('start_camera')
    start_yolo = LaunchConfiguration('start_yolo')
    start_dashboard_api = LaunchConfiguration('start_dashboard_api')
    start_nav2 = LaunchConfiguration('start_nav2')
    start_navigation_control = LaunchConfiguration(
        'start_navigation_control'
    )

    dashboard_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('dashboard'),
                'launch',
                'dashboard_stream.launch.py',
            ])
        ),
        condition=IfCondition(start_dashboard_api),
        launch_arguments={
            'params_file': dashboard_params,
            'slam_map_topic': dashboard_slam_map_topic,
            'slam_scan_topic': dashboard_slam_scan_topic,
            'slam_pose_topic': dashboard_slam_pose_topic,
            'slam_enable_scan': dashboard_slam_enable_scan,
            'slam_base_frame': dashboard_slam_base_frame,
        }.items(),
    )
    nav2_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drive'),
                'launch',
                'bringup_launch.xml',
            ])
        ),
        condition=IfCondition(start_nav2),
        launch_arguments={
            'workspace': workspace,
            'map': map_file,
            'keepout_mask': keepout_mask,
            'params_file': nav2_params,
        }.items(),
    )
    nav_goal_bridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drive'),
                'launch',
                'target_map_pose_nav.launch.xml',
            ])
        ),
        condition=IfCondition(start_navigation_control),
        launch_arguments={
            'workspace': workspace,
            'map': map_file,
            'keepout_mask': keepout_mask,
            'params_file': nav2_params,
            'start_nav2': 'false',
        }.items(),
    )
    rviz_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('drive'),
                'launch',
                'nav2_view.launch.xml',
            ])
        ),
        condition=IfCondition(use_rviz),
        launch_arguments={'workspace': workspace}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'poter_ws',
            ]),
        ),
        DeclareLaunchArgument(
            'video_device',
            default_value='/dev/video2',
        ),
        DeclareLaunchArgument(
            'camera_info_file',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'main_camera',
                'camera_info.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'map',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'SLAM',
                'current_map.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'keepout_mask',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'SLAM',
                'keepout_mask.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'yolo_weights',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'weights',
                'best.pt',
            ]),
        ),
        DeclareLaunchArgument(
            'yolo_obb_weights',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'weights',
                'best1.pt',
            ]),
        ),
        DeclareLaunchArgument(
            'calibration_yaml',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'central',
                'camera_map_calibration.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'b1_camera_left_offset_m',
            default_value='0.15',
            description=(
                'B-1 parking goal offset along camera-image left, in meters'
            ),
        ),
        DeclareLaunchArgument(
            'b1_camera_down_offset_m',
            default_value='0.03',
            description=(
                'B-1 parking goal offset along camera-image down, in meters'
            ),
        ),
        DeclareLaunchArgument(
            'b1_waiting_distance_m',
            default_value='0.25',
            description='Distance behind the B-1 final pose used while occupied',
        ),
        DeclareLaunchArgument(
            'b1_waiting_camera_down_offset_m',
            default_value='0.06',
            description='Additional camera-down offset for the B-1 waiting pose',
        ),
        DeclareLaunchArgument(
            'a_zone_waiting_distance_m',
            default_value='0.20',
            description='Distance behind the A-zone final pose used while occupied',
        ),
        DeclareLaunchArgument(
            'a_zone_waiting_camera_down_offset_m',
            default_value='0.05',
            description='Additional camera-down offset for the A waiting pose',
        ),
        DeclareLaunchArgument(
            'dashboard_params_file',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'dashboard',
                'dashboard.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'dashboard_slam_map_topic',
            default_value='/map',
        ),
        DeclareLaunchArgument(
            'dashboard_slam_scan_topic',
            default_value='/scan',
        ),
        DeclareLaunchArgument(
            'dashboard_slam_pose_topic',
            default_value='',
        ),
        DeclareLaunchArgument(
            'dashboard_slam_enable_scan',
            default_value='true',
        ),
        DeclareLaunchArgument(
            'dashboard_slam_base_frame',
            default_value='base_footprint',
        ),
        DeclareLaunchArgument(
            'nav2_params_file',
            default_value=PathJoinSubstitution([
                FindPackageShare('drive'),
                'params',
                'nav2_params.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'control_params_file',
            default_value=PathJoinSubstitution([
                workspace,
                'config',
                'central',
                'control_gateway.yaml',
            ]),
        ),
        DeclareLaunchArgument(
            'control_host',
            default_value='0.0.0.0',
        ),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_camera', default_value='true'),
        DeclareLaunchArgument('start_yolo', default_value='true'),
        DeclareLaunchArgument(
            'start_dashboard_api',
            default_value='true',
        ),
        DeclareLaunchArgument('start_nav2', default_value='true'),
        DeclareLaunchArgument(
            'start_navigation_control',
            default_value='true',
        ),
        Node(
            package='v4l2_camera',
            executable='v4l2_camera_node',
            name='topdown_camera',
            output='screen',
            condition=IfCondition(start_camera),
            parameters=[{
                'video_device': video_device,
                'image_size': [640, 480],
                'time_per_frame': [1, 30],
                'camera_info_url': ParameterValue(
                    ['file://', camera_info_file],
                    value_type=str,
                ),
            }],
            remappings=[
                ('image_raw', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
            ],
        ),
        Node(
            package='image_proc',
            executable='rectify_node',
            name='topdown_rectify',
            output='screen',
            condition=IfCondition(start_camera),
            remappings=[
                ('image', '/camera/image_raw'),
                ('camera_info', '/camera/camera_info'),
                ('image_rect', '/camera/image_rect'),
            ],
        ),
        Node(
            package='image_transport',
            executable='republish',
            name='topdown_compressed_republisher',
            output='screen',
            condition=IfCondition(start_camera),
            arguments=['raw', 'compressed'],
            remappings=[
                ('in', '/camera/image_rect'),
                ('out', '/image_rect'),
            ],
        ),
        Node(
            package='yolo',
            executable='yolo_node',
            name='central_yolo',
            output='screen',
            condition=IfCondition(start_yolo),
            parameters=[{
                'weights_path': yolo_weights,
                'obb_weights_path': yolo_obb_weights,
                'input_topic': '/image_rect/compressed',
                'input_is_compressed': True,
            }],
        ),
        dashboard_launch,
        nav2_launch,
        Node(
            package='central',
            executable='camera_to_map_bridge',
            name='camera_to_map_bridge',
            output='screen',
            condition=IfCondition(start_navigation_control),
            parameters=[{
                'calibration_yaml': calibration_yaml,
                'b1_camera_left_offset_m': ParameterValue(
                    b1_camera_left_offset_m,
                    value_type=float,
                ),
                'b1_camera_down_offset_m': ParameterValue(
                    b1_camera_down_offset_m,
                    value_type=float,
                ),
                'b1_waiting_distance_m': ParameterValue(
                    b1_waiting_distance_m,
                    value_type=float,
                ),
                'b1_waiting_camera_down_offset_m': ParameterValue(
                    b1_waiting_camera_down_offset_m,
                    value_type=float,
                ),
                'a_zone_waiting_distance_m': ParameterValue(
                    a_zone_waiting_distance_m,
                    value_type=float,
                ),
                'a_zone_waiting_camera_down_offset_m': ParameterValue(
                    a_zone_waiting_camera_down_offset_m,
                    value_type=float,
                ),
                'waypoint_mode': False,
            }],
        ),
        nav_goal_bridge_launch,
        Node(
            package='central',
            executable='control_gateway',
            name='central_control_gateway',
            output='screen',
            condition=IfCondition(start_navigation_control),
            parameters=[
                control_params,
                {
                    'host': control_host,
                    'api_token': api_token,
                },
            ],
        ),
        rviz_launch,
    ])
