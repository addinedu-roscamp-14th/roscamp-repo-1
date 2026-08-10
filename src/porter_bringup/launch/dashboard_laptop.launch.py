"""Launch the desktop control dashboard on the dashboard laptop."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    UnsetEnvironmentVariable,
)
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Start the GUI with streams pointing at the central laptop."""
    workspace = LaunchConfiguration('workspace')
    central_ip = LaunchConfiguration('central_ip')
    video_port = LaunchConfiguration('video_port')
    python_executable = LaunchConfiguration('python_executable')
    api_token = LaunchConfiguration('api_token')
    ollama_host = LaunchConfiguration('ollama_host')
    llm_model = LaunchConfiguration('llm_model')
    llm_num_ctx = LaunchConfiguration('llm_num_ctx')
    realtime_llm_enabled = LaunchConfiguration('realtime_llm_enabled')
    realtime_llm_interval_sec = LaunchConfiguration(
        'realtime_llm_interval_sec'
    )
    realtime_llm_heartbeat_sec = LaunchConfiguration(
        'realtime_llm_heartbeat_sec'
    )
    realtime_llm_initial_delay_sec = LaunchConfiguration(
        'realtime_llm_initial_delay_sec'
    )
    rmw_implementation = LaunchConfiguration('rmw_implementation')
    discovery_range = LaunchConfiguration('discovery_range')

    dashboard_directory = PathJoinSubstitution([
        workspace,
        'port_control_system1',
    ])
    dashboard_script = PathJoinSubstitution([
        dashboard_directory,
        'agv_control_center.py',
    ])
    fastdds_profile = PathJoinSubstitution([
        FindPackageShare('porter_bringup'),
        'config',
        'fastdds_udp_only.xml',
    ])

    return LaunchDescription([
        DeclareLaunchArgument(
            'workspace',
            default_value=PathJoinSubstitution([
                EnvironmentVariable('HOME'),
                'poter_ws',
            ]),
        ),
        DeclareLaunchArgument(
            'central_ip',
            description='IP address of the central ROS laptop',
        ),
        DeclareLaunchArgument(
            'video_port',
            default_value='8000',
            description=(
                'dashboard_stream_node HTTP port on the central laptop '
                '(must match its port: parameter in dashboard.yaml)'
            ),
        ),
        DeclareLaunchArgument(
            'discovery_server_port',
            default_value='11811',
            description=(
                'Fast DDS discovery server port on the central laptop; must '
                'match discovery_server_port: in fleet_central_laptop.launch.py'
            ),
        ),
        DeclareLaunchArgument(
            'python_executable',
            default_value=PathJoinSubstitution([
                workspace,
                '.venv',
                'bin',
                'python',
            ]),
        ),
        DeclareLaunchArgument(
            'api_token',
            default_value=EnvironmentVariable(
                'PORT_CONTROL_API_TOKEN',
                default_value='',
            ),
        ),
        DeclareLaunchArgument(
            'ollama_host',
            default_value=EnvironmentVariable(
                'OLLAMA_HOST',
                default_value='http://agent.sds.codes',
            ),
        ),
        DeclareLaunchArgument(
            'llm_model',
            default_value=EnvironmentVariable(
                'LOCAL_LLM_MODEL',
                default_value='gemma4:31b',
            ),
        ),
        DeclareLaunchArgument(
            'llm_num_ctx',
            default_value=EnvironmentVariable(
                'LOCAL_LLM_NUM_CTX',
                default_value='8192',
            ),
            description='Ollama context window used for vision requests',
        ),
        DeclareLaunchArgument(
            'realtime_llm_enabled',
            default_value='true',
            description='Continuously reassess the latest operator objective',
        ),
        DeclareLaunchArgument(
            'realtime_llm_interval_sec',
            default_value='2.0',
            description='Minimum live-scene polling interval',
        ),
        DeclareLaunchArgument(
            'realtime_llm_heartbeat_sec',
            default_value='5.0',
            description='Reassess unchanged scenes at this interval',
        ),
        DeclareLaunchArgument(
            'realtime_llm_initial_delay_sec',
            default_value='5.0',
            description='Delay before reassessing a newly dispatched command',
        ),
        DeclareLaunchArgument(
            'rmw_implementation',
            default_value='rmw_cyclonedds_cpp',
            description=(
                'Must match the central stack and the zenoh bridge; a '
                'different RMW cannot see the graph at all'
            ),
        ),
        DeclareLaunchArgument(
            'discovery_range',
            default_value='LOCALHOST',
            description=(
                'LOCALHOST when the dashboard runs on the central laptop '
                '(the zenoh bridge carries traffic to the AMRs). Use SUBNET '
                'only for a dashboard on a separate machine.'
            ),
        ),
        # The fleet now discovers over the zenoh bridge with CycloneDDS, not a
        # Fast DDS discovery server. The dashboard has to match the rest of the
        # central stack exactly: a different RMW cannot see the graph at all,
        # so the GUI came up isolated and every /central/fleet/* subscription
        # plus /agv*/cmd_vel_manual stayed silent.
        SetEnvironmentVariable('RMW_IMPLEMENTATION', rmw_implementation),
        SetEnvironmentVariable(
            'ROS_AUTOMATIC_DISCOVERY_RANGE',
            discovery_range,
        ),
        UnsetEnvironmentVariable('ROS_DISCOVERY_SERVER'),
        SetEnvironmentVariable(
            'PORT_CONTROL_CCTV_URL',
            ['http://', central_ip, ':', video_port, '/video'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_SLAM_URL',
            ['http://', central_ip, ':', video_port, '/slam/video'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_DETECTIONS_URL',
            ['http://', central_ip, ':', video_port, '/detections'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_API_URL',
            ['http://', central_ip, ':8100'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_API_TOKEN',
            api_token,
        ),
        SetEnvironmentVariable(
            'FASTDDS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        SetEnvironmentVariable(
            'FASTRTPS_DEFAULT_PROFILES_FILE',
            fastdds_profile,
        ),
        SetEnvironmentVariable(
            'FASTDDS_BUILTIN_TRANSPORTS',
            'UDPv4',
        ),
        SetEnvironmentVariable('OLLAMA_HOST', ollama_host),
        SetEnvironmentVariable('LOCAL_LLM_MODEL', llm_model),
        SetEnvironmentVariable('LOCAL_LLM_NUM_CTX', llm_num_ctx),
        SetEnvironmentVariable(
            'PORT_CONTROL_REALTIME_LLM_ENABLED',
            realtime_llm_enabled,
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_REALTIME_LLM_INTERVAL_SEC',
            realtime_llm_interval_sec,
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_REALTIME_LLM_HEARTBEAT_SEC',
            realtime_llm_heartbeat_sec,
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_REALTIME_LLM_INITIAL_DELAY_SEC',
            realtime_llm_initial_delay_sec,
        ),
        ExecuteProcess(
            cmd=[python_executable, dashboard_script],
            cwd=dashboard_directory,
            output='screen',
        ),
    ])
