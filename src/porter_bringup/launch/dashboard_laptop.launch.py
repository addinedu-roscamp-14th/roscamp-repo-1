"""Launch the desktop control dashboard on the dashboard laptop."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
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
    python_executable = LaunchConfiguration('python_executable')
    api_token = LaunchConfiguration('api_token')
    ollama_host = LaunchConfiguration('ollama_host')
    llm_model = LaunchConfiguration('llm_model')

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
                default_value='http://127.0.0.1:11434',
            ),
        ),
        DeclareLaunchArgument(
            'llm_model',
            default_value=EnvironmentVariable(
                'LOCAL_LLM_MODEL',
                default_value='gemma4:31b',
            ),
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_CCTV_URL',
            ['http://', central_ip, ':8000/video'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_SLAM_URL',
            ['http://', central_ip, ':8000/slam/video'],
        ),
        SetEnvironmentVariable(
            'PORT_CONTROL_DETECTIONS_URL',
            ['http://', central_ip, ':8000/detections'],
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
        ExecuteProcess(
            cmd=[python_executable, dashboard_script],
            cwd=dashboard_directory,
            output='screen',
        ),
    ])
