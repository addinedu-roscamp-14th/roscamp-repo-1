"""Launch isolated official-URDF versus controller-coordinate diagnostics."""

from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


DEFAULT_URDF = (
    '/home/choe-gyu-seung/bizlink-Yahboom.jetcobot_ws/src/'
    'jetcobot_description/urdf/jetcobot.urdf'
)


def create_nodes(context):
    """Read the external URDF and construct an isolated prefixed TF tree."""
    urdf_path = Path(
        LaunchConfiguration('official_urdf_path').perform(context)
    ).expanduser().resolve()
    if not urdf_path.is_file():
        raise RuntimeError(f'official URDF not found: {urdf_path}')
    robot_description = urdf_path.read_text(encoding='utf-8')
    frame_prefix = LaunchConfiguration('frame_prefix').perform(context)
    base_frame = f'{frame_prefix}base_link'
    flange_frame = f'{frame_prefix}6_Link'
    return [
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            namespace='official_urdf_test',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': frame_prefix,
            }],
            remappings=[
                ('joint_states', '/official_urdf_test/joint_states'),
            ],
        ),
        Node(
            package='jetcobot_model_diagnostics',
            executable='official_urdf_coords_test',
            name='official_urdf_coords_test',
            output='screen',
            parameters=[{
                'serial_port': LaunchConfiguration('serial_port'),
                'baud_rate': ParameterValue(
                    LaunchConfiguration('baud_rate'), value_type=int
                ),
                'joint_states_topic': '/official_urdf_test/joint_states',
                'base_frame': base_frame,
                'flange_frame': flange_frame,
                'read_count': ParameterValue(
                    LaunchConfiguration('read_count'), value_type=int
                ),
                'output_csv': LaunchConfiguration('output_csv'),
            }],
        ),
    ]


def generate_launch_description():
    """Declare launch arguments without touching the existing robot stack."""
    return LaunchDescription([
        DeclareLaunchArgument(
            'official_urdf_path', default_value=DEFAULT_URDF
        ),
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('frame_prefix', default_value='official/'),
        DeclareLaunchArgument('read_count', default_value='5'),
        DeclareLaunchArgument(
            'output_csv',
            default_value=(
                '~/poter_ws/test_results/'
                'official_urdf_coords_samples.csv'
            ),
        ),
        OpaqueFunction(function=create_nodes),
    ])
