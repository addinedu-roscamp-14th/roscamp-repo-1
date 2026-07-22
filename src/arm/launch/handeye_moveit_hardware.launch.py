"""Launch MoveIt/RViz and the sole JetCobot serial-port owner."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    """Start physical MoveIt control and both required joint-state topics."""
    moveit_share = Path(
        get_package_share_directory('jetcobot_moveit_config')
    )
    arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/ttyUSB0'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('trajectory_speed', default_value='15'),
        DeclareLaunchArgument(
            'goal_correction_speed', default_value='15'
        ),
        DeclareLaunchArgument('goal_tolerance_deg', default_value='2.5'),
        DeclareLaunchArgument('goal_timeout_sec', default_value='15.0'),
        DeclareLaunchArgument('joint_state_rate_hz', default_value='10.0'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
    ]

    moveit = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(moveit_share / 'launch' / 'real_planning.launch.py')
        ),
        launch_arguments={
            'use_rviz': LaunchConfiguration('use_rviz'),
        }.items(),
    )
    bridge = Node(
        package='arm',
        executable='jetcobot_trajectory_bridge',
        name='jetcobot_trajectory_bridge',
        namespace='arm',
        output='screen',
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': ParameterValue(
                LaunchConfiguration('baud_rate'), value_type=int
            ),
            'speed': ParameterValue(
                LaunchConfiguration('trajectory_speed'), value_type=int
            ),
            'goal_correction_speed': ParameterValue(
                LaunchConfiguration('goal_correction_speed'), value_type=int
            ),
            'goal_tolerance_deg': ParameterValue(
                LaunchConfiguration('goal_tolerance_deg'), value_type=float
            ),
            'goal_timeout_sec': ParameterValue(
                LaunchConfiguration('goal_timeout_sec'), value_type=float
            ),
            'joint_state_rate_hz': ParameterValue(
                LaunchConfiguration('joint_state_rate_hz'), value_type=float
            ),
            'joint_states_topic': '/joint_states',
            'additional_joint_states_topic': '/arm/joint_states',
        }],
    )
    return LaunchDescription(arguments + [moveit, bridge])
