"""Publish the JetCobot model TF from measured hardware joints."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    """Create the hardware joint and robot state publisher launch graph."""
    description_share = Path(
        get_package_share_directory('jetcobot_description')
    )
    robot_description = (
        description_share / 'urdf' / 'jetcobot.urdf'
    ).read_text(encoding='utf-8')

    arguments = [
        DeclareLaunchArgument('serial_port', default_value='/dev/jetcobot'),
        DeclareLaunchArgument('baud_rate', default_value='1000000'),
        DeclareLaunchArgument('publish_rate', default_value='10.0'),
        DeclareLaunchArgument('frame_prefix', default_value='arm2/'),
        DeclareLaunchArgument(
            'start_hardware_joint_publisher', default_value='true'
        ),
    ]

    joint_state_node = Node(
        package='arm2',
        executable='arm2_hardware_joint_state_publisher',
        name='hardware_joint_state_publisher',
        namespace='arm2',
        output='screen',
        condition=IfCondition(
            LaunchConfiguration('start_hardware_joint_publisher')
        ),
        parameters=[{
            'serial_port': LaunchConfiguration('serial_port'),
            'baud_rate': LaunchConfiguration('baud_rate'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'joint_states_topic': '/arm2/joint_states',
        }],
    )

    state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        namespace='arm2',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'frame_prefix': LaunchConfiguration('frame_prefix'),
        }],
        remappings=[('joint_states', '/arm2/joint_states')],
    )

    return LaunchDescription(
        arguments + [joint_state_node, state_publisher_node]
    )
