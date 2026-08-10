"""Launch the second JetCobot MoveIt move_group node."""

import os

from arm2.arm2_moveit_config import build_arm2_moveit_config
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)


def generate_launch_description():
    """Create move_group with the arm2-prefixed robot model."""
    moveit_config = build_arm2_moveit_config()
    description = LaunchDescription([
        DeclareBooleanLaunchArg('debug', default_value=False),
        DeclareBooleanLaunchArg(
            'allow_trajectory_execution', default_value=True
        ),
        DeclareBooleanLaunchArg(
            'publish_monitored_planning_scene', default_value=True
        ),
        DeclareLaunchArgument(
            'capabilities',
            default_value=moveit_config.move_group_capabilities[
                'capabilities'
            ],
        ),
        DeclareLaunchArgument(
            'disable_capabilities',
            default_value=moveit_config.move_group_capabilities[
                'disable_capabilities'
            ],
        ),
        DeclareBooleanLaunchArg('monitor_dynamics', default_value=False),
    ])
    should_publish = LaunchConfiguration(
        'publish_monitored_planning_scene'
    )
    move_group_configuration = {
        'publish_robot_description_semantic': True,
        'allow_trajectory_execution': LaunchConfiguration(
            'allow_trajectory_execution'
        ),
        'capabilities': ParameterValue(
            LaunchConfiguration('capabilities'), value_type=str
        ),
        'disable_capabilities': ParameterValue(
            LaunchConfiguration('disable_capabilities'), value_type=str
        ),
        'publish_planning_scene': should_publish,
        'publish_geometry_updates': should_publish,
        'publish_state_updates': should_publish,
        'publish_transforms_updates': should_publish,
        'monitor_dynamics': False,
    }
    add_debuggable_node(
        description,
        package='moveit_ros_move_group',
        executable='move_group',
        commands_file=str(
            moveit_config.package_path / 'launch' / 'gdb_settings.gdb'
        ),
        output='screen',
        parameters=[
            moveit_config.to_dict(),
            move_group_configuration,
        ],
        extra_debug_args=['--debug'],
        additional_env={'DISPLAY': os.environ.get('DISPLAY', '')},
        sigterm_timeout='20.0',
        sigkill_timeout='5.0',
    )
    return description
