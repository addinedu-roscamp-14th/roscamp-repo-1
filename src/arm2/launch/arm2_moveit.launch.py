"""Launch a fully namespaced MoveIt stack for the second JetCobot."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import PushRosNamespace


def _include(launch_directory, filename, condition=None):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(launch_directory / filename)),
        condition=condition,
    )


def generate_launch_description():
    """Start arm2 state publishing, planning and optional RViz."""
    launch_directory = Path(
        get_package_share_directory('arm2')
    ) / 'launch'
    use_rviz = LaunchConfiguration('use_rviz')
    namespaced_moveit = GroupAction(actions=[
        PushRosNamespace('arm2'),
        _include(
            launch_directory, 'arm2_static_virtual_joint_tfs.launch.py'
        ),
        _include(launch_directory, 'arm2_moveit_rsp.launch.py'),
        _include(launch_directory, 'arm2_move_group.launch.py'),
        _include(
            launch_directory,
            'arm2_moveit_rviz.launch.py',
            condition=IfCondition(use_rviz),
        ),
    ])
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        namespaced_moveit,
    ])
