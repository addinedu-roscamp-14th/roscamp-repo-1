"""Launch MoveIt planning against the physical trajectory bridge."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def include_launch(filename, condition=None):
    """Include one generated launch file from this package."""
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('jetcobot_moveit_config'),
                'launch',
                filename,
            ])
        ),
        condition=condition,
    )


def generate_launch_description():
    """Start state publishing, move_group, and optional RViz without fake control."""
    use_rviz = LaunchConfiguration('use_rviz')
    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='true'),
        include_launch('static_virtual_joint_tfs.launch.py'),
        include_launch('rsp.launch.py'),
        include_launch('move_group.launch.py'),
        include_launch(
            'moveit_rviz.launch.py', condition=IfCondition(use_rviz)
        ),
    ])
