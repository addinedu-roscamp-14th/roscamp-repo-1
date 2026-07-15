"""Publish the saved JetCobot Eye-in-Hand calibration as TF."""

from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """Include the easy_handeye2 calibration publisher."""
    handeye_share = Path(get_package_share_directory('easy_handeye2'))
    name_argument = DeclareLaunchArgument(
        'name', default_value='jetcobot_eye_in_hand'
    )
    publisher = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(handeye_share / 'launch' / 'publish.launch.py')
        ),
        launch_arguments={
            'name': LaunchConfiguration('name'),
        }.items(),
    )
    return LaunchDescription([name_argument, publisher])
