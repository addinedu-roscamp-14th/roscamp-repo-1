"""Publish the arm2 MoveIt virtual joint transform."""

from arm2.arm2_moveit_config import build_arm2_moveit_config
from moveit_configs_utils.launches import (
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    """Create world to arm2/dummy static TF from the prefixed SRDF."""
    return generate_static_virtual_joint_tfs_launch(
        build_arm2_moveit_config()
    )
