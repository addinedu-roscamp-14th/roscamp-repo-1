"""Launch robot_state_publisher for the arm2 MoveIt model."""

from arm2.arm2_moveit_config import build_arm2_moveit_config
from moveit_configs_utils.launches import generate_rsp_launch


def generate_launch_description():
    """Create robot_state_publisher with arm2-prefixed links."""
    return generate_rsp_launch(build_arm2_moveit_config())
