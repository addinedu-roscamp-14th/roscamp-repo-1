"""Launch the second JetCobot MoveIt move_group node."""

from arm2.arm2_moveit_config import build_arm2_moveit_config
from moveit_configs_utils.launches import generate_move_group_launch


def generate_launch_description():
    """Create move_group with the arm2-prefixed robot model."""
    return generate_move_group_launch(build_arm2_moveit_config())
