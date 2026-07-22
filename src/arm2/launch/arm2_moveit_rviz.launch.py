"""Launch RViz for the arm2 MoveIt instance."""

from arm2.arm2_moveit_config import build_arm2_moveit_config
from moveit_configs_utils.launches import generate_moveit_rviz_launch


def generate_launch_description():
    """Create namespaced RViz with the arm2 MoveIt parameters."""
    return generate_moveit_rviz_launch(build_arm2_moveit_config())
