"""Tests for automatic arm2 hand-eye target generation."""

from arm2.arm2_auto_handeye_sampler import (
    calibration_target_labels,
    generate_calibration_targets,
    quaternion_angle_degrees,
    trajectory_is_reasonable,
)
from geometry_msgs.msg import PoseStamped
import numpy as np
from trajectory_msgs.msg import JointTrajectoryPoint


def make_home():
    """Create a simple TCP home pose."""
    pose = PoseStamped()
    pose.header.frame_id = 'arm2/base_link'
    pose.pose.position.x = 0.20
    pose.pose.position.y = -0.05
    pose.pose.position.z = 0.15
    pose.pose.orientation.w = 1.0
    return pose


def test_targets_are_small_changes_around_home():
    """Sampling matches easy_handeye2's 12 rotations and 5 translations."""
    targets = generate_calibration_targets(make_home(), 25.0, 0.10)

    assert len(targets) == 17
    assert np.isclose(
        quaternion_angle_degrees(
            [0.0, 0.0, 0.0, 1.0],
            [targets[0].pose.orientation.x,
             targets[0].pose.orientation.y,
             targets[0].pose.orientation.z,
             targets[0].pose.orientation.w],
        ),
        25.0,
    )
    assert np.isclose(
        quaternion_angle_degrees(
            [0.0, 0.0, 0.0, 1.0],
            [targets[6].pose.orientation.x,
             targets[6].pose.orientation.y,
             targets[6].pose.orientation.z,
             targets[6].pose.orientation.w],
        ),
        12.5,
    )
    assert np.isclose(targets[12].pose.position.x, 0.25)
    assert np.isclose(targets[13].pose.position.x, 0.15)
    assert np.isclose(targets[14].pose.position.y, 0.05)
    assert np.isclose(targets[15].pose.position.y, -0.15)
    assert np.isclose(targets[16].pose.position.z, 0.15 + 0.10 / 3.0)


def test_quaternion_angle_ignores_equivalent_sign():
    """Equivalent quaternion signs have zero angular difference."""
    assert quaternion_angle_degrees(
        [0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, -1.0]
    ) == 0.0


def test_target_labels_match_official_order():
    """Status labels identify the exact official pose that failed."""
    labels = calibration_target_labels(25.0, 0.10)

    assert len(labels) == 17
    assert labels[1] == 'Roll -25deg'
    assert labels[12] == 'X +50mm'
    assert labels[-1].startswith('Z +33.3333')


def test_preflight_rejects_excessive_joint_travel():
    """Official preflight rejects plans with excessive joint movement."""
    start = JointTrajectoryPoint(positions=[0.0] * 6)
    safe = JointTrajectoryPoint(positions=[np.deg2rad(20.0)] * 6)
    excessive = JointTrajectoryPoint(positions=[np.deg2rad(100.0)] * 6)
    limits = [90.0, 90.0, 90.0, 90.0, 180.0, 350.0]

    assert trajectory_is_reasonable([start, safe], limits)
    assert not trajectory_is_reasonable([start, excessive], limits)
    assert not trajectory_is_reasonable([], limits)
