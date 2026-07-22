"""Tests for JetCobot trajectory interpolation without robot hardware."""

import math

from arm.jetcobot_trajectory_bridge import (
    duration_seconds,
    interpolate_positions,
    JetCobotTrajectoryBridge,
    joint_errors_degrees,
)
from builtin_interfaces.msg import Duration
import pytest
from trajectory_msgs.msg import JointTrajectoryPoint


def make_point(seconds, positions):
    point = JointTrajectoryPoint()
    point.time_from_start = Duration(sec=seconds)
    point.positions = positions
    return point


def test_duration_seconds():
    assert duration_seconds(Duration(sec=2, nanosec=500_000_000)) == 2.5


def test_interpolate_positions_between_points():
    points = [make_point(0, [0.0, 1.0]), make_point(2, [2.0, 3.0])]
    assert interpolate_positions(points, 1.0) == [1.0, 2.0]


def test_interpolate_positions_clamps_after_finish():
    points = [make_point(0, [0.0]), make_point(2, [2.0])]
    assert interpolate_positions(points, 3.0) == [2.0]


def test_j2_limit_matches_urdf_not_old_135_degree_guard():
    positions = [0.0] * 6
    positions[1] = math.radians(-136.2)
    JetCobotTrajectoryBridge._validate_joint_limits(positions)

    positions[1] = math.radians(-167.0)
    with pytest.raises(RuntimeError, match='J2 target'):
        JetCobotTrajectoryBridge._validate_joint_limits(positions)


def test_joint_errors_report_target_minus_actual():
    assert joint_errors_degrees(
        [10.0, -20.0, 30.0], [12.5, -21.0, 30.0]
    ) == [2.5, -1.0, 0.0]
