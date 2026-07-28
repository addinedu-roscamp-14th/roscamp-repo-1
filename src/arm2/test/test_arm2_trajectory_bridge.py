"""Tests for JetCobot trajectory interpolation without robot hardware."""

import math

from arm2.arm2_jetcobot_trajectory_bridge import (
    cumulative_joint_travel_degrees,
    duration_seconds,
    interpolate_positions,
    JetCobotTrajectoryBridge,
    joint_errors_degrees,
    validate_home_angles,
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

    positions[1] = math.radians(-141.0)
    with pytest.raises(RuntimeError, match='J2 target'):
        JetCobotTrajectoryBridge._validate_joint_limits(positions)


def test_j3_limit_allows_small_tracking_overshoot():
    positions = [0.0] * 6
    positions[2] = -2.31308
    JetCobotTrajectoryBridge._validate_joint_limits(positions)

    positions[2] = math.radians(-151.0)
    with pytest.raises(RuntimeError, match='J3 target'):
        JetCobotTrajectoryBridge._validate_joint_limits(positions)


def test_joint_errors_report_target_minus_actual():
    assert joint_errors_degrees(
        [10.0, -20.0, 30.0], [12.5, -21.0, 30.0]
    ) == [2.5, -1.0, 0.0]


def test_validate_home_angles_accepts_measured_pose():
    measured = [-90.61, 8.96, -36.29, -52.29, 2.02, -48.25]
    assert validate_home_angles(measured) == measured


def test_validate_home_angles_rejects_wrong_length_and_limit():
    with pytest.raises(ValueError, match='six values'):
        validate_home_angles([0.0] * 5)
    with pytest.raises(ValueError, match='home J5'):
        validate_home_angles([0.0, 0.0, 0.0, 0.0, 161.0, 0.0])


def test_j6_is_limited_to_plus_or_minus_150_degrees():
    positions = [0.0] * 6
    positions[5] = math.radians(150.0)
    JetCobotTrajectoryBridge._validate_joint_limits(positions)

    positions[5] = math.radians(150.1)
    with pytest.raises(RuntimeError, match='J6 target'):
        JetCobotTrajectoryBridge._validate_joint_limits(positions)


def test_cumulative_j6_travel_counts_direction_changes():
    points = [
        make_point(1, [0.0] * 5 + [math.radians(20.0)]),
        make_point(2, [0.0] * 5 + [math.radians(-10.0)]),
    ]
    assert cumulative_joint_travel_degrees(0.0, points, 5) == pytest.approx(
        50.0
    )
