"""Tests for the backend-independent sequence."""

from arm_simple_pick_place.sequence import (
    Heights,
    MarkerPose,
    observation_roles,
    pick_steps,
    place_steps,
    pose_within_tolerance,
    remaining_observation_roles,
)
import pytest


HEIGHTS = Heights(0.20, 0.0, 0.18, 0.012, 0.20)
MARKER = MarkerPose(0.11, -0.07, 0.025, 32.0)


def test_pick_sequence_preserves_xy_and_attitude_for_vertical_motion():
    steps = pick_steps(MARKER, HEIGHTS)
    poses = [step.pose for step in steps if step.action == 'move']

    assert [step.action for step in steps] == [
        'gripper_open', 'move', 'move', 'gripper_close', 'move'
    ]
    assert all(pose[:2] == (0.11, -0.07) for pose in poses)
    assert all(pose[3:] == (-180.0, 0.0, 32.0) for pose in poses)
    assert [pose[2] for pose in poses] == [0.20, 0.025, 0.18]


def test_place_uses_marker_z_plus_clearance():
    steps = place_steps(MARKER, HEIGHTS)
    poses = [step.pose for step in steps if step.action == 'move']

    assert [step.action for step in steps] == [
        'move', 'move', 'gripper_open', 'move'
    ]
    assert [pose[2] for pose in poses] == pytest.approx(
        [0.20, 0.037, 0.20]
    )


def test_pick_and_place_add_common_marker_yaw_offset():
    pick_poses = [
        step.pose for step in pick_steps(MARKER, HEIGHTS, 45.0)
        if step.action == 'move'
    ]
    place_poses = [
        step.pose for step in place_steps(MARKER, HEIGHTS, 45.0)
        if step.action == 'move'
    ]

    assert all(pose[5] == pytest.approx(77.0) for pose in pick_poses)
    assert all(pose[5] == pytest.approx(77.0) for pose in place_poses)


def test_marker_yaw_offset_wraps_to_signed_degrees():
    marker = MarkerPose(0.0, 0.0, 0.0, 170.0)
    pose = pick_steps(marker, HEIGHTS, 45.0)[1].pose

    assert pose[5] == pytest.approx(-145.0)


def test_pose_tolerances_are_inclusive_and_yaw_wraps():
    target = [0.1, 0.2, 0.3, -180.0, 0.0, 179.0]
    actual = [0.105, 0.196, 0.3, -177.0, -2.0, -179.0]

    assert pose_within_tolerance(actual, target, 0.005, 3.0)
    assert not pose_within_tolerance(actual, target, 0.0049, 3.0)


def test_pick_and_place_observes_pick_then_place():
    assert observation_roles('pick') == ('pick',)
    assert observation_roles('place') == ('place',)
    assert observation_roles('pick_and_place') == ('pick', 'place')


def test_second_pose_searches_every_marker_missing_from_first():
    required = observation_roles('pick_and_place')

    assert remaining_observation_roles(required, {}) == ('pick', 'place')
    assert remaining_observation_roles(
        required, {'place': object()}
    ) == ('pick',)
    assert remaining_observation_roles(
        required, {'pick': object(), 'place': object()}
    ) == ()
