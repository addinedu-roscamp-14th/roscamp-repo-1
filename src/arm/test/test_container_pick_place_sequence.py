"""Tests for sequential pick/place marker role selection."""

import threading
from types import SimpleNamespace

from arm.container_pick_coordinator import (
    CartesianPlanningError,
    apply_radial_xy_offset,
)
from arm.container_pick_place_coordinator import (
    ContainerPickPlaceCoordinator,
    alternative_ik_seeds,
    validated_joint_angles_degrees,
)

import numpy as np

import pytest


def test_other_role_maps_pick_and_place_both_directions():
    """The second observation always requests the opposite marker role."""
    assert ContainerPickPlaceCoordinator.other_role('pick') == 'place'
    assert ContainerPickPlaceCoordinator.other_role('place') == 'pick'


def test_other_role_rejects_unknown_role():
    """An unknown detector role cannot silently become a motion target."""
    with pytest.raises(ValueError, match='unknown marker role'):
        ContainerPickPlaceCoordinator.other_role('unknown')


def test_observation_joint_pose_accepts_six_angles():
    """A valid configured J1 through J6 pose is retained in degrees."""
    angles = [-10.0, 20.0, -30.0, 40.0, -50.0, 60.0]
    assert np.allclose(validated_joint_angles_degrees(angles), angles)


def test_observation_joint_pose_rejects_limit_violation():
    """A fixed observation pose cannot exceed physical joint limits."""
    with pytest.raises(ValueError, match='J2'):
        validated_joint_angles_degrees([0.0, 180.0, 0.0, 0.0, 0.0, 0.0])


def test_motion_workspace_check_runs_after_observation_lock():
    """An observed marker cannot authorize an unsafe final motion target."""
    coordinator = object.__new__(ContainerPickPlaceCoordinator)
    coordinator.in_workspace = lambda value: value[2] >= 0.0

    def pose(z):
        return SimpleNamespace(pose=SimpleNamespace(
            position=SimpleNamespace(x=0.1, y=0.1, z=z)
        ))

    with pytest.raises(RuntimeError, match='place target outside workspace'):
        coordinator.validate_locked_motion_targets(
            (pose(0.05), pose(0.10)),
            (pose(-0.01), pose(0.10)),
        )


def test_radial_offset_increases_base_distance_without_changing_direction():
    """A common radial correction works for targets at different bearings."""
    corrected = apply_radial_xy_offset([0.12, 0.16, 0.05], 0.02)

    assert np.allclose(corrected, [0.132, 0.176, 0.05])
    assert np.isclose(np.linalg.norm(corrected[:2]), 0.22)


def test_alternate_ik_seeds_are_deterministic_and_within_limits():
    """Branch search keeps the current seed and samples safe joint limits."""
    current = np.radians([0.0, 30.0, -40.0, -20.0, 10.0, -50.0])

    first = alternative_ik_seeds(current, 12)
    second = alternative_ik_seeds(current, 12)

    assert len(first) == 12
    assert np.allclose(first[0], current)
    assert all(np.allclose(left, right) for left, right in zip(first, second))
    assert all(seed.shape == (6,) for seed in first)
    for seed in first:
        validated_joint_angles_degrees(np.degrees(seed))


def test_place_alternate_branch_keeps_exact_target_pose():
    """Fallback changes joint branch without changing the place pose."""
    coordinator = object.__new__(ContainerPickPlaceCoordinator)
    coordinator.motion_backend = 'moveit'
    coordinator.latest_joint_lock = threading.Lock()
    coordinator.latest_joint_positions = np.zeros(6)
    coordinator.place_ik_branch_attempts = 2
    coordinator.place_ik_timeout = 0.2
    coordinator.place_branch_min_fraction = 0.999
    coordinator.publish_status = lambda text: None
    solved_targets = []

    def solve(target, seed, timeout):
        solved_targets.append(target)
        return np.asarray(seed) + 0.1

    coordinator.solve_collision_free_ik = solve
    points = [
        SimpleNamespace(positions=np.zeros(6)),
        SimpleNamespace(positions=np.full(6, 0.1)),
    ]
    coordinator.plan_cartesian_from_joint_state = (
        lambda target, joints: SimpleNamespace(
            error_code=SimpleNamespace(val=1),
            fraction=1.0,
            solution=SimpleNamespace(
                joint_trajectory=SimpleNamespace(points=points)
            ),
        )
    )
    coordinator._plan_moveit_joint_goal = (
        lambda names, joints: ('safe-preplace-plan', 0.1)
    )
    executed = []
    coordinator._execute_moveit_trajectory = executed.append
    descended = []
    coordinator.move_cartesian_to_pose = descended.append
    place = object()
    preplace = object()

    coordinator.move_place_with_alternate_ik(
        place, preplace, CartesianPlanningError('self collision')
    )

    assert solved_targets[0] is preplace
    assert solved_targets[1] is place
    assert descended == [place]
    assert executed == ['safe-preplace-plan']
