"""Tests for container pick transform calculations."""

from arm2.arm2_container_pick_coordinator import (
    apply_vertical_pick_offsets,
    cartesian_path_acceptable,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    inverted_l_workspace_contains,
    lift_distance_candidates,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
)
import numpy as np


def test_compose_pose_rotates_marker_offset():
    """Marker yaw rotates its local grasp offset into the base frame."""
    rotation = quaternion_from_rpy_degrees(0.0, 0.0, 90.0)
    translation, result_rotation = compose_pose(
        [0.1, 0.2, 0.3],
        rotation,
        [0.05, 0.0, 0.0],
        quaternion_from_rpy_degrees(0.0, 0.0, 0.0),
    )
    assert np.allclose(translation, [0.1, 0.25, 0.3], atol=1e-9)
    assert np.allclose(result_rotation, rotation, atol=1e-9)


def test_rpy_quaternion_round_trip():
    """A configured grasp orientation survives quaternion conversion."""
    expected = [25.0, -30.0, 70.0]
    quaternion = quaternion_from_rpy_degrees(*expected)
    actual = quaternion_to_rpy_degrees(quaternion)
    assert np.allclose(actual, expected, atol=1e-9)


def test_measured_grasp_offset_reproduces_taught_tcp_pose():
    """The measured marker offset recreates the manually taught TCP pose."""
    translation, rotation = compose_pose(
        [0.0702611714, -0.1812154682, 0.0539621953],
        [-0.0109490729, -0.0413679949, 0.9891935085, -0.1402319851],
        [0.018108, 0.009738, -0.041746],
        quaternion_from_rpy_degrees(-166.852, 11.738, -66.238),
    )
    expected_rotation = np.array([0.431, 0.895, -0.106, 0.030])
    expected_rotation /= np.linalg.norm(expected_rotation)

    assert np.allclose(translation, [0.056, -0.192, 0.011], atol=2e-6)
    assert abs(float(np.dot(rotation, expected_rotation))) > 0.999999


def test_fixed_base_grasp_ignores_unstable_marker_rotation():
    """An aligned container uses marker XYZ and a fixed taught TCP attitude."""
    expected_rotation = quaternion_from_rpy_degrees(-170.530, 8.370, 129.248)
    translation, rotation = compose_fixed_base_pose(
        [0.070261, -0.181215, 0.053962],
        [-0.014261, -0.010785, -0.042962],
        expected_rotation,
    )

    assert np.allclose(translation, [0.056, -0.192, 0.011], atol=1e-9)
    assert np.allclose(rotation, expected_rotation, atol=1e-9)


def test_extra_depth_does_not_lower_pregrasp():
    """Extra grasp depth changes final Z without changing visual pregrasp."""
    grasp, pregrasp = apply_vertical_pick_offsets(
        [0.05, -0.18, 0.011], pregrasp_lift=0.08, extra_depth=0.005
    )

    assert np.allclose(grasp, [0.05, -0.18, 0.006])
    assert np.allclose(pregrasp, [0.05, -0.18, 0.091])


def test_yaw_follow_rotates_grasp_and_xy_offset_only():
    """Marker yaw rotates XY and gripper yaw while retaining roll/pitch."""
    translation, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [0.02, 0.0, -0.04],
        [-170.0, 8.0, 120.0],
        marker_yaw_degrees=60.0,
        reference_marker_yaw_degrees=30.0,
    )
    rpy = quaternion_to_rpy_degrees(rotation)

    assert yaw_delta == 30.0
    assert np.allclose(
        translation,
        [0.10 + 0.02 * np.cos(np.deg2rad(30.0)),
         0.20 + 0.02 * np.sin(np.deg2rad(30.0)),
         0.01],
    )
    assert np.allclose(rpy, [-170.0, 8.0, 150.0], atol=1e-9)


def test_lift_candidates_descend_to_exact_minimum():
    """Adaptive lift searches downward and always tests its minimum."""
    assert np.allclose(
        lift_distance_candidates(0.18, 0.05, 0.02),
        [0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.05],
    )


def test_cartesian_shortfall_is_bounded_in_metres():
    """A lower fraction is accepted only for a small absolute residual."""
    assert cartesian_path_acceptable(0.955, 0.08, 0.97, 0.90, 0.005)
    assert not cartesian_path_acceptable(0.955, 0.18, 0.97, 0.90, 0.005)
    assert not cartesian_path_acceptable(0.89, 0.01, 0.97, 0.90, 0.005)


def test_inverted_l_workspace_excludes_upper_left_quadrant():
    """The configured bottom/right L rejects only its upper-left cutout."""
    bounds = (
        np.array([-0.28, -0.28]),
        np.array([0.28, 0.0]),
        np.array([0.0, -0.28]),
        np.array([0.28, 0.28]),
    )
    assert inverted_l_workspace_contains([0.20, 0.20], *bounds)
    assert inverted_l_workspace_contains([-0.20, -0.20], *bounds)
    assert inverted_l_workspace_contains([0.20, -0.20], *bounds)
    assert not inverted_l_workspace_contains([-0.20, 0.20], *bounds)
