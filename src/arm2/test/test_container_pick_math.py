"""Tests for container pick transform calculations."""

from arm2.container_pick_coordinator import (
    apply_vertical_pick_offsets,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    interpolate_half_turn_profile,
    interpolate_periodic_profiles,
    lift_distance_candidates,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
    select_layer_index,
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


def test_container_z_offset_and_tcp_offset_preserve_taught_geometry():
    """A 20 mm marker-to-container drop is not applied twice."""
    marker = np.array([0.100333, -0.131, 0.1213])
    container = marker + np.array([0.0, 0.0, -0.020])
    grasp = container + np.array([0.048667, 0.015, -0.0093])

    assert np.allclose(grasp, [0.149, -0.116, 0.092], atol=1e-9)
    final_grasp, _ = apply_vertical_pick_offsets(grasp, 0.08, 0.005)
    assert np.allclose(final_grasp, [0.149, -0.116, 0.087], atol=1e-9)


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


def test_yaw_follow_uses_equivalent_gripper_angle_when_marker_reversed():
    """A reversed marker rotates XY fully without forcing a 180deg wrist turn."""
    translation, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [0.02, 0.0, -0.04],
        [-170.0, 8.0, 120.0],
        marker_yaw_degrees=-150.0,
        reference_marker_yaw_degrees=30.0,
        gripper_yaw_symmetry_degrees=180.0,
    )
    rpy = quaternion_to_rpy_degrees(rotation)

    assert yaw_delta == -180.0
    assert np.allclose(translation, [0.08, 0.20, 0.01], atol=1e-9)
    assert np.allclose(rpy, [-170.0, 8.0, 120.0], atol=1e-9)


def test_reversed_marker_can_keep_base_frame_xy_correction():
    """A centered marker reversal must not flip hand-taught base XY error."""
    translation, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [-0.03, 0.05, -0.04],
        [180.0, 0.0, 95.0],
        marker_yaw_degrees=110.0,
        reference_marker_yaw_degrees=-70.0,
        gripper_yaw_symmetry_degrees=180.0,
        rotate_xy_with_marker=False,
    )

    assert yaw_delta == -180.0
    assert np.allclose(translation, [0.07, 0.25, 0.01], atol=1e-9)
    assert np.allclose(
        quaternion_to_rpy_degrees(rotation), [180.0, 0.0, 95.0], atol=1e-9
    )


def test_half_turn_profile_interpolates_and_repeats_at_180_degrees():
    base = [-0.03, 0.05, -0.04]
    quarter = [-0.04, 0.01, -0.05]

    assert np.allclose(
        interpolate_half_turn_profile(-70, -70, 20, base, quarter), base
    )
    assert np.allclose(
        interpolate_half_turn_profile(20, -70, 20, base, quarter), quarter
    )
    assert np.allclose(
        interpolate_half_turn_profile(110, -70, 20, base, quarter), base
    )
    assert np.allclose(
        interpolate_half_turn_profile(-25, -70, 20, base, quarter),
        [-0.035, 0.03, -0.045],
    )


def test_multi_point_periodic_profile_uses_neighboring_measurements():
    yaws = [-70.0, -25.0, 20.0, 65.0]
    values = [[0.0], [1.0], [2.0], [3.0]]

    assert np.allclose(interpolate_periodic_profiles(-25, yaws, values), [1.0])
    assert np.allclose(interpolate_periodic_profiles(-2.5, yaws, values), [1.5])
    # -70 and +110 are the same parallel-gripper axis.
    assert np.allclose(interpolate_periodic_profiles(110, yaws, values), [0.0])


def test_lift_candidates_descend_to_exact_minimum():
    """Adaptive lift searches downward and always tests its minimum."""
    assert np.allclose(
        lift_distance_candidates(0.18, 0.05, 0.02),
        [0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.05],
    )


def test_select_layer_index_uses_nearest_taught_height():
    values = [-0.12570, -0.08777, -0.04898]
    assert select_layer_index(-0.123, values, 0.025)[0] == 0
    assert select_layer_index(-0.090, values, 0.025)[0] == 1
    assert select_layer_index(-0.050, values, 0.025)[0] == 2


def test_select_layer_index_rejects_unknown_height():
    index, error = select_layer_index(
        0.010, [-0.12570, -0.08777, -0.04898], 0.025
    )
    assert index is None
    assert error > 0.025
