"""Tests for container pick transform calculations."""

from arm2.arm2_container_pick_coordinator import (
    apply_base_frame_correction,
    apply_marker_yaw_correction,
    apply_vertical_pick_offsets,
    bounded_visual_servo_step,
    calculate_heading_aligned_stack_poses,
    calculate_stack_poses,
    cartesian_path_acceptable,
    cartesian_segment_executable,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    ContainerPickCoordinator,
    grouped_marker_locks_satisfied,
    id_transfer_scan_specs,
    inverted_l_workspace_contains,
    lift_distance_candidates,
    nearest_symmetric_yaw_degrees,
    placed_count_for_destination_floor,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
    stack_layer_z_offset,
    symmetric_marker_yaw_degrees,
    trailer_placement_layer,
    visual_servo_within_tolerance,
)
import numpy as np


def test_trailer_scan_requires_source_and_either_trailer():
    """A load scan accepts trailer ID 9 or 10, but always needs its source."""
    assert grouped_marker_locks_satisfied(
        ['source', 'trailer-9', None], (0,), (1, 2)
    )
    assert grouped_marker_locks_satisfied(
        ['source', None, 'trailer-10'], (0,), (1, 2)
    )
    assert not grouped_marker_locks_satisfied(
        [None, 'trailer-9', 'trailer-10'], (0,), (1, 2)
    )
    assert not grouped_marker_locks_satisfied(
        ['source', None, None], (0,), (1, 2)
    )


def test_fixed_id_transfer_scans_source_but_reuses_destination_cache():
    frames = {1: 'container-1', 13: 'fixed-marker-13'}
    histories = {1: 'history-1', 13: 'history-13'}

    fixed_specs = id_transfer_scan_specs(
        1, 13, 'A-2-1', frames, histories
    )
    dynamic_specs = id_transfer_scan_specs(
        1, 3, None,
        {1: 'container-1', 3: 'container-3'},
        {1: 'history-1', 3: 'history-3'},
    )

    assert fixed_specs == [
        ('source container ID 1', 'container-1', 'history-1')
    ]
    assert len(dynamic_specs) == 2


def test_place_correction_follows_marker_red_axis():
    """Marker-local -X remains left of the red axis after marker rotation."""
    correction = [-0.04, 0.0, 0.0]
    assert np.allclose(
        apply_marker_yaw_correction([0.2, 0.1, 0.05], correction, 0.0),
        [0.16, 0.1, 0.05],
    )
    assert np.allclose(
        apply_marker_yaw_correction([0.2, 0.1, 0.05], correction, 90.0),
        [0.2, 0.06, 0.05],
        atol=1e-9,
    )


def test_marker_correction_heading_is_identical_after_half_turn():
    """A rectangular container at yaw+180 uses the same correction axes."""
    reference = -1.728
    yaw_a = symmetric_marker_yaw_degrees(12.0, reference, 180.0)
    yaw_b = symmetric_marker_yaw_degrees(192.0, reference, 180.0)
    assert np.isclose(yaw_a, yaw_b)
    correction = [-0.02, -0.015, -0.02]
    target_a = apply_marker_yaw_correction(
        [0.1, 0.2, 0.05], correction, yaw_a
    )
    target_b = apply_marker_yaw_correction(
        [0.1, 0.2, 0.05], correction, yaw_b
    )
    assert np.allclose(target_a, target_b, atol=1e-9)


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


def test_stop_above_raises_only_final_grasp():
    """A stop-above adjustment preserves the verified pregrasp target."""
    grasp, pregrasp = apply_vertical_pick_offsets(
        [0.05, -0.18, 0.011],
        pregrasp_lift=0.08,
        extra_depth=0.0,
        stop_above=0.02,
    )

    assert np.allclose(grasp, [0.05, -0.18, 0.031])
    assert np.allclose(pregrasp, [0.05, -0.18, 0.091])


def test_stack_pose_preserves_marker_to_tcp_offset():
    """Stacking places the source marker one container above the target."""
    release, approach = calculate_stack_poses(
        source_marker=[0.08, -0.18, 0.06],
        destination_marker=[0.16, -0.10, 0.06],
        grasp_translation=[0.066, -0.191, 0.028],
        container_height=0.05,
        approach_clearance=0.08,
        xy_offset=[0.002, -0.003],
    )

    assert np.allclose(release, [0.148, -0.114, 0.078])
    assert np.allclose(approach, [0.148, -0.114, 0.158])


def test_stack_release_follows_destination_heading():
    """Release yaw and grasp offset follow the destination marker heading."""
    release, approach, rotation, yaw_delta = (
        calculate_heading_aligned_stack_poses(
            destination_marker=[0.16, -0.10, 0.06],
            grasp_offset=[-0.014, -0.010, -0.032],
            grasp_rpy_degrees=[-170.0, 8.0, 120.0],
            destination_yaw_degrees=60.0,
            reference_marker_yaw_degrees=30.0,
            container_height=0.035,
            approach_clearance=0.08,
            extra_depth=0.0,
            xy_offset=[0.002, -0.003],
        )
    )

    expected_offset = np.array([
        -0.014 * np.cos(np.deg2rad(30.0))
        + 0.010 * np.sin(np.deg2rad(30.0)),
        -0.014 * np.sin(np.deg2rad(30.0))
        - 0.010 * np.cos(np.deg2rad(30.0)),
    ])
    expected_xy = np.array([0.16, -0.10]) + expected_offset + [0.002, -0.003]
    assert yaw_delta == 30.0
    assert np.allclose(release[:2], expected_xy)
    assert np.isclose(release[2], 0.063)
    assert np.allclose(approach, release + [0.0, 0.0, 0.08])
    assert np.allclose(
        quaternion_to_rpy_degrees(rotation), [-170.0, 8.0, 150.0]
    )


def test_stack_release_applies_positive_base_z_offset():
    """A positive placement offset raises release and approach equally."""
    release, approach, _, _ = calculate_heading_aligned_stack_poses(
        destination_marker=[0.16, -0.10, 0.06],
        grasp_offset=[-0.014, -0.010, -0.032],
        grasp_rpy_degrees=[-170.0, 8.0, 120.0],
        destination_yaw_degrees=30.0,
        reference_marker_yaw_degrees=30.0,
        container_height=0.035,
        approach_clearance=0.08,
        extra_depth=0.0,
        xy_offset=[0.0, 0.0],
        z_offset=0.03,
    )
    assert np.isclose(release[2], 0.093)
    assert np.isclose(approach[2], 0.173)


def test_stack_layer_offset_increases_by_container_height():
    """Every completed placement raises the next layer by one container."""
    assert np.isclose(stack_layer_z_offset(0.015, 0.035, 0), 0.015)
    assert np.isclose(stack_layer_z_offset(0.015, 0.035, 1), 0.050)
    assert np.isclose(stack_layer_z_offset(0.015, 0.035, 2), 0.085)


def test_each_vehicle_trailer_always_uses_its_single_deck_layer():
    assert trailer_placement_layer(9) == 0
    assert trailer_placement_layer(10) == 0


def test_explicit_destination_floor_overrides_reset_memory_count():
    """A DB floor maps directly to the zero-based physical stack layer."""
    assert placed_count_for_destination_floor(0, 3, 3) == 2
    assert placed_count_for_destination_floor(2, 1, 3) == 0
    assert placed_count_for_destination_floor(2, 0, 3) == 2


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


def test_yaw_follow_treats_180_degrees_as_same_rectangular_grasp():
    """A rectangular grasp is unchanged when the marker rotates 180 degrees."""
    reference_translation, reference_rotation, reference_delta = (
        compose_yaw_follow_pose(
            [0.10, 0.20, 0.05],
            [0.02, -0.01, -0.04],
            [-170.0, 8.0, 120.0],
            marker_yaw_degrees=2.0,
            reference_marker_yaw_degrees=2.0,
            yaw_symmetry_degrees=180.0,
        )
    )
    rotated_translation, rotated_rotation, rotated_delta = (
        compose_yaw_follow_pose(
            [0.10, 0.20, 0.05],
            [0.02, -0.01, -0.04],
            [-170.0, 8.0, 120.0],
            marker_yaw_degrees=182.0,
            reference_marker_yaw_degrees=2.0,
            yaw_symmetry_degrees=180.0,
        )
    )

    assert reference_delta == 0.0
    assert rotated_delta == 0.0
    assert np.allclose(rotated_translation, reference_translation)
    assert np.allclose(rotated_rotation, reference_rotation)


def test_rectangular_grasp_keeps_90_degree_heading_change():
    """The 180-degree symmetry must not collapse a 90-degree turn."""
    _, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [0.02, -0.01, -0.04],
        [-170.0, 8.0, 120.0],
        marker_yaw_degrees=92.0,
        reference_marker_yaw_degrees=2.0,
        yaw_symmetry_degrees=180.0,
    )

    assert abs(yaw_delta) == 90.0
    assert np.isclose(
        abs(quaternion_to_rpy_degrees(rotation)[2] - 120.0),
        90.0,
    )


def test_nearest_symmetric_yaw_minimizes_gripper_rotation():
    """A 180-degree-equivalent grasp nearest current TCP yaw is selected."""
    assert nearest_symmetric_yaw_degrees(140.0, -35.0, 180.0) == -40.0
    assert nearest_symmetric_yaw_degrees(-140.0, 35.0, 180.0) == 40.0


def test_yaw_follow_can_keep_base_frame_position_correction():
    """Yaw may follow the marker while an empirical base offset stays fixed."""
    translation, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [-0.028, -0.006, -0.001],
        [171.0, -7.0, -87.0],
        marker_yaw_degrees=93.0,
        reference_marker_yaw_degrees=3.0,
        rotate_offset=False,
    )

    assert yaw_delta == 90.0
    assert np.allclose(translation, [0.072, 0.194, 0.049])
    assert np.allclose(
        quaternion_to_rpy_degrees(rotation),
        [171.0, -7.0, 3.0],
        atol=1e-9,
    )


def test_base_frame_correction_does_not_rotate_with_marker_yaw():
    """A final base-frame correction is identical at every marker heading."""
    correction = [0.0, -0.01, 0.002]
    translations = []
    for marker_yaw in (-90.0, 0.0, 90.0):
        translation, _, _ = compose_yaw_follow_pose(
            [0.10, 0.20, 0.05],
            [0.02, 0.0, -0.04],
            [-170.0, 8.0, 120.0],
            marker_yaw_degrees=marker_yaw,
            reference_marker_yaw_degrees=0.0,
        )
        corrected = apply_base_frame_correction(translation, correction)
        assert np.allclose(corrected - translation, correction)
        translations.append(corrected)

    assert len(translations) == 3


def test_runtime_tuning_vector_rejects_unsafe_values():
    """Runtime tuning must reject malformed and metre-scale offset mistakes."""
    valid = ContainerPickCoordinator._validated_tuning_vector(
        'grasp_offset_xyz_m',
        [0.006, -0.01, -0.04],
        limit=0.5,
    )
    assert np.allclose(valid, [0.006, -0.01, -0.04])

    for values in ([0.0, 0.0], [-0.3, 3.0, 0.0]):
        try:
            ContainerPickCoordinator._validated_tuning_vector(
                'grasp_offset_xyz_m',
                values,
                limit=0.5,
            )
        except ValueError:
            continue
        raise AssertionError(f'unsafe tuning was accepted: {values}')


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
    assert cartesian_path_acceptable(0.80, 0.0045, 0.97, 0.90, 0.005)
    assert not cartesian_path_acceptable(0.20, 0.0045, 0.97, 0.90, 0.005)


def test_visual_servo_step_applies_gain_and_xy_norm_limit():
    """Visual correction scales error and caps the planar step norm."""
    xy_step, yaw_step = bounded_visual_servo_step(
        [0.008, -0.004],
        5.0,
        xy_gain=0.6,
        yaw_gain=0.6,
        max_xy_step=0.005,
        max_yaw_step_degrees=2.0,
    )

    assert np.isclose(np.linalg.norm(xy_step), 0.005)
    assert np.sign(xy_step[0]) == 1
    assert np.sign(xy_step[1]) == -1
    assert yaw_step == 2.0


def test_visual_servo_tolerance_requires_both_xy_and_yaw():
    """Convergence requires every planar axis and yaw to be bounded."""
    assert visual_servo_within_tolerance(
        [0.0015, -0.0010], 1.5, 0.002, 2.0
    )
    assert not visual_servo_within_tolerance(
        [0.0021, 0.0], 1.5, 0.002, 2.0
    )
    assert not visual_servo_within_tolerance(
        [0.001, 0.0], 2.1, 0.002, 2.0
    )


def test_segmented_descent_requires_safe_progress_and_attempt_budget():
    """Partial descent executes only with substantial bounded progress."""
    assert cartesian_segment_executable(0.651, 0.066, 0.65, 0.005, 0, 5)
    assert not cartesian_segment_executable(
        0.64, 0.066, 0.65, 0.005, 0, 5
    )
    assert not cartesian_segment_executable(
        0.90, 0.004, 0.65, 0.005, 0, 5
    )
    assert not cartesian_segment_executable(
        0.90, 0.071, 0.65, 0.005, 5, 5
    )


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
