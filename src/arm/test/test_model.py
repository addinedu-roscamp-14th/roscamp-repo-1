"""Tests for direct Pick/Place classification and sequence logic."""

from pathlib import Path
import copy

from arm_pick_place.model import (
    MarkerObservation,
    build_pick_place_steps,
    build_split_pick_place_steps,
    calibration_levels_for_surface,
    calibrate_target,
    classify_floor,
    destination_level,
    load_floor_calibration,
    parse_stations,
    safe_z_candidates,
    select_symmetric_yaw,
)

import pytest

import yaml


CALIBRATION_FILE = (
    Path(__file__).parents[1]
    / 'config'
    / 'floor_calibration.yaml'
)


@pytest.fixture
def floors():
    """Load the calibration bundled with this package."""
    return load_floor_calibration(CALIBRATION_FILE)


def observation_from_sample(floors, floor_number, index=0):
    """Create an observation from one real calibration point."""
    xyz = floors[floor_number].marker_points[index]
    return MarkerObservation(*xyz, -94.0, 'station_a')


def test_every_training_marker_is_classified_to_its_floor(floors):
    """Every captured TF point, including floor 0, retains its floor."""
    for number, floor in floors.items():
        for xyz in floor.marker_points:
            observation = MarkerObservation(*xyz, -94.0, 'station_a')
            result, _ = classify_floor(observation, floors)
            assert result == number


def test_homography_projects_an_inlier_to_taught_xy(floors):
    """A floor-1 inlier maps close to its corresponding taught XY."""
    observation = observation_from_sample(floors, 1, 0)
    target, _ = calibrate_target(observation, floors)
    expected = floors[1].taught_points[0]
    assert target.marker_floor == 1
    assert target.x_m == pytest.approx(expected[0], abs=0.002)
    assert target.y_m == pytest.approx(expected[1], abs=0.002)


def test_far_extrapolation_is_rejected(floors):
    """Do not trust a projective transform far outside collected points."""
    observation = MarkerObservation(0.30, -0.50, 0.07, -94.0, 'station_a')
    with pytest.raises(ValueError):
        calibrate_target(observation, floors)


def test_classification_rejects_a_surface_with_only_one_level(floors):
    """A single plane cannot provide a floor-separation safety check."""
    observation = observation_from_sample(floors, 1, 0)
    with pytest.raises(ValueError, match='at least two calibrated levels'):
        classify_floor(observation, {1: floors[1]})


def test_sequence_uses_floor_specific_pick_and_place_z(floors):
    """Place support floor N selects destination-floor N+1 Place Z."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 2, 0), floors
    )
    place, _ = calibrate_target(observation_from_sample(floors, 1, 2), floors)
    steps = build_pick_place_steps(
        pick, place, floors, 0.200, 0.210, -45.0
    )
    assert len(steps) == 9
    assert steps[1].pose[2] == pytest.approx(0.200)
    assert steps[2].pose[2] == pytest.approx(floors[2].pick_z_m)
    assert steps[4].pose[2] == pytest.approx(0.200)
    assert steps[5].pose[2] == pytest.approx(0.210)
    assert steps[6].pose[2] == pytest.approx(floors[2].place_z_m)
    assert steps[8].pose[2] == pytest.approx(0.210)
    assert [step.action for step in steps] == [
        'gripper_open', 'move', 'move', 'gripper_close', 'move',
        'move', 'move', 'gripper_open_after_stop', 'move',
    ]


def test_floor_zero_support_uses_floor_one_place_z(floors):
    """A bare station marker uses H[0] and destination-floor-1 Place Z."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 1, 0), floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 0, 0), floors
    )

    steps = build_pick_place_steps(
        pick, place, floors, 0.200, 0.210, -45.0
    )

    assert place.marker_floor == 0
    assert steps[6].pose[2] == pytest.approx(floors[1].place_z_m)


def test_floor_zero_cannot_be_used_as_a_pick_height(floors):
    """Geometry-only floor 0 must not generate a Pick descent."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 0, 0), floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 1, 0), floors
    )

    with pytest.raises(ValueError, match='floor 0 has no taught Pick Z'):
        build_pick_place_steps(
            pick, place, floors, 0.200, 0.210, -45.0
        )


def test_split_sequence_separates_xy_motion_and_target_rotation(floors):
    """Fallback preserves RPY during XY moves and rotates only afterward."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 1, 0), floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 1, 2), floors
    )
    current_rpy = (-139.09, -18.37, -143.59)
    steps = build_split_pick_place_steps(
        pick, place, floors, 0.190, 0.210, current_rpy, -45.0
    )

    assert len(steps) == 11
    assert [step.action for step in steps] == [
        'gripper_open', 'move', 'move', 'move', 'gripper_close',
        'move', 'move', 'move', 'move', 'gripper_open_after_stop',
        'move',
    ]
    assert steps[1].pose[:3] == pytest.approx(
        (pick.x_m, pick.y_m, 0.190)
    )
    assert steps[1].pose[3:] == pytest.approx(current_rpy)
    pick_target_rpy = (-180.0, 0.0, -139.0)
    assert steps[2].pose[3:] == pytest.approx(pick_target_rpy)
    assert steps[3].pose[2] == pytest.approx(floors[1].pick_z_m)

    assert steps[6].pose[:3] == pytest.approx(
        (place.x_m, place.y_m, 0.210)
    )
    assert steps[6].pose[3:] == pytest.approx(pick_target_rpy)
    assert steps[7].pose[3:] == pytest.approx((-180.0, 0.0, -139.0))
    assert steps[8].pose[2] == pytest.approx(floors[2].place_z_m)


def test_place_on_floor_three_is_rejected_without_floor_four(floors):
    """A detected third-floor support must not silently reuse floor-three Z."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 1, 0), floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 3, 2), floors
    )
    with pytest.raises(ValueError, match='unsupported destination floor 4'):
        build_pick_place_steps(
            pick, place, floors, 0.220, 0.220, -45.0
        )


def test_station_json_is_extensible():
    """Multiple station poses can be added without changing Python code."""
    stations = parse_stations(
        '[{"name":"station_a","joint_angles_deg":[1,2,3,4,5,6],'
        '"timeout_sec":3},'
        '{"name":"station_b","calibration_surface":"agv",'
        '"joint_angles_deg":[6,5,4,3,2,1],"timeout_sec":5}]'
    )
    assert [station.name for station in stations] == [
        'station_a', 'station_b'
    ]
    assert stations[1].timeout_sec == 5.0
    assert stations[0].calibration_surface == 'station'
    assert stations[1].calibration_surface == 'agv'


def test_agv_levels_are_loaded_scoped_and_planned(tmp_path):
    """AGV observations use only agv_0/agv_1 and their explicit Z map."""
    document = yaml.safe_load(CALIBRATION_FILE.read_text(encoding='utf-8'))
    entries = document['floor_calibration']['floors']
    entries['agv_0'] = copy.deepcopy(entries[0])
    entries['agv_1'] = copy.deepcopy(entries[1])
    for sample in entries['agv_0']['xy_samples']:
        sample['floor'] = 'agv_0'
    for sample in entries['agv_1']['xy_samples']:
        sample['floor'] = 'agv_1'
    path = tmp_path / 'calibration.yaml'
    path.write_text(yaml.safe_dump(document), encoding='utf-8')

    loaded = load_floor_calibration(path)
    station_levels = calibration_levels_for_surface(loaded, 'station')
    agv_levels = calibration_levels_for_surface(loaded, 'agv')

    assert list(station_levels) == [0, 1, 2, 3]
    assert list(agv_levels) == ['agv_0', 'agv_1']
    assert destination_level('agv_0') == 'agv_1'
    pick, _ = calibrate_target(
        observation_from_sample(agv_levels, 'agv_1'), agv_levels
    )
    place, _ = calibrate_target(
        observation_from_sample(agv_levels, 'agv_0'), agv_levels
    )
    steps = build_pick_place_steps(
        pick, place, loaded, 0.200, 0.210, -45.0
    )
    assert steps[2].pose[2] == pytest.approx(
        loaded['agv_1'].pick_z_m
    )
    assert steps[6].pose[2] == pytest.approx(
        loaded['agv_1'].place_z_m
    )


def test_symmetric_yaw_avoids_a_half_turn():
    """The 180-degree equivalent is chosen when it needs less rotation."""
    selected, branch, rotation = select_symmetric_yaw(-10.0, 170.0)
    assert selected == pytest.approx(170.0)
    assert branch == pytest.approx(180.0)
    assert rotation == pytest.approx(0.0)


def test_symmetric_yaw_keeps_nominal_when_it_is_closer():
    """Do not flip an already nearby nominal gripper orientation."""
    selected, branch, rotation = select_symmetric_yaw(170.0, -170.0)
    assert selected == pytest.approx(170.0)
    assert branch == pytest.approx(0.0)
    assert rotation == pytest.approx(20.0)


def test_safe_z_candidates_lower_by_one_centimetre():
    """Generate only candidates retaining the requested Z clearance."""
    candidates = safe_z_candidates(0.220, 0.010, 3, 0.195)
    assert candidates == pytest.approx((0.220, 0.210, 0.200))


def test_safe_z_candidates_can_keep_only_configured_height():
    """Stop lowering once the next candidate violates minimum safe Z."""
    candidates = safe_z_candidates(0.220, 0.010, 3, 0.215)
    assert candidates == pytest.approx((0.220,))
