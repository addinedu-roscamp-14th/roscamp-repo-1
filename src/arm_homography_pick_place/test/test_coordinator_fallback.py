"""Tests for controller-result-based Pick fallback without hardware."""

import threading
from dataclasses import replace
from pathlib import Path

from arm_homography_pick_place.coordinator import (
    CoordinateMoveError,
    HomographyPickPlace,
)
from arm_homography_pick_place.model import (
    MarkerObservation,
    calibrate_target,
    load_floor_calibration,
    parse_stations,
)

import pytest


@pytest.fixture
def floors():
    """Load the calibration used by the coordinator package."""
    path = (
        Path(__file__).parents[3]
        / 'calibration_results'
        / 'floor_calibration.yaml'
    )
    return load_floor_calibration(path)


def observation_from_sample(floors, floor_number):
    """Return one exact training observation for a requested floor."""
    xyz = floors[floor_number].marker_points[0]
    return MarkerObservation(*xyz, -94.0, 'station_a')


def make_coordinator(floors):
    """Construct the planning subset of the ROS node."""
    coordinator = object.__new__(HomographyPickPlace)
    coordinator.floors = floors
    coordinator.safe_z = 0.220
    coordinator.yaw_offset = -45.0
    coordinator.place_yaw_offset = 0.0
    coordinator.cross_station_place_yaw_offset = -45.0
    coordinator.serial_lock = threading.Lock()
    coordinator.stop_event = threading.Event()
    coordinator.statuses = []
    coordinator.publish_status = coordinator.statuses.append
    coordinator.work_states = []
    coordinator.publish_work_state = coordinator.work_states.append
    parameters = {
        'minimum_safe_clearance_m': 0.020,
        'safe_z_lowering_step_m': 0.010,
        'maximum_safe_z_lowering_steps': 3,
    }
    coordinator.parameter = parameters.__getitem__

    class CurrentPoseRobot:
        def get_coords(self):
            return [0.0, -180.0, 220.0, -140.0, -18.0, -143.0]

    coordinator.robot = CurrentPoseRobot()
    return coordinator


def make_targets(floors):
    """Use a 1-floor Pick and a 2-floor support/3-floor destination."""
    pick, _ = calibrate_target(
        observation_from_sample(floors, 1), floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 2), floors
    )
    return {'pick': pick, 'place': place}


def test_planning_skips_ik_and_starts_with_combined_approach(floors):
    """No firmware IK call is allowed before the controller command."""
    coordinator = make_coordinator(floors)

    class Robot:
        def get_coords(self):
            return [0.0, -180.0, 220.0, -140.0, -18.0, -143.0]

        def solve_inv_kinematics(self, coords, angles):
            raise AssertionError('firmware IK must not be called')

    coordinator.robot = Robot()
    steps = coordinator.select_feasible_plan(make_targets(floors))

    assert len(steps) == 9
    assert steps[1].pose[2] == pytest.approx(0.190)
    assert steps[1].pose[3:] == pytest.approx((-180.0, 0.0, -139.0))
    assert steps[4].pose[2] == pytest.approx(0.190)
    assert steps[5].pose[2] == pytest.approx(0.210)
    assert any('IK precheck disabled' in text
               for text in coordinator.statuses)


def test_same_station_place_uses_zero_yaw_offset(floors):
    """A same-station Place aligns with marker yaw modulo 180 degrees."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['place'] = replace(targets['place'], yaw_deg=86.0)

    steps = coordinator.select_feasible_plan(targets)

    assert steps[1].pose[5] == pytest.approx(-139.0)
    assert steps[2].pose[5] == pytest.approx(-139.0)
    assert steps[5].pose[5] == pytest.approx(-94.0)
    assert steps[6].pose[5] == pytest.approx(-94.0)
    assert steps[8].pose[5] == pytest.approx(-94.0)
    assert any('cross_station=False' in text
               for text in coordinator.statuses)
    assert any('place_offset=0.00' in text for text in coordinator.statuses)


@pytest.mark.parametrize(
    ('pick_station', 'place_station'),
    (
        ('station_agv', 'station_a'),
        ('station_a', 'station_agv'),
    ),
)
def test_cross_station_place_uses_minus_45_degree_offset(
    floors, pick_station, place_station
):
    """Crossing AGV/station applies the dedicated Place correction."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['pick'] = replace(targets['pick'], station=pick_station)
    targets['place'] = replace(
        targets['place'], station=place_station, yaw_deg=86.0
    )

    steps = coordinator.select_feasible_plan(targets)

    assert steps[5].pose[5] == pytest.approx(-139.0)
    assert steps[6].pose[5] == pytest.approx(-139.0)
    assert steps[8].pose[5] == pytest.approx(-139.0)
    assert any('cross_station=True' in text
               for text in coordinator.statuses)
    assert any('place_offset=-45.00' in text
               for text in coordinator.statuses)


def test_pick_is_split_only_after_combined_controller_move_fails(floors):
    """A failed combined Pick approach retries XY-current-RPY then RPY."""
    coordinator = make_coordinator(floors)
    steps = coordinator.select_feasible_plan(make_targets(floors))

    class Robot:
        def __init__(self):
            self.stop_calls = 0

        def stop(self):
            self.stop_calls += 1

        def get_coords(self):
            return [10.0, -180.0, 210.0, -140.0, -18.0, -143.0]

    coordinator.robot = Robot()
    coordinator.command_gripper = lambda opened: None
    coordinator.wait_until_robot_stopped = lambda context='PLACE': None
    commanded = []

    def move_coords(pose):
        commanded.append(pose)
        if len(commanded) == 1:
            raise CoordinateMoveError('combined target not reached')

    coordinator.move_coords = move_coords
    coordinator.execute_steps(steps)

    assert coordinator.robot.stop_calls == 1
    assert commanded[1][:3] == pytest.approx(commanded[0][:3])
    assert commanded[1][3:] == pytest.approx((-140.0, -18.0, -143.0))
    assert commanded[2] == pytest.approx(commanded[0])
    assert commanded[3][2] == pytest.approx(floors[1].pick_z_m)
    assert any('switching to split approach' in text
               for text in coordinator.statuses)


def test_successful_combined_pick_does_not_add_split_moves(floors):
    """A successful controller move proceeds directly to Pick descent."""
    coordinator = make_coordinator(floors)
    steps = coordinator.select_feasible_plan(make_targets(floors))
    coordinator.command_gripper = lambda opened: None
    coordinator.wait_until_robot_stopped = lambda context='PLACE': None
    commanded = []
    coordinator.move_coords = commanded.append

    coordinator.execute_steps(steps)

    assert len(commanded) == 6
    assert commanded[0] == pytest.approx(steps[1].pose)
    assert commanded[1] == pytest.approx(steps[2].pose)
    assert not any('switching to split approach' in text
                   for text in coordinator.statuses)
    assert coordinator.work_states == [
        'PICK_STARTED',
        'PICK_COMPLETED',
        'PLACE_STARTED',
        'PLACE_COMPLETED',
    ]


def test_search_scopes_agv_then_station_and_keeps_cross_station_targets(
    floors,
):
    """One scan freezes AGV Pick and station-A Place in base coordinates."""
    coordinator = object.__new__(HomographyPickPlace)
    coordinator.floors = {
        **floors,
        'agv_0': replace(floors[0], number='agv_0'),
        'agv_1': replace(floors[1], number='agv_1'),
    }
    coordinator.stations = parse_stations(
        '[{"name":"station_agv","calibration_surface":"agv",'
        '"joint_angles_deg":[15.38,35.59,-2.81,-90.96,4.13,-37.26],'
        '"timeout_sec":0.01},'
        '{"name":"station_a","calibration_surface":"station",'
        '"joint_angles_deg":[-86.39,57.12,-15.46,-88.15,7.99,-36.82],'
        '"timeout_sec":0.01}]'
    )
    coordinator.pick_frame = 'pick'
    coordinator.place_frame = 'place'
    coordinator.stop_event = threading.Event()
    coordinator.active_station = ''
    coordinator.statuses = []
    coordinator.publish_status = coordinator.statuses.append
    coordinator.set_detection_enabled = lambda _enabled: None
    visited = []
    coordinator.move_observation = lambda pose: visited.append(tuple(pose))
    parameters = {
        'observation_settle_sec': 0.0001,
        'maximum_floor_error_m': 0.010,
        'minimum_floor_separation_m': 0.015,
        'maximum_h_extrapolation_m': 0.025,
        'command_area_margin_m': 0.020,
    }
    coordinator.parameter = parameters.__getitem__

    def stable_marker(frame, station):
        if station == 'station_agv' and frame == 'pick':
            xyz = coordinator.floors['agv_1'].marker_points[0]
            return MarkerObservation(*xyz, -94.0, station)
        if station == 'station_a' and frame == 'place':
            xyz = coordinator.floors[0].marker_points[0]
            return MarkerObservation(*xyz, -94.0, station)
        return None

    coordinator.stable_marker = stable_marker

    found = coordinator.search_stations()

    assert len(visited) == 2
    assert found['pick'][1].station == 'station_agv'
    assert found['pick'][1].marker_floor == 'agv_1'
    assert found['place'][1].station == 'station_a'
    assert found['place'][1].marker_floor == 0
