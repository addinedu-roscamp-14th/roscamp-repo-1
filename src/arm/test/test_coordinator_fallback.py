"""Tests for controller-result-based Pick fallback without hardware."""

import threading
from dataclasses import replace
from pathlib import Path

from arm_pick_place.coordinator import (
    CoordinateMoveError,
    HomographyPickPlace,
)
from arm_pick_place.model import (
    MarkerObservation,
    calibrate_target,
    load_floor_calibration,
    parse_stations,
)

import pytest
import yaml


@pytest.fixture
def floors():
    """Load the calibration used by the coordinator package."""
    path = (
        Path(__file__).parents[1]
        / 'config'
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
    coordinator.safe_z = 0.250
    coordinator.yaw_offset = -45.0
    coordinator.place_yaw_offset = -45.0
    coordinator.cross_station_place_yaw_offset = -45.0
    coordinator.disable_agv_symmetric_yaw_selection = True
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
        'place_stop_poll_interval_sec': 0.01,
        'place_stop_timeout_sec': 1.0,
        'place_stop_required_samples': 1,
    }
    coordinator.parameter = parameters.__getitem__

    class CurrentPoseRobot:
        def get_coords(self):
            return [0.0, -180.0, 220.0, -140.0, -18.0, -143.0]

    coordinator.robot = CurrentPoseRobot()
    return coordinator


def make_targets(floors):
    """Use a 1-floor Pick and a 2-floor support/3-floor destination."""
    station_floors = {
        level: calibration
        for level, calibration in floors.items()
        if isinstance(level, int) and not isinstance(level, bool)
    }
    pick, _ = calibrate_target(
        observation_from_sample(floors, 1), station_floors
    )
    place, _ = calibrate_target(
        observation_from_sample(floors, 2), station_floors
    )
    return {'pick': pick, 'place': place}


def test_runtime_config_disables_agv_symmetric_yaw_selection():
    """AGV symmetric yaw selection is disabled by launch-time config."""
    config_path = (
        Path(__file__).parents[1]
        / 'config'
        / 'container_pick_place.yaml'
    )
    parameters = yaml.safe_load(config_path.read_text())[
        '/arm/pick_place'
    ]['ros__parameters']

    assert parameters['disable_agv_symmetric_yaw_selection'] is True


def test_runtime_config_uses_arm1_measured_place_yaw_offsets():
    """The integrated YAML retains ARM1's measured -45-degree offsets."""
    config_path = (
        Path(__file__).parents[1]
        / 'config'
        / 'container_pick_place.yaml'
    )
    parameters = yaml.safe_load(config_path.read_text())[
        '/arm/pick_place'
    ]['ros__parameters']

    assert parameters['place_marker_yaw_offset_deg'] == pytest.approx(-45.0)
    assert parameters[
        'cross_station_place_yaw_offset_deg'
    ] == pytest.approx(-45.0)


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
    assert steps[1].pose[2] == pytest.approx(0.250)
    assert steps[1].pose[3:] == pytest.approx((-180.0, 0.0, -139.0))
    assert steps[4].pose[2] == pytest.approx(0.250)
    assert steps[5].pose[2] == pytest.approx(0.250)
    assert any('IK precheck disabled' in text
               for text in coordinator.statuses)


def test_same_station_place_uses_measured_yaw_offset(floors):
    """A same-station Place applies ARM1's measured -45-degree offset."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['place'] = replace(targets['place'], yaw_deg=86.0)

    steps = coordinator.select_feasible_plan(targets)

    assert steps[1].pose[5] == pytest.approx(-139.0)
    assert steps[2].pose[5] == pytest.approx(-139.0)
    assert steps[5].pose[5] == pytest.approx(-139.0)
    assert steps[6].pose[5] == pytest.approx(-139.0)
    assert steps[8].pose[5] == pytest.approx(-139.0)
    assert any('cross_station=False' in text
               for text in coordinator.statuses)
    assert any('place_offset=-45.00' in text for text in coordinator.statuses)


def test_agv_pick_uses_nominal_aruco_yaw_without_symmetric_branch(floors):
    """AGV Pick keeps nominal ArUco yaw even when +180 is nearer."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['pick'] = replace(
        targets['pick'], station='station_agv', yaw_deg=35.0
    )
    targets['place'] = replace(
        targets['place'], station='station_a', yaw_deg=86.0
    )

    steps = coordinator.select_feasible_plan(targets)

    assert steps[1].pose[5] == pytest.approx(-10.0)
    assert steps[5].pose[5] == pytest.approx(41.0)
    assert any('pick_yaw_mode=nominal' in text
               for text in coordinator.statuses)


def test_agv_place_uses_nominal_aruco_yaw_without_symmetric_branch(floors):
    """AGV Place keeps nominal ArUco yaw even when +180 is nearer."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['pick'] = replace(targets['pick'], station='station_a')
    targets['place'] = replace(
        targets['place'], station='station_agv', yaw_deg=-53.0
    )

    steps = coordinator.select_feasible_plan(targets)

    assert steps[1].pose[5] == pytest.approx(-139.0)
    assert steps[5].pose[5] == pytest.approx(-98.0)
    assert steps[6].pose[5] == pytest.approx(-98.0)
    assert steps[8].pose[5] == pytest.approx(-98.0)
    assert any('place_yaw_mode=nominal' in text
               for text in coordinator.statuses)


def test_other_different_stations_use_configured_cross_station_offset(floors):
    """All cross-station routes use the configured measured correction."""
    coordinator = make_coordinator(floors)
    targets = make_targets(floors)
    targets['pick'] = replace(targets['pick'], station='station_a')
    targets['place'] = replace(
        targets['place'], station='station_b', yaw_deg=86.0
    )

    steps = coordinator.select_feasible_plan(targets)

    assert steps[5].pose[5] == pytest.approx(-139.0)
    assert steps[6].pose[5] == pytest.approx(-139.0)
    assert steps[8].pose[5] == pytest.approx(-139.0)
    assert any('cross_station=True' in text
               for text in coordinator.statuses)
    assert any('place_offset=-45.00' in text for text in coordinator.statuses)


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


def test_pick_safe_z_lowers_after_each_coordinate_timeout(floors):
    """Pick tries 250, 240, then 230 mm and rises to the successful Z."""
    coordinator = make_coordinator(floors)
    steps = coordinator.select_feasible_plan(make_targets(floors))

    class Robot:
        def __init__(self):
            self.stop_calls = 0

        def get_coords(self):
            return [0.0, -180.0, 250.0, -140.0, -18.0, -143.0]

        def stop(self):
            self.stop_calls += 1

    coordinator.robot = Robot()
    coordinator.command_gripper = lambda opened: None
    coordinator.wait_until_robot_stopped = lambda context='PLACE': None
    commanded = []

    def move_coords(pose):
        commanded.append(pose)
        if (
            pose[:2] == steps[1].pose[:2]
            and pose[2] in (0.250, 0.240)
        ):
            raise CoordinateMoveError('get_coords polling timed out')

    coordinator.move_coords = move_coords
    coordinator.execute_steps(steps)

    assert [pose[2] for pose in commanded[:3]] == pytest.approx(
        [0.250, 0.240, 0.230]
    )
    assert commanded[4][2] == pytest.approx(0.230)
    assert coordinator.robot.stop_calls == 2
    assert any('retrying at 240 mm' in text for text in coordinator.statuses)
    assert any('retrying at 230 mm' in text for text in coordinator.statuses)


def test_place_safe_z_lowers_and_rises_to_successful_height(floors):
    """Place retries a timed-out 250 mm approach at 240 mm."""
    coordinator = make_coordinator(floors)
    steps = coordinator.select_feasible_plan(make_targets(floors))

    class Robot:
        def __init__(self):
            self.stop_calls = 0

        def get_coords(self):
            return [0.0, -180.0, 250.0, -140.0, -18.0, -143.0]

        def stop(self):
            self.stop_calls += 1

    coordinator.robot = Robot()
    coordinator.command_gripper = lambda opened: None
    coordinator.wait_until_robot_stopped = lambda context='PLACE': None
    commanded = []
    place_failed = False

    def move_coords(pose):
        nonlocal place_failed
        commanded.append(pose)
        if pose == steps[5].pose and not place_failed:
            place_failed = True
            raise CoordinateMoveError('get_coords polling timed out')

    coordinator.move_coords = move_coords
    coordinator.execute_steps(steps)

    place_attempts = [
        pose for pose in commanded
        if pose[:2] == steps[5].pose[:2]
    ]
    assert place_attempts[0][2] == pytest.approx(0.250)
    assert place_attempts[1][2] == pytest.approx(0.240)
    assert place_attempts[-1][2] == pytest.approx(0.240)
    assert coordinator.robot.stop_calls == 1
    assert any('retrying at 240 mm' in text for text in coordinator.statuses)


def test_job_returns_to_second_station_observation_pose(floors):
    """Final homing uses the second configured station-A observation pose."""
    coordinator = make_coordinator(floors)
    coordinator.stations = parse_stations(
        '[{"name":"station_agv","calibration_surface":"agv",'
        '"joint_angles_deg":[10.98,42.01,-30.58,-72.77,17.31,-22.14],'
        '"timeout_sec":3.0},'
        '{"name":"station_a","calibration_surface":"station",'
        '"joint_angles_deg":[-86.39,57.12,-15.46,-88.15,7.99,-36.82],'
        '"timeout_sec":5.0}]'
    )
    coordinator.active_station = ''
    detection_states = []
    coordinator.set_detection_enabled = detection_states.append
    visited = []
    coordinator.move_observation = lambda pose: visited.append(tuple(pose))
    coordinator.wait_until_robot_stopped = lambda _context: None

    coordinator.return_to_second_observation_pose()

    assert coordinator.active_station == 'station_a'
    assert detection_states == [False]
    assert visited == [coordinator.stations[1].joint_angles_deg]
    assert any('surface=station' in text for text in coordinator.statuses)


def test_work_completed_is_published_before_observation_cleanup(floors):
    """Place completion precedes the best-effort final observation move."""
    coordinator = make_coordinator(floors)
    events = []
    coordinator.publish_work_state = lambda state: events.append(
        ('state', state)
    )
    coordinator.publish_status = lambda status: events.append(
        ('status', status)
    )
    coordinator.return_to_second_observation_pose = lambda: events.append(
        ('cleanup', 'station_a')
    )

    coordinator.complete_work_then_cleanup()

    assert events[0] == ('state', 'WORK_COMPLETED')
    assert events[1] == ('status', '작업 종료: Place 완료')
    assert events[2] == ('cleanup', 'station_a')


def test_cleanup_failure_does_not_reverse_completed_work(floors):
    """A post-work homing failure leaves the completed state intact."""
    coordinator = make_coordinator(floors)

    def fail_cleanup():
        raise RuntimeError('homing failed')

    coordinator.return_to_second_observation_pose = fail_cleanup

    coordinator.complete_work_then_cleanup()

    assert coordinator.work_states == ['WORK_COMPLETED']
    assert any('WORK_COMPLETED 유지' in text
               for text in coordinator.statuses)


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
        '"joint_angles_deg":[10.98,42.01,-30.58,-72.77,17.31,-22.14],'
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
    coordinator.wait_until_robot_stopped = lambda _context: None
    visited = []
    coordinator.move_observation = lambda pose: visited.append(tuple(pose))
    parameters = {
        'observation_settle_sec': 0.0001,
        'maximum_floor_error_m': 0.010,
        'minimum_floor_separation_m': 0.015,
        'maximum_h_extrapolation_m': 0.025,
        'command_area_margin_m': 0.020,
        'place_stop_poll_interval_sec': 0.01,
        'place_stop_timeout_sec': 1.0,
        'place_stop_required_samples': 1,
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
