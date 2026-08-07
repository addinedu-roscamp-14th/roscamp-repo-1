import math
import threading
import time

from central.fleet_collision_supervisor import (
    advance_along_path,
    closest_predicted_approach,
    FleetCollisionSupervisor,
    pixel_to_map,
    predict_positions,
    select_fresh_position,
)
import numpy as np


class _ServiceClient:
    def __init__(self, ready):
        self.ready = ready

    def service_is_ready(self):
        return self.ready


class _Motion:
    def __init__(self, locked_zone=''):
        self.locked_zone = locked_zone


class _FakeLogger:
    def __init__(self):
        self.errors = []

    def error(self, message, **kwargs):
        self.errors.append(message)

    def warning(self, message, **kwargs):
        pass

    def info(self, message, **kwargs):
        pass


class _FakeFuture:
    def add_done_callback(self, callback):
        pass


class _AsyncServiceClient:
    def __init__(self, ready=True):
        self.ready = ready
        self.calls = []

    def service_is_ready(self):
        return self.ready

    def call_async(self, request):
        self.calls.append(request)
        return _FakeFuture()


def test_pixel_to_map_applies_homogeneous_transform():
    homography = np.asarray([
        [0.01, 0.0, -1.0],
        [0.0, -0.01, 2.0],
        [0.0, 0.0, 1.0],
    ])

    result = pixel_to_map(homography, (150.0, 125.0))

    assert np.allclose(result, [0.5, 0.75])


def test_advance_along_path_starts_at_nearest_path_section():
    path = [
        np.asarray([0.0, 0.0]),
        np.asarray([1.0, 0.0]),
        np.asarray([2.0, 0.0]),
    ]

    result = advance_along_path([0.8, 0.1], path, 0.5)

    assert np.allclose(result, [1.3, 0.0], atol=1e-6)


def test_crossing_plans_report_imminent_close_approach():
    times = [0.0, 0.5, 1.0, 1.5, 2.0]
    first = predict_positions(
        [-0.5, 0.0],
        [0.0, 0.0],
        [[0.5, 0.0]],
        0.5,
        times,
    )
    second = predict_positions(
        [0.0, -0.5],
        [0.0, 0.0],
        [[0.0, 0.5]],
        0.5,
        times,
    )

    distance, ttc = closest_predicted_approach(first, second, times)

    assert math.isclose(distance, 0.0, abs_tol=1e-9)
    assert math.isclose(ttc, 1.0, abs_tol=1e-9)


def test_held_vehicle_remains_stationary_during_prediction():
    times = [0.0, 0.5, 1.0]

    positions = predict_positions(
        [0.2, -0.1],
        [0.3, 0.0],
        [[1.0, -0.1]],
        0.3,
        times,
        held=True,
    )

    assert all(np.allclose(position, [0.2, -0.1]) for position in positions)


def test_position_selection_prefers_fresh_topdown_vision():
    position, source = select_fresh_position(
        [0.4, 0.2],
        0.1,
        [0.5, 0.3],
        0.1,
        1.5,
        1.5,
    )

    assert source == 'vision'
    assert np.allclose(position, [0.4, 0.2])


def test_position_selection_uses_fleet_pose_when_vehicle_is_occluded():
    position, source = select_fresh_position(
        [0.4, 0.2],
        2.0,
        [0.5, 0.3],
        0.2,
        1.5,
        1.5,
    )

    assert source == 'fleet'
    assert np.allclose(position, [0.5, 0.3])


def test_position_selection_rejects_two_stale_sources():
    position, source = select_fresh_position(
        [0.4, 0.2],
        2.0,
        [0.5, 0.3],
        2.0,
        1.5,
        1.5,
    )

    assert position is None
    assert source == 'missing'


def test_hold_client_prefers_dedicated_safety_hold_service():
    supervisor = object.__new__(FleetCollisionSupervisor)
    safety = _ServiceClient(True)
    emergency = _ServiceClient(True)
    supervisor.hold_clients = {'agv1': safety}
    supervisor.emergency_hold_clients = {'agv1': emergency}
    supervisor._hold_transport = ''

    client, transport = supervisor._select_hold_client('agv1', True)

    assert client is safety
    assert transport == 'safety_hold'


def test_hold_client_falls_back_to_existing_emergency_service():
    supervisor = object.__new__(FleetCollisionSupervisor)
    safety = _ServiceClient(False)
    emergency = _ServiceClient(True)
    supervisor.hold_clients = {'agv2': safety}
    supervisor.emergency_hold_clients = {'agv2': emergency}
    supervisor._hold_transport = ''

    client, transport = supervisor._select_hold_client('agv2', True)

    assert client is emergency
    assert transport == 'emergency_stop'


def test_hold_release_uses_the_same_transport_that_set_the_latch():
    supervisor = object.__new__(FleetCollisionSupervisor)
    safety = _ServiceClient(True)
    emergency = _ServiceClient(True)
    supervisor.hold_clients = {'agv2': safety}
    supervisor.emergency_hold_clients = {'agv2': emergency}
    supervisor._hold_transport = 'emergency_stop'

    client, transport = supervisor._select_hold_client('agv2', False)

    assert client is emergency
    assert transport == 'emergency_stop'


def test_b1_occupant_has_priority_over_other_moving_vehicle():
    supervisor = object.__new__(FleetCollisionSupervisor)
    supervisor.motion = {
        'agv1': _Motion('A'),
        'agv2': _Motion('B-1'),
    }
    supervisor.preferred_priority = 'agv1'

    selected = supervisor._select_yield_vehicle({
        'agv1': True,
        'agv2': True,
    })

    assert selected == 'agv1'


def test_agv1_b1_occupant_also_has_priority():
    supervisor = object.__new__(FleetCollisionSupervisor)
    supervisor.motion = {
        'agv1': _Motion('B-1'),
        'agv2': _Motion('A'),
    }
    supervisor.preferred_priority = 'agv1'

    selected = supervisor._select_yield_vehicle({
        'agv1': True,
        'agv2': False,
    })

    assert selected == 'agv2'


def test_hold_transition_not_stuck_before_timeout():
    stuck, pending_sec = FleetCollisionSupervisor._hold_transition_is_stuck(
        started_at=100.0, timeout_sec=2.0, now=101.5,
    )

    assert stuck is False
    assert pending_sec == 1.5


def test_hold_transition_stuck_after_timeout():
    stuck, pending_sec = FleetCollisionSupervisor._hold_transition_is_stuck(
        started_at=100.0, timeout_sec=2.0, now=102.5,
    )

    assert stuck is True
    assert pending_sec == 2.5


def test_request_hold_recovers_from_a_stuck_transition():
    """
    A stuck service call must not block hold/release evaluation forever.

    The watchdog should clear it and let the pending request (e.g. a release
    once vehicles are far apart) go through in real time.
    """
    supervisor = object.__new__(FleetCollisionSupervisor)
    supervisor._logger = _FakeLogger()
    supervisor.get_logger = lambda: supervisor._logger
    supervisor._hold_transition = True
    supervisor._hold_transition_started_at = time.monotonic() - 10.0
    supervisor.hold_transition_timeout_sec = 2.0
    supervisor._hold_transport = 'safety_hold'
    client = _AsyncServiceClient(ready=True)
    supervisor.hold_clients = {'agv1': client}
    supervisor.emergency_hold_clients = {'agv1': _AsyncServiceClient(False)}

    supervisor._request_hold('agv1', False)

    assert supervisor._logger.errors
    assert supervisor._hold_transition is True
    assert len(client.calls) == 1


def test_request_hold_ignores_a_still_pending_transition():
    supervisor = object.__new__(FleetCollisionSupervisor)
    supervisor._logger = _FakeLogger()
    supervisor.get_logger = lambda: supervisor._logger
    supervisor._hold_transition = True
    supervisor._hold_transition_started_at = time.monotonic()
    supervisor.hold_transition_timeout_sec = 2.0
    client = _AsyncServiceClient(ready=True)
    supervisor.hold_clients = {'agv1': client}
    supervisor.emergency_hold_clients = {'agv1': _AsyncServiceClient(False)}

    supervisor._request_hold('agv1', False)

    assert not supervisor._logger.errors
    assert not client.calls


class _Track:
    def __init__(self, received_at=None, position=None):
        self.received_at = received_at
        self.position = position


class _MotionState:
    def __init__(self, received_at=None, position=None):
        self.fleet_position_received_at = received_at
        self.fleet_position = position


def _supervisor_with_stale_tracking(*, enabled=True, held_for=None):
    """Build a supervisor whose two position sources have both gone stale."""
    supervisor = object.__new__(FleetCollisionSupervisor)
    supervisor._logger = _FakeLogger()
    supervisor.get_logger = lambda: supervisor._logger
    supervisor._lock = threading.Lock()
    supervisor.tracks = {vid: _Track() for vid in ('agv1', 'agv2')}
    supervisor.motion = {vid: _MotionState() for vid in ('agv1', 'agv2')}
    supervisor.max_detection_age = 1.5
    supervisor.max_fleet_pose_age = 1.5
    supervisor.enabled = enabled
    supervisor.held_vehicle = 'agv1' if held_for is not None else ''
    supervisor._hold_started_at = (
        None if held_for is None else time.monotonic() - held_for
    )
    supervisor.minimum_hold_sec = 0.5
    supervisor.max_hold_sec = 10.0
    supervisor.identity_mismatch_m = 0.45
    supervisor.released = []
    supervisor._request_hold = (
        lambda vid, state: supervisor.released.append((vid, state))
    )
    supervisor._publish_status = lambda now, fresh: None
    return supervisor


def test_hold_is_released_once_it_outlives_max_hold_sec_without_tracking():
    """
    Holding a vehicle stops it, which is exactly when vision tends to lose it.

    The normal release path needs fresh positions, so a hold taken as tracking
    degrades would otherwise never be lifted.
    """
    supervisor = _supervisor_with_stale_tracking(held_for=12.0)

    supervisor._evaluate()

    assert supervisor.released == [('agv1', False)]
    assert supervisor._logger.errors


def test_hold_is_kept_while_still_within_max_hold_sec():
    supervisor = _supervisor_with_stale_tracking(held_for=2.0)

    supervisor._evaluate()

    assert supervisor.released == []


def test_disabling_the_supervisor_retries_the_release_without_waiting():
    """_set_enabled's release can hit an unavailable service; retry here."""
    supervisor = _supervisor_with_stale_tracking(enabled=False, held_for=0.1)

    supervisor._evaluate()

    assert supervisor.released == [('agv1', False)]


def test_no_release_is_requested_when_nothing_is_held():
    supervisor = _supervisor_with_stale_tracking(held_for=None)

    supervisor._evaluate()

    assert supervisor.released == []


def test_identity_mismatch_is_none_when_a_source_is_missing():
    assert FleetCollisionSupervisor.identity_mismatch_distance(
        None, 0.1, (1.0, 1.0), 0.1, 1.5, 1.5
    ) is None


def test_identity_mismatch_is_none_when_a_source_is_stale():
    assert FleetCollisionSupervisor.identity_mismatch_distance(
        (1.0, 1.0), 9.0, (1.0, 1.0), 0.1, 1.5, 1.5
    ) is None


def test_identity_mismatch_is_small_when_both_sources_agree():
    distance = FleetCollisionSupervisor.identity_mismatch_distance(
        (1.6489, 0.1618), 0.03, (1.6280, 0.1330), 0.4, 1.5, 1.5
    )
    assert distance is not None and distance < 0.05


def test_identity_mismatch_exposes_a_swapped_detector_label():
    """The yellow box sat on agv2 while agv1_labels still claimed it.

    Vision then reported agv1 at agv2's location, 1.39 m from agv1's own
    fleet pose, and the supervisor read the two vehicles as nearly touching.
    """
    distance = FleetCollisionSupervisor.identity_mismatch_distance(
        (1.6489, 0.1618), 0.03, (0.2607, -0.0024), 0.4, 1.5, 1.5
    )
    assert distance is not None and distance > 1.0


def _judged_separation(horizon, gap_m, step=0.1, speed=0.12):
    """Separation the supervisor acts on for two vehicles closing head-on."""
    times = np.arange(0.0, horizon + step * 0.5, step).tolist()
    first = predict_positions([0.0, 0.0], [speed, 0.0], [], speed, times)
    second = predict_positions([gap_m, 0.0], [-speed, 0.0], [], speed, times)
    distance, _ = closest_predicted_approach(first, second, times)
    return distance


def test_zero_horizon_judges_the_separation_measured_right_now():
    """Horizon 0 means 'stop when they are actually close', not 'will be'."""
    assert math.isclose(_judged_separation(0.0, 0.9), 0.9, abs_tol=1e-6)


def test_a_predictive_horizon_stops_vehicles_that_are_still_far_apart():
    """Why the predictive form was too eager in a workspace this small.

    Two vehicles a comfortable 0.9 m apart are judged at 0.18 m -- under the
    0.22 m stop threshold -- purely because they are closing.
    """
    assert _judged_separation(3.0, 0.9) < 0.22
    assert _judged_separation(0.0, 0.9) > 0.22
