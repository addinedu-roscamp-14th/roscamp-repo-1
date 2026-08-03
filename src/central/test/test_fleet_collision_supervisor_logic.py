import math

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
