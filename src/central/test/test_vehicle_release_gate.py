"""Tests for the arm-to-fleet vehicle departure gate."""

import json
import threading
import time
import types
import weakref

from central.arm_dispatcher import ArmDispatcher
from central.fleet_dispatcher import FleetDispatcher

from porter_interfaces.msg import VehicleState

from std_msgs.msg import String


class StubLogger:
    def __init__(self):
        self.messages = []

    def info(self, message):
        self.messages.append(('info', message))

    def warning(self, message):
        self.messages.append(('warning', message))


class StubGoalHandle:
    def __init__(self, cancel=False):
        self.is_cancel_requested = cancel


def make_goal(vehicle_id='agv1', final_for_vehicle=True):
    return types.SimpleNamespace(
        vehicle_id=vehicle_id,
        final_for_vehicle=final_for_vehicle,
    )


def make_gated_dispatcher(held=(), timeout_sec=1.0):
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher._cargo_condition = threading.Condition()
    dispatcher._cargo_held_vehicles = set(held)
    dispatcher.cargo_hold_timeout_sec = timeout_sec
    dispatcher._logger = StubLogger()
    dispatcher.get_logger = lambda: dispatcher._logger
    return dispatcher


def hold_message(vehicle_ids):
    message = String()
    message.data = json.dumps({'held_vehicles': list(vehicle_ids)})
    return message


def test_only_final_vehicle_cargo_operations_gate_departure():
    assert ArmDispatcher._gates_vehicle(make_goal(), 'transfer_to_slot')
    assert ArmDispatcher._gates_vehicle(make_goal(), 'load_to_trailer')
    assert ArmDispatcher._gates_vehicle(make_goal(), 'pick_place')
    # A scan touches no vehicle, so it must never hold one.
    assert not ArmDispatcher._gates_vehicle(make_goal(), 'scan_destinations')
    # Warehouse-internal moves are not tied to a vehicle either.
    assert not ArmDispatcher._gates_vehicle(make_goal(), 'transfer_by_id')


def test_amr2_uses_the_same_arm_completion_departure_gate():
    goal = make_goal(vehicle_id='agv2')

    assert ArmDispatcher._gates_vehicle(goal, 'pick_place')
    assert ArmDispatcher._gates_vehicle(goal, 'transfer_to_slot')
    assert ArmDispatcher._gates_vehicle(goal, 'load_to_trailer')


def test_gate_needs_both_a_vehicle_and_the_final_flag():
    assert not ArmDispatcher._gates_vehicle(
        make_goal(vehicle_id=''), 'transfer_to_slot'
    )
    assert not ArmDispatcher._gates_vehicle(
        make_goal(final_for_vehicle=False), 'transfer_to_slot'
    )


def test_unheld_vehicle_departs_without_waiting():
    dispatcher = make_gated_dispatcher()

    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(), 'agv1'
    )

    assert allowed
    assert state == ''


def test_held_vehicle_waits_until_the_arm_releases_it():
    dispatcher = make_gated_dispatcher(held=('agv1',), timeout_sec=5.0)

    def release_later():
        time.sleep(0.2)
        dispatcher._on_vehicle_holds(hold_message([]))

    releaser = threading.Thread(target=release_later)
    started = time.monotonic()
    releaser.start()
    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(), 'agv1'
    )
    releaser.join()

    assert allowed
    assert state == ''
    assert time.monotonic() - started >= 0.2


def test_held_amr2_waits_until_the_arm_releases_it():
    dispatcher = make_gated_dispatcher(held=('agv2',), timeout_sec=5.0)

    def release_later():
        time.sleep(0.2)
        dispatcher._on_vehicle_holds(hold_message([]))

    releaser = threading.Thread(target=release_later)
    releaser.start()
    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(), 'agv2'
    )
    releaser.join()

    assert allowed
    assert state == ''


def test_other_vehicles_are_not_blocked_by_a_held_one():
    dispatcher = make_gated_dispatcher(held=('agv1',))

    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(), 'agv2'
    )

    assert allowed
    assert state == ''


def test_hold_times_out_rather_than_stranding_the_vehicle():
    dispatcher = make_gated_dispatcher(held=('agv1',), timeout_sec=0.2)

    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(), 'agv1'
    )

    assert not allowed
    assert state == 'timeout'


def test_cancel_while_held_reports_cancellation():
    dispatcher = make_gated_dispatcher(held=('agv1',), timeout_sec=5.0)

    allowed, state = dispatcher._wait_for_cargo_release(
        StubGoalHandle(cancel=True), 'agv1'
    )

    assert not allowed
    assert state == 'canceled'


def test_snapshot_replaces_the_hold_set_wholesale():
    dispatcher = make_gated_dispatcher(held=('agv1',))

    dispatcher._on_vehicle_holds(hold_message(['agv2']))

    # agv1 must be dropped even though the snapshot never mentions it;
    # snapshots are authoritative so a missed message cannot strand a vehicle.
    assert dispatcher._cargo_held_vehicles == {'agv2'}


def test_malformed_snapshot_leaves_the_previous_holds_intact():
    dispatcher = make_gated_dispatcher(held=('agv1',))
    broken = String()
    broken.data = 'not json'

    dispatcher._on_vehicle_holds(broken)

    assert dispatcher._cargo_held_vehicles == {'agv1'}
    assert dispatcher._logger.messages[-1][0] == 'warning'


def test_arm_result_completes_navigation_predecessor_timeline():
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher._lock = threading.RLock()
    dispatcher._command_condition = threading.Condition(dispatcher._lock)
    dispatcher._command_outcomes = {}
    message = String()
    message.data = json.dumps({
        'command_id': 'arm-step-1',
        'success': True,
    })

    dispatcher._on_arm_result(message)

    assert dispatcher._command_outcomes == {'arm-step-1': True}


def test_invalid_arm_result_cannot_release_a_navigation_step():
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher._lock = threading.RLock()
    dispatcher._command_condition = threading.Condition(dispatcher._lock)
    dispatcher._command_outcomes = {}
    message = String()
    message.data = json.dumps({
        'command_id': 'arm-step-1',
        'success': 'true',
    })

    dispatcher._on_arm_result(message)

    assert dispatcher._command_outcomes == {}


def test_park_waits_for_successful_arm_predecessor():
    scheduled = []

    class StubExecutor:
        def create_task(self, task):
            scheduled.append(task)

    dispatcher = object.__new__(FleetDispatcher)
    dispatcher._lock = threading.RLock()
    dispatcher._command_condition = threading.Condition(dispatcher._lock)
    dispatcher._command_outcomes = {}
    dispatcher._pending_parks = {}
    executor = StubExecutor()
    dispatcher._Node__executor_weakref = weakref.ref(executor)
    dispatcher._dispatch_park = (
        lambda vehicle_id, wait_until_ready=False:
        ('park', vehicle_id, wait_until_ready)
    )
    dispatcher._logger = StubLogger()
    dispatcher.get_logger = lambda: dispatcher._logger
    request = String()
    request.data = json.dumps({
        'vehicle_id': 'agv1',
        'predecessor_command_id': 'arm-load-1',
    })

    dispatcher._on_park_request(request)

    assert scheduled == []
    assert dispatcher._pending_parks == {'arm-load-1': ['agv1']}

    dispatcher._record_command_outcome('arm-load-1', True)

    assert scheduled == [('park', 'agv1', True)]
    assert dispatcher._pending_parks == {}


def test_namespaced_vehicle_ids_are_normalized():
    dispatcher = make_gated_dispatcher()

    dispatcher._on_vehicle_holds(hold_message(['/agv1', 'agv2', '']))

    assert dispatcher._cargo_held_vehicles == {'agv1', 'agv2'}


def make_arrival_dispatcher(zone='A', max_age_sec=5.0):
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.condition = threading.Condition()
    dispatcher.vehicle_states = {}
    dispatcher.vehicle_arrival_zone = zone
    dispatcher.vehicle_state_max_age_sec = max_age_sec
    return dispatcher


def make_per_arm_arrival_dispatcher(max_age_sec=5.0):
    dispatcher = make_arrival_dispatcher(max_age_sec=max_age_sec)
    dispatcher.vehicle_arrival_zones = {'arm1': 'B-1', 'arm2': 'A'}
    return dispatcher


def vehicle_state(
    state=VehicleState.READY,
    locked_zone='A',
    vehicle_id='agv1',
):
    message = VehicleState()
    message.vehicle_id = vehicle_id
    message.state = state
    message.locked_zone = locked_zone
    return message


def test_vehicle_parked_in_the_work_zone_has_arrived():
    dispatcher = make_arrival_dispatcher()
    dispatcher._on_vehicle_state('agv1', vehicle_state())

    assert dispatcher._vehicle_has_arrived('agv1')


def test_vehicle_still_driving_has_not_arrived():
    dispatcher = make_arrival_dispatcher()
    dispatcher._on_vehicle_state(
        'agv1', vehicle_state(state=VehicleState.BUSY)
    )

    # BUSY means a command is still running, so the trailer is not parked
    # in front of the arm yet.
    assert not dispatcher._vehicle_has_arrived('agv1')


def test_vehicle_ready_in_another_zone_has_not_arrived():
    dispatcher = make_arrival_dispatcher()
    dispatcher._on_vehicle_state('agv1', vehicle_state(locked_zone='B-1'))

    assert not dispatcher._vehicle_has_arrived('agv1')


def test_arm1_recognizes_b1_arrival_but_arm2_does_not():
    dispatcher = make_per_arm_arrival_dispatcher()
    dispatcher._on_vehicle_state(
        'agv1', vehicle_state(locked_zone='B-1')
    )

    assert dispatcher._vehicle_has_arrived('agv1', 'arm1')
    assert not dispatcher._vehicle_has_arrived('agv1', 'arm2')


def test_arm2_recognizes_a_arrival_but_arm1_does_not():
    dispatcher = make_per_arm_arrival_dispatcher()
    dispatcher._on_vehicle_state('agv1', vehicle_state(locked_zone='A'))

    assert dispatcher._vehicle_has_arrived('agv1', 'arm2')
    assert not dispatcher._vehicle_has_arrived('agv1', 'arm1')


def test_amr2_arrival_is_recognized_at_both_arm_work_zones():
    dispatcher = make_per_arm_arrival_dispatcher()
    dispatcher._on_vehicle_state(
        'agv2', vehicle_state(locked_zone='B-1', vehicle_id='agv2')
    )

    assert dispatcher._vehicle_has_arrived('agv2', 'arm1')
    assert not dispatcher._vehicle_has_arrived('agv2', 'arm2')

    dispatcher._on_vehicle_state(
        'agv2', vehicle_state(locked_zone='A', vehicle_id='agv2')
    )

    assert dispatcher._vehicle_has_arrived('agv2', 'arm2')
    assert not dispatcher._vehicle_has_arrived('agv2', 'arm1')


def test_unknown_vehicle_has_not_arrived():
    dispatcher = make_arrival_dispatcher()

    assert not dispatcher._vehicle_has_arrived('agv1')


def test_stale_telemetry_does_not_count_as_arrival():
    dispatcher = make_arrival_dispatcher(max_age_sec=0.01)
    dispatcher._on_vehicle_state('agv1', vehicle_state())
    time.sleep(0.05)

    # A pose from seconds ago says nothing about where the vehicle is now.
    assert not dispatcher._vehicle_has_arrived('agv1')
