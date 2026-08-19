"""Tests for ARM2 operation-level terminal event handling."""

import threading
import types

from central.arm_dispatcher import (
    ArmDispatcher,
    is_terminal_arm1_state,
    is_terminal_event,
    movement_from_goal,
)
from porter_interfaces.msg import ArmState
from std_msgs.msg import String


def test_intermediate_completed_state_is_not_terminal():
    event = {
        'phase': 'SOURCE_LOCKED',
        'state': 'COMPLETED',
        'progress': 20,
    }

    assert not is_terminal_event(event)


def test_final_completed_and_failed_phases_are_terminal():
    assert is_terminal_event({'phase': 'COMPLETED', 'state': 'COMPLETED'})
    assert is_terminal_event({'phase': 'FAILED', 'state': 'FAILED'})
    assert is_terminal_event({'phase': 'STOPPED', 'state': 'COMPLETED'})


def test_arm1_fixed_work_state_terminal_contract():
    assert is_terminal_arm1_state('WORK_COMPLETED')
    assert is_terminal_arm1_state('FAILED')
    assert is_terminal_arm1_state('STOPPED')
    assert not is_terminal_arm1_state('PICK_COMPLETED')
    assert not is_terminal_arm1_state('PLACE_COMPLETED')


def test_arm_dispatcher_tracks_both_active_commands_independently():
    dispatcher = object.__new__(ArmDispatcher)
    arm1 = types.SimpleNamespace(arm_id='arm1', command_id='one')
    arm2 = types.SimpleNamespace(arm_id='arm2', command_id='two')
    dispatcher.active_commands = {'arm1': arm1, 'arm2': arm2}
    dispatcher.active_command = arm2

    assert dispatcher._active_command_for('arm1') is arm1
    assert dispatcher._active_command_for('arm2') is arm2


def test_arm1_accepts_only_pick_place_and_stop_operations():
    dispatcher = object.__new__(ArmDispatcher)

    accepted, error = dispatcher._validate(types.SimpleNamespace(
        arm_id='arm1', operation='pick_place', source_id=2, destination_id=9
    ))
    rejected, rejected_error = dispatcher._validate(types.SimpleNamespace(
        arm_id='arm1', operation='load_to_trailer', source_id=2,
        destination_id=9,
    ))

    assert (accepted, error) == ('pick_place', '')
    assert rejected is None
    assert 'unsupported ARM1 operation' in rejected_error


def test_arm1_rejects_missing_or_equal_dynamic_marker_ids():
    dispatcher = object.__new__(ArmDispatcher)

    operation, error = dispatcher._validate(types.SimpleNamespace(
        arm_id='arm1', operation='pick_place', source_id=-1,
        destination_id=-1,
    ))
    same_operation, same_error = dispatcher._validate(types.SimpleNamespace(
        arm_id='arm1', operation='pick_place', source_id=2,
        destination_id=2,
    ))

    assert operation is None
    assert same_operation is None
    assert '0..49' in error
    assert '0..49' in same_error


def test_arm1_goal_becomes_dynamic_execute_service_request():
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.arm1_execute_client = object()
    goal = types.SimpleNamespace(
        arm_id='arm1', source_id=6, destination_id=9
    )

    client, request, accepted_field = dispatcher._service_for_goal(
        goal, 'pick_place'
    )

    assert client is dispatcher.arm1_execute_client
    assert request.pick_id == 6
    assert request.place_id == 9
    assert accepted_field == 'accepted'


def test_two_arm_operations_share_vehicle_cargo_identity():
    load = types.SimpleNamespace(
        operation='pick_place', arm_id='arm1', vehicle_id='agv1',
        source_id=6, destination_id=10, destination_slot='',
        command_id='load', mission_id='mission',
    )
    first = movement_from_goal(load, True, 'op-load', {})
    assert first['container_id'] == '6'
    assert first['destination_location'] == 'AMR1'

    unload = types.SimpleNamespace(
        operation='transfer_to_slot', arm_id='arm2', vehicle_id='agv1',
        source_id=-1, destination_id=-1, destination_slot='A-1-2',
        command_id='unload', mission_id='mission',
    )
    second = movement_from_goal(unload, True, 'op-unload', {'agv1': '6'})
    assert second['container_id'] == '6'
    assert second['source_location'] == 'AMR1'
    assert second['destination_location'] == 'A-1-2'


def test_arm1_trailer_to_ship_uses_correct_agv_marker_mapping():
    goal = types.SimpleNamespace(
        operation='pick_place', arm_id='arm1', vehicle_id='agv2',
        source_id=9, destination_id=23, destination_slot='',
        command_id='ship', mission_id='mission',
    )
    event = movement_from_goal(goal, True, 'op-ship', {'agv2': '6'})
    assert event['container_id'] == '6'
    assert event['source_location'] == 'AMR2'
    assert event['destination_location'] == '선박-6'


def test_explicit_container_id_survives_dispatcher_restart():
    goal = types.SimpleNamespace(
        operation='transfer_to_slot', arm_id='arm2', vehicle_id='agv1',
        source_id=-1, destination_id=-1, destination_slot='A-1-2',
        container_id='6', command_id='unload', mission_id='mission',
    )
    event = movement_from_goal(goal, True, 'op-unload', {})
    assert event['container_id'] == '6'
    assert event['source_location'] == 'AMR1'


def test_arm1_work_state_updates_structured_central_state():
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.condition = threading.Condition()
    dispatcher.arm1_event_sequence = 0
    dispatcher.arm1_events = []
    dispatcher.arm1_last_error = ''
    dispatcher.arm1_state = ArmState.OFFLINE
    dispatcher.arm1_state_text = 'offline'
    dispatcher.arm1_latest_state_at = None

    message = String()
    message.data = 'PICK_STARTED'
    dispatcher._on_arm1_work_state(message)

    assert dispatcher.arm1_state == ArmState.BUSY
    assert dispatcher.arm1_state_text == 'PICK_STARTED'
    assert dispatcher.arm1_events == [(1, 'PICK_STARTED')]


def test_arm1_failed_state_preserves_the_detailed_status_reason():
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.condition = threading.Condition()
    dispatcher.arm1_event_sequence = 0
    dispatcher.arm1_events = []
    dispatcher.arm1_last_error = ''
    dispatcher.arm1_status_text = '작업 실패 및 정지: marker 10 not found'
    dispatcher.arm1_state = ArmState.BUSY
    dispatcher.arm1_state_text = 'SEARCHING'
    dispatcher.arm1_latest_state_at = None

    message = String()
    message.data = 'FAILED'
    dispatcher._on_arm1_work_state(message)

    assert dispatcher.arm1_state == ArmState.ERROR
    assert dispatcher.arm1_last_error == (
        '작업 실패 및 정지: marker 10 not found'
    )


def test_arm1_status_arriving_after_failed_replaces_generic_error():
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.condition = threading.Condition()
    dispatcher.arm1_state = ArmState.ERROR
    dispatcher.arm1_last_error = 'ARM1 pick/place reported FAILED'
    dispatcher.arm1_status_text = ''
    message = String()
    message.data = '작업 실패 및 정지: marker 6 not found'

    dispatcher._on_arm1_status(message)

    assert dispatcher.arm1_last_error == (
        '작업 실패 및 정지: marker 6 not found'
    )


def test_arm1_wait_requires_fresh_start_then_completion():
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.condition = threading.Condition()
    dispatcher.arm1_events = [
        (1, 'WORK_COMPLETED'),
        (2, 'WORK_STARTED'),
        (3, 'PICK_COMPLETED'),
        (4, 'WORK_COMPLETED'),
    ]
    dispatcher.stop_generations = {'arm1': 0, 'arm2': 0}
    dispatcher.get_parameter = lambda _name: types.SimpleNamespace(value=1.0)

    class GoalHandle:
        is_cancel_requested = False
        request = types.SimpleNamespace(command_id='arm1-test')

        def __init__(self):
            self.feedback = []

        def publish_feedback(self, feedback):
            self.feedback.append(feedback.phase)

    goal_handle = GoalHandle()
    state, operation_id, error = dispatcher._wait_for_arm1_terminal(
        goal_handle, baseline_sequence=1, command_generation=0
    )

    assert state == 'WORK_COMPLETED'
    assert operation_id == 'arm1-arm1-test'
    assert error == ''
    assert goal_handle.feedback == [
        'WORK_STARTED', 'PICK_COMPLETED', 'WORK_COMPLETED'
    ]


class StubServiceClient:
    def __init__(self, ready):
        self.ready = ready

    def service_is_ready(self):
        return self.ready


def make_arm2_connectivity_dispatcher(service_ready):
    dispatcher = object.__new__(ArmDispatcher)
    dispatcher.trigger_clients = {
        'stop_pick': StubServiceClient(service_ready),
        'scan_destinations': StubServiceClient(False),
    }
    dispatcher.transfer_by_id_client = StubServiceClient(False)
    dispatcher.active_command = None
    dispatcher.arm2_state = ArmState.OFFLINE
    dispatcher.arm2_state_text = 'waiting for ARM2 event or service'
    dispatcher.arm2_last_error = ''
    return dispatcher


def test_arm2_idle_service_changes_initial_offline_to_ready():
    dispatcher = make_arm2_connectivity_dispatcher(service_ready=True)

    dispatcher._refresh_arm2_idle_connectivity()

    assert dispatcher.arm2_state == ArmState.READY
    assert dispatcher.arm2_state_text == 'SERVICE_CONNECTED'


def test_arm2_service_disappearance_returns_service_only_state_offline():
    dispatcher = make_arm2_connectivity_dispatcher(service_ready=True)
    dispatcher._refresh_arm2_idle_connectivity()
    dispatcher.trigger_clients['stop_pick'].ready = False

    dispatcher._refresh_arm2_idle_connectivity()

    assert dispatcher.arm2_state == ArmState.OFFLINE
    assert dispatcher.arm2_state_text == 'waiting for ARM2 event or service'


def test_arm2_service_probe_does_not_overwrite_event_error():
    dispatcher = make_arm2_connectivity_dispatcher(service_ready=True)
    dispatcher.arm2_state = ArmState.ERROR
    dispatcher.arm2_state_text = 'FAILED'
    dispatcher.arm2_last_error = 'pick failed'

    dispatcher._refresh_arm2_idle_connectivity()

    assert dispatcher.arm2_state == ArmState.ERROR
    assert dispatcher.arm2_state_text == 'FAILED'
    assert dispatcher.arm2_last_error == 'pick failed'
