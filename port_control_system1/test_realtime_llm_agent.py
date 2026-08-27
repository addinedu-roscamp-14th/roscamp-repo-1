"""Tests for event-driven realtime VLM supervision helpers."""

import json
import time
from types import SimpleNamespace

import pytest

import realtime_llm_agent

from realtime_llm_agent import (
    RealtimeLLMAgent,
    action_signature,
    observation_signature,
    supervision_command,
)


class PreviousProcessCycleStore:
    def __init__(self):
        self.load_calls = 0

    def load(self):
        self.load_calls += 1
        return realtime_llm_agent.AutonomousCycle(
            phase='UNLOADING_INBOUND',
            inbound_ids=['2', '3', '6'],
            outbound_ids=['8'],
            active_move={'container_id': '2'},
            identical_failures=3,
        )

    def save(self, _cycle):
        pass


def test_process_restart_preserves_only_outbound_intent():
    store = PreviousProcessCycleStore()

    agent = RealtimeLLMAgent(cycle_store=store)

    assert store.load_calls == 1
    assert agent._cycle.phase == 'WAITING_FOR_INBOUND'
    assert agent._cycle.inbound_ids == []
    assert agent._cycle.outbound_ids == ['8']
    assert agent._cycle.active_move is None
    assert agent._cycle.identical_failures == 0


def fleet_status(agv1_state='READY', agv2_state='READY', zone='B-1:FREE;A:FREE'):
    return {
        'telemetry': {
            'vehicles': {
                'agv1': {
                    'state': agv1_state,
                    'current_command_id': '',
                    'locked_zone': '',
                    'pose': {'x': 0.1, 'y': 0.2},
                },
                'agv2': {
                    'state': agv2_state,
                    'current_command_id': '',
                    'locked_zone': '',
                    'pose': {'x': 1.0, 'y': 0.2},
                },
            },
            'b1_zone': zone,
        }
    }


def detections(center_x=100.0):
    return {
        'detections': [{
            'label': 'B-1',
            'confidence': 0.9,
            'bbox_xyxy': [center_x - 10.0, 100.0, center_x + 10.0, 120.0],
        }]
    }


def test_observation_signature_ignores_tiny_detection_jitter():
    first = observation_signature(detections(100.0), fleet_status())
    second = observation_signature(detections(104.0), fleet_status())

    assert first == second


def test_observation_signature_changes_on_vehicle_state_event():
    ready = observation_signature(detections(), fleet_status())
    busy = observation_signature(
        detections(), fleet_status(agv1_state='BUSY')
    )

    assert ready != busy


def test_action_signature_is_stable_across_key_order():
    first = {
        'type': 'visual_navigation',
        'vehicle_id': 'agv1',
        'detection_index': 2,
    }
    second = {
        'detection_index': 2,
        'vehicle_id': 'agv1',
        'type': 'visual_navigation',
    }

    assert action_signature(first) == action_signature(second)


def test_supervision_prompt_contains_objective_and_live_state():
    prompt = supervision_command(
        '두 차량으로 화물을 옮겨',
        fleet_status(zone='B-1:agv1;A:FREE'),
        '추가 동작 없음',
    )

    assert '두 차량으로 화물을 옮겨' in prompt
    assert 'B-1:agv1;A:FREE' in prompt
    assert '추가 동작 없음' in prompt
    assert '명령을 반복하지 마세요' in prompt


def test_parallel_vehicle_selection_reserves_distinct_ready_vehicles():
    agent = object.__new__(RealtimeLLMAgent)
    reserved = set()
    status = fleet_status()

    first = agent._resolve_vehicle('', None, {}, status, reserved)
    second = agent._resolve_vehicle('', None, {}, status, reserved)

    assert (first, second) == ('agv1', 'agv2')


def test_autonomous_vehicle_selection_alternates_ready_vehicles():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    status = fleet_status()

    first = agent._ready_vehicle(status)
    agent._cycle.last_vehicle_id = first
    second = agent._ready_vehicle(status)
    agent._cycle.last_vehicle_id = second
    third = agent._ready_vehicle(status)

    assert (first, second, third) == ('agv1', 'agv2', 'agv1')


def test_autonomous_vehicle_selection_uses_only_available_vehicle():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        last_vehicle_id='agv1'
    )

    selected = agent._ready_vehicle(
        fleet_status(agv1_state='BUSY', agv2_state='READY')
    )

    assert selected == 'agv2'


def test_autonomous_selection_accepts_idle_vehicle_outside_parking_zone():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        last_vehicle_id='agv2'
    )
    status = fleet_status(agv2_state='BUSY')
    status['telemetry']['vehicles']['agv1'].update({
        'state': 'READY',
        'current_command_id': '',
        'locked_zone': '',
    })

    selected = agent._ready_vehicle(status)

    assert selected == 'agv1'


def test_loaded_amr_recovery_selects_the_same_vehicle():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    status = fleet_status()
    inventory = {'cargos': [{
        'container_id': '6', 'location': 'AMR2', 'floor': 1,
    }]}

    selected = agent._ready_vehicle(
        status,
        inventory_snapshot=inventory,
        require_empty_trailer=False,
        required_vehicle_id='agv2',
    )

    assert selected == 'agv2'


def test_arm_failure_retry_reuses_vehicle_occupying_b1():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        last_vehicle_id='agv1'
    )
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'

    selected = agent._ready_vehicle(status, preferred_zone='B-1')

    # Round-robin would normally select agv2 after agv1, but the trailer
    # already at ARM1 must remain assigned to this retry.
    assert selected == 'agv1'


def test_other_vehicle_waits_when_b1_owner_is_not_ready():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        last_vehicle_id='agv1'
    )
    status = fleet_status(agv1_state='BUSY', agv2_state='READY')
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'

    selected = agent._ready_vehicle(status, preferred_zone='B-1')

    assert selected == ''


def test_arm2_failure_retry_reuses_vehicle_occupying_a_station():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        last_vehicle_id='agv2'
    )
    status = fleet_status()
    status['telemetry']['vehicles']['agv2']['locked_zone'] = 'A'

    selected = agent._ready_vehicle(status, preferred_zone='A')

    assert selected == 'agv2'


def test_vehicle_with_db_occupied_trailer_is_not_selected():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    inventory = {'cargos': [{
        'container_id': '8', 'location': 'AMR1', 'floor': 1,
    }]}

    selected = agent._ready_vehicle(
        fleet_status(),
        inventory_snapshot=inventory,
        require_empty_trailer=True,
    )

    assert selected == 'agv2'


def test_b1_owner_with_occupied_trailer_blocks_replacement_vehicle():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'
    inventory = {'cargos': [{
        'container_id': '8', 'location': 'AMR1', 'floor': 1,
    }]}

    selected = agent._ready_vehicle(
        status,
        preferred_zone='B-1',
        inventory_snapshot=inventory,
        require_empty_trailer=True,
    )

    assert selected == ''


def test_new_objective_seeds_actions_already_sent_by_manual_command():
    agent = RealtimeLLMAgent()
    agent.initial_delay_sec = 5.0
    action = {
        'type': 'visual_navigation',
        'vehicle_id': 'agv1',
        'detection_index': 0,
    }

    agent.set_objective('agv1을 B-1로 보내', [action])

    assert action_signature(action) in agent._sent_actions
    assert agent.snapshot().objective == 'agv1을 B-1로 보내'
    assert agent._not_before_monotonic > agent._last_evaluation_monotonic


def test_waiting_mode_requests_parking_only_once_per_vehicle():
    class Client:
        def __init__(self):
            self.calls = []

        def send_park(self, vehicle_id):
            self.calls.append(vehicle_id)
            return {'command_id': f'park-{vehicle_id}'}

    agent = object.__new__(RealtimeLLMAgent)
    agent._autonomy_park_requests = set()
    client = Client()
    status = fleet_status()

    agent._park_idle_vehicles(client, status)
    agent._park_idle_vehicles(client, status)

    assert client.calls == ['agv1', 'agv2']


def test_work_assignment_allows_vehicle_to_be_parked_again_later():
    agent = object.__new__(RealtimeLLMAgent)
    agent._autonomy_park_requests = {'agv1'}

    agent._autonomy_park_requests.discard('agv1')

    assert 'agv1' not in agent._autonomy_park_requests


def test_inbound_transport_waits_for_all_arm_scans():
    status = fleet_status()
    status['telemetry']['autonomy'] = {
        'arm1_ship_cache_ready': True,
        'arm2_destination_cache_ready': True,
        'inbound_scan_pending': True,
    }
    status['telemetry']['arms'] = {
        'arm1': {'current_operation': 'scan_inbound'},
    }

    blocker = RealtimeLLMAgent._autonomy_scan_blocker(
        'UNLOADING_INBOUND', status
    )

    assert blocker == 'ARM1 입항 컨테이너 스캔 완료 대기 중'


def test_inbound_transport_starts_only_after_both_caches_are_ready():
    status = fleet_status()
    status['telemetry']['autonomy'] = {
        'arm1_ship_cache_ready': True,
        'arm2_destination_cache_ready': True,
        'inbound_scan_pending': False,
    }
    status['telemetry']['arms'] = {
        'arm1': {'current_operation': ''},
    }

    blocker = RealtimeLLMAgent._autonomy_scan_blocker(
        'UNLOADING_INBOUND', status
    )

    assert blocker == ''


def test_transient_autonomy_error_is_persisted(tmp_path):
    agent = object.__new__(RealtimeLLMAgent)
    agent._lock = __import__('threading').RLock()
    agent._snapshot = RealtimeLLMAgent().snapshot()
    agent._diagnostic_path = str(tmp_path / 'autonomy_status.json')

    agent._update_snapshot(
        state='ERROR', phase='UNLOADING_INBOUND',
        last_error='planner rejected output',
    )

    payload = json.loads((tmp_path / 'autonomy_status.json').read_text())
    assert payload['state'] == 'ERROR'
    assert payload['phase'] == 'UNLOADING_INBOUND'
    assert payload['last_error'] == 'planner rejected output'


def test_autonomous_mission_id_dependency_is_available():
    """The dispatch path must be able to create autonomous mission IDs."""
    mission_id = f'auto-{realtime_llm_agent.uuid.uuid4().hex[:12]}'

    assert mission_id.startswith('auto-')
    assert len(mission_id) == 17


def test_active_move_with_missing_db_cargo_is_failed_not_waited():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = SimpleNamespace(active_move={
        'container_id': '2', '_steps': [], '_step_index': 0,
    })

    outcome, detail = agent._advance_active_move(
        None, {'telemetry': {}}, {'cargos': []}
    )

    assert outcome == 'failed'
    assert '최신 DB에 없어' in detail


def test_parallel_moves_allow_different_arm_stations_at_the_same_time():
    agent = object.__new__(RealtimeLLMAgent)
    arm1_move = {
        '_steps': [{'type': 'arm1_pick_place'}],
        '_step_index': 0,
        '_current_command_id': 'arm1-command',
    }
    arm2_move = {
        '_steps': [{'type': 'arm_transfer_to_slot'}],
        '_step_index': 0,
        '_current_command_id': '',
    }
    agent._cycle = SimpleNamespace(active_moves={
        'agv1': arm1_move,
        'agv2': arm2_move,
    })

    assert agent._active_step_resources_available(
        arm2_move, {'telemetry': {'arms': {}}}
    )


def test_parallel_moves_serialize_the_same_arm_station():
    agent = object.__new__(RealtimeLLMAgent)
    first = {
        '_steps': [{'type': 'arm1_pick_place'}],
        '_step_index': 0,
        '_current_command_id': 'arm1-command',
    }
    second = {
        '_steps': [{'type': 'zone_navigation', 'zone': 'B-1'}],
        '_step_index': 0,
        '_current_command_id': '',
    }
    agent._cycle = SimpleNamespace(active_moves={
        'agv1': first,
        'agv2': second,
    })

    assert not agent._active_step_resources_available(
        second, {'telemetry': {'arms': {}}}
    )


def test_ready_vehicle_excludes_vehicle_with_an_active_move():
    agent = object.__new__(RealtimeLLMAgent)
    agent._cycle = SimpleNamespace(last_vehicle_id='')

    selected = agent._ready_vehicle(
        fleet_status(), excluded_vehicle_ids={'agv1'}
    )

    assert selected == 'agv2'


def test_removing_primary_parallel_move_promotes_the_other_vehicle():
    agent = object.__new__(RealtimeLLMAgent)
    first = {'_vehicle_id': 'agv1', '_mission_id': 'one'}
    second = {'_vehicle_id': 'agv2', '_mission_id': 'two'}
    agent._cycle = SimpleNamespace(
        active_move=first,
        active_mission_id='one',
        active_moves={'agv1': first, 'agv2': second},
    )

    agent._remove_active_move('agv1')

    assert agent._cycle.active_moves == {'agv2': second}
    assert agent._cycle.active_move is second
    assert agent._cycle.active_mission_id == 'two'


def test_completed_trailer_load_is_not_physically_sent_again():
    agent = object.__new__(RealtimeLLMAgent)
    move = {
        'container_id': '8',
        'destination_location': 'AMR1',
        '_steps': [{
            'type': 'arm1_pick_place', 'arm_id': 'arm1',
            'source_id': 8, 'destination_id': 10,
            'vehicle_id': 'agv1',
        }],
        '_step_index': 0,
        '_current_command_id': '',
        '_dispatched_at': 0.0,
        '_nav_missing_since': 0.0,
    }
    agent._cycle = SimpleNamespace(active_move=move)
    agent._save_cycle = lambda: None
    agent._dispatch_active_step = lambda _client: pytest.fail(
        'already completed ARM step must not be dispatched'
    )
    inventory = {'cargos': [{
        'container_id': '8', 'location': 'AMR1', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(
        None, {'telemetry': {}}, inventory
    )

    assert (outcome, detail) == ('completed', '')


def test_arm_source_id_must_match_active_db_container():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 7, 'destination_id': 10,
    }
    move = {'container_id': '8'}
    inventory = {'cargos': [{
        'container_id': '8', 'location': '선박-2', 'floor': 1,
    }]}

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='does not match DB container',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, inventory, {'telemetry': {}}
        )


def test_arm_load_rejects_a_different_db_cargo_already_on_trailer():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 7, 'destination_id': 10,
    }
    move = {'container_id': '7'}
    inventory = {'cargos': [
        {'container_id': '7', 'location': '선박-2', 'floor': 1},
        {'container_id': '8', 'location': 'AMR1', 'floor': 1},
    ]}

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='AMR1 already carries DB cargo: 8',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, inventory, {'telemetry': {}}
        )


def test_second_ship_pickup_waits_until_loaded_vehicle_clears_b1():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 7, 'destination_id': 9,
    }
    move = {'container_id': '7'}
    inventory = {'cargos': [
        {'container_id': '7', 'location': '선박-2', 'floor': 1},
        {'container_id': '6', 'location': 'AMR1', 'floor': 1},
    ]}
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='B-1 이탈 후 다음 ARM1 상차',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, inventory, status
        )


def test_next_ship_pickup_allowed_after_loaded_vehicle_clears_b1():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 7, 'destination_id': 9,
    }
    move = {'container_id': '7'}
    inventory = {'cargos': [
        {'container_id': '7', 'location': '선박-2', 'floor': 1},
        {'container_id': '6', 'location': 'AMR1', 'floor': 1},
    ]}
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = ''

    RealtimeLLMAgent._validate_active_step(
        step, move, inventory, status
    )


def test_second_ship_arm_step_waits_without_failing_while_b1_is_loaded():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 7, 'destination_id': 9,
    }
    inventory = {'cargos': [
        {'container_id': '7', 'location': '선박-2', 'floor': 1},
        {'container_id': '6', 'location': 'AMR1', 'floor': 1},
    ]}
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'

    reason = RealtimeLLMAgent._arm_step_wait_reason(
        step, status, inventory
    )

    assert 'B-1 이탈 후 다음 ARM1 상차' in reason


def test_navigation_to_b1_is_not_blocked_by_loaded_vehicle_wait():
    step = {
        'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': 'agv2',
    }
    inventory = {'cargos': [
        {'container_id': '6', 'location': 'AMR1', 'floor': 1},
    ]}
    status = fleet_status()
    status['telemetry']['vehicles']['agv1']['locked_zone'] = 'B-1'

    assert RealtimeLLMAgent._arm_step_wait_reason(
        step, status, inventory
    ) == ''


def test_next_arm_command_waits_for_other_physical_result_to_reach_db():
    agent = object.__new__(RealtimeLLMAgent)
    first = {
        '_vehicle_id': 'agv1',
        '_awaiting_arm_db_sync': 'arm-first',
    }
    second = {'_vehicle_id': 'agv2'}
    agent._cycle = realtime_llm_agent.AutonomousCycle(
        active_move=first,
        active_moves={'agv1': first, 'agv2': second},
    )

    reason = agent._other_arm_result_waiting_for_db(second)

    assert 'arm-first의 DB 반영 대기' in reason


def test_arm2_unload_requires_planned_container_on_selected_trailer():
    step = {
        'type': 'arm_transfer_to_slot', 'arm_id': 'arm2',
        'destination_slot': 'A-1-1', 'vehicle_id': 'agv1',
    }
    move = {
        'container_id': '7', 'source_location': '선박-5',
        '_vehicle_id': 'agv1',
    }
    inventory = {'cargos': [
        {'container_id': '7', 'location': '선박-5', 'floor': 1},
        {'container_id': '8', 'location': 'AMR1', 'floor': 1},
    ]}

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='expected AMR1, DB=선박-5',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, inventory, {'telemetry': {}}
        )


def test_arm2_unload_accepts_planned_container_on_selected_trailer():
    step = {
        'type': 'arm_transfer_to_slot', 'arm_id': 'arm2',
        'destination_slot': 'A-1-1', 'vehicle_id': 'agv1',
    }
    move = {
        'container_id': '7', 'source_location': '선박-5',
        '_vehicle_id': 'agv1',
    }
    inventory = {'cargos': [{
        'container_id': '7', 'location': 'AMR1', 'floor': 1,
    }]}

    RealtimeLLMAgent._validate_active_step(
        step, move, inventory, {'telemetry': {}}
    )


@pytest.mark.parametrize(
    ('vehicle_id', 'trailer_location'),
    [('agv1', 'AMR1'), ('agv2', 'AMR2')],
)
def test_arm1_ship_place_validates_fresh_container_on_selected_trailer(
    vehicle_id, trailer_location
):
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 6, 'destination_id': 19,
        'vehicle_id': vehicle_id,
    }
    move = {
        'container_id': '6', 'source_location': 'A-1-1',
        'destination_location': '선박-2', '_vehicle_id': vehicle_id,
    }
    inventory = {'cargos': [{
        'container_id': '6', 'location': trailer_location, 'floor': 1,
    }]}
    status = fleet_status()
    status['telemetry']['autonomy'] = {'arm1_ship_cache_ready': True}

    RealtimeLLMAgent._validate_active_step(
        step, move, inventory, status
    )


def test_arm1_ship_place_rejects_container_not_on_selected_trailer():
    step = {
        'type': 'arm1_pick_place', 'arm_id': 'arm1',
        'source_id': 6, 'destination_id': 19, 'vehicle_id': 'agv1',
    }
    move = {
        'container_id': '6', 'source_location': 'A-1-1',
        'destination_location': '선박-2', '_vehicle_id': 'agv1',
    }
    inventory = {'cargos': [{
        'container_id': '6', 'location': 'A-1-1', 'floor': 3,
    }]}
    status = fleet_status()
    status['telemetry']['autonomy'] = {'arm1_ship_cache_ready': True}

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='expected AMR1, DB=A-1-1',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, inventory, status
        )


def test_orphaned_arm_command_is_failed_after_restart_timeout():
    agent = object.__new__(RealtimeLLMAgent)
    agent.arm_command_orphan_timeout_sec = 10.0
    agent._cycle = SimpleNamespace(active_move={
        'container_id': '2',
        'destination_location': 'A-2-1',
        '_steps': [{
            'type': 'arm_transfer_to_slot', 'arm_id': 'arm2',
            'destination_slot': 'A-2-1',
        }],
        '_step_index': 0,
        '_current_command_id': 'arm-lost',
        '_dispatched_at': 1.0,
    })
    status = {
        'telemetry': {
            'arms': {'arm2': {'current_command_id': ''}},
            'last_arm_result': {'command_id': 'different'},
        }
    }
    inventory = {'cargos': [{
        'container_id': '2', 'location': 'AMR1', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(
        None, status, inventory
    )

    assert outcome == 'failed'
    assert '사라진 ARM 명령 arm-lost' in detail


def test_concurrent_arm_result_is_read_by_command_id_after_last_is_overwritten():
    agent = object.__new__(RealtimeLLMAgent)
    move = {
        'container_id': '2',
        'destination_location': '선박-2',
        '_steps': [{
            'type': 'arm1_scan_inbound', 'arm_id': 'arm1',
        }],
        '_step_index': 0,
        '_current_command_id': 'arm2-finished-first',
        '_dispatched_at': 1.0,
        '_vehicle_id': 'agv1',
    }
    agent._cycle = SimpleNamespace(
        active_move=move,
        active_mission_id='mission-1',
    )
    agent._save_cycle = lambda: None
    status = {'telemetry': {
        'arms': {'arm1': {'current_command_id': ''}},
        'last_arm_result': {
            'command_id': 'arm1-finished-last', 'success': True,
        },
        'arm_results': {
            'arm2-finished-first': {
                'command_id': 'arm2-finished-first', 'success': True,
            },
            'arm1-finished-last': {
                'command_id': 'arm1-finished-last', 'success': True,
            },
        },
    }}
    inventory = {'cargos': [{
        'container_id': '2', 'location': '선박-2', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(None, status, inventory)

    assert (outcome, detail) == ('completed', '')


def test_execution_details_show_every_physical_step():
    move = {
        '_step_index': 1,
        '_steps': [
            {'type': 'zone_navigation', 'zone': 'B-1'},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                'source_id': 6, 'destination_id': 10,
            },
        ],
    }

    all_steps = json.loads(RealtimeLLMAgent._execution_steps_json(move))
    current = json.loads(RealtimeLLMAgent._current_step_json(move))

    assert [step['type'] for step in all_steps] == [
        'zone_navigation', 'arm1_pick_place'
    ]
    assert current['sequence'] == 2
    assert current['total'] == 2
    assert current['source_id'] == 6


def test_missing_navigation_command_is_resent_when_not_arrived():
    agent = object.__new__(RealtimeLLMAgent)
    agent.nav_command_retry_grace_sec = 1.0
    agent.nav_command_max_resends = 2
    move = {
        'container_id': '6',
        '_vehicle_id': 'agv1',
        '_steps': [{
            'type': 'zone_navigation', 'zone': 'B-1',
            'vehicle_id': 'agv1',
        }],
        '_step_index': 0,
        '_current_command_id': 'nav-lost',
        '_dispatched_at': 1.0,
        '_nav_missing_since': 1.0,
        '_step_retry_counts': {},
    }
    agent._cycle = SimpleNamespace(active_move=move)
    agent._save_cycle = lambda: None
    dispatched = []
    agent._dispatch_active_step = lambda client: dispatched.append(client)
    status = {'telemetry': {'vehicles': {'agv1': {
        'state': 'READY', 'current_command_id': '', 'locked_zone': '',
    }}}}
    inventory = {'cargos': [{
        'container_id': '6', 'location': '선박-4', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(
        'client', status, inventory
    )

    assert (outcome, detail) == ('waiting', '')
    assert dispatched == ['client']
    assert move['_step_retry_counts'] == {'0': 1}
    assert move['_current_command_id'] == ''


def test_a_navigation_does_not_advance_before_exact_command_result():
    agent = object.__new__(RealtimeLLMAgent)
    move = {
        'container_id': '6',
        '_vehicle_id': 'agv1',
        '_steps': [{
            'type': 'zone_navigation', 'zone': 'A',
            'vehicle_id': 'agv1',
        }],
        '_step_index': 0,
        '_current_command_id': 'nav-aligning',
        '_dispatched_at': time.time() - 2.0,
        '_nav_missing_since': 0.0,
        '_step_retry_counts': {},
    }
    agent._cycle = SimpleNamespace(active_move=move)
    status = {'telemetry': {
        'vehicles': {'agv1': {
            'state': 'READY',
            'current_command_id': '',
            'locked_zone': 'A',
        }},
        'navigation_results': {},
    }}
    inventory = {'cargos': [{
        'container_id': '6', 'location': 'AMR1', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(None, status, inventory)

    assert (outcome, detail) == ('waiting', '')
    assert move['_step_index'] == 0


def test_a_navigation_advances_after_exact_success_result():
    agent = object.__new__(RealtimeLLMAgent)
    move = {
        'container_id': '6',
        'destination_location': 'A-1-1',
        '_vehicle_id': 'agv1',
        '_steps': [{
            'type': 'zone_navigation', 'zone': 'A',
            'vehicle_id': 'agv1',
        }],
        '_step_index': 0,
        '_current_command_id': 'nav-aligned',
        '_dispatched_at': time.time() - 2.0,
        '_nav_missing_since': 0.0,
        '_step_retry_counts': {},
    }
    agent._cycle = SimpleNamespace(active_move=move)
    agent._save_cycle = lambda: None
    status = {'telemetry': {
        'vehicles': {'agv1': {
            'state': 'READY',
            'current_command_id': '',
            'locked_zone': 'A',
        }},
        'navigation_results': {
            'nav-aligned': {
                'command_id': 'nav-aligned', 'success': True,
            },
        },
    }}
    inventory = {'cargos': [{
        'container_id': '6', 'location': 'A-1-1', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(None, status, inventory)

    assert (outcome, detail) == ('completed', '')
    assert move['_step_index'] == 1


def test_navigation_resends_stop_at_finite_limit():
    agent = object.__new__(RealtimeLLMAgent)
    agent.nav_command_retry_grace_sec = 1.0
    agent.nav_command_max_resends = 2
    agent._cycle = SimpleNamespace(active_move={
        'container_id': '6', '_vehicle_id': 'agv1',
        '_steps': [{'type': 'zone_navigation', 'zone': 'B-1'}],
        '_step_index': 0, '_current_command_id': 'nav-third',
        '_dispatched_at': 1.0, '_nav_missing_since': 1.0,
        '_step_retry_counts': {'0': 2},
    })
    agent._save_cycle = lambda: None
    status = {'telemetry': {'vehicles': {'agv1': {
        'state': 'READY', 'current_command_id': '', 'locked_zone': '',
    }}}}
    inventory = {'cargos': [{
        'container_id': '6', 'location': '선박-4', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(
        None, status, inventory
    )

    assert outcome == 'failed'
    assert '2회 재전송 후 중단' in detail


def test_arrived_vehicle_skips_occluded_b1_yolo_lookup():
    agent = object.__new__(RealtimeLLMAgent)
    move = {
        'container_id': '6',
        'destination_location': 'A-1-1',
        '_vehicle_id': 'agv1',
        '_steps': [
            {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': 'agv1'},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                'source_id': 6, 'destination_id': 10,
                'vehicle_id': 'agv1',
            },
        ],
        '_step_index': 0,
        '_current_command_id': '',
        '_dispatched_at': 0.0,
        '_nav_missing_since': 0.0,
    }
    agent._cycle = SimpleNamespace(active_move=move)
    agent._save_cycle = lambda: None
    agent._validate_active_step = lambda *args: None
    dispatched = []
    agent._dispatch_active_step = lambda client: dispatched.append(
        agent._cycle.active_move['_steps'][
            agent._cycle.active_move['_step_index']
        ]['type']
    )
    status = {'telemetry': {'vehicles': {'agv1': {
        'state': 'READY', 'current_command_id': '', 'locked_zone': 'B-1',
    }}, 'last_arm_result': {}}}
    inventory = {'cargos': [{
        'container_id': '6', 'location': '선박-4', 'floor': 1,
    }]}

    outcome, detail = agent._advance_active_move(
        'client', status, inventory
    )

    assert (outcome, detail) == ('waiting', '')
    assert move['_step_index'] == 1
    assert dispatched == ['arm1_pick_place']


def test_reserved_but_navigating_vehicle_does_not_skip_yolo_goal():
    step = {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': 'agv1'}
    move = {'_vehicle_id': 'agv1'}
    status = {'telemetry': {'vehicles': {'agv1': {
        'state': 'NAVIGATING',
        'current_command_id': 'nav-1',
        'locked_zone': 'B-1',
    }}}}

    assert not RealtimeLLMAgent._navigation_step_already_reached(
        step, move, status
    )


def test_a_navigation_uses_registered_goal_when_yolo_label_is_missing():
    agent = object.__new__(RealtimeLLMAgent)
    agent.location_loader = lambda: {
        '창고 A': {'cctv_pixel': [104.5, 164.4]},
    }

    target, heading, mode = agent._zone_goal(
        'A', {'detections': []}, 640, 480,
        allow_registered_a=True,
    )

    assert target == {'x': 104.5, 'y': 164.4}
    assert heading == {'x': 104.5, 'y': 114.4}
    assert mode == 'parking_a'


def test_db_available_a_destination_enables_registered_navigation_fallback():
    move = {
        'container_id': '6',
        'source_location': '선박-2',
        'destination_location': 'A-1-1',
        'destination_floor': 1,
    }
    step = {'type': 'zone_navigation', 'zone': 'A'}
    snapshot = {'cargos': [{
        'container_id': '6', 'location': 'AMR1', 'floor': 1,
    }]}

    RealtimeLLMAgent._validate_active_step(
        step, move, snapshot, {'telemetry': {}}
    )

    assert move['_db_a_navigation_verified'] is True


def test_db_occupied_a_destination_rejects_wrong_floor_fallback():
    move = {
        'container_id': '6',
        'source_location': '선박-2',
        'destination_location': 'A-1-1',
        'destination_floor': 1,
    }
    step = {'type': 'zone_navigation', 'zone': 'A'}
    snapshot = {'cargos': [
        {'container_id': '6', 'location': 'AMR1', 'floor': 1},
        {'container_id': '2', 'location': 'A-1-1', 'floor': 1},
    ]}

    with pytest.raises(
        realtime_llm_agent.AutonomousPolicyError,
        match='현재 적재 가능한 상태가 아님',
    ):
        RealtimeLLMAgent._validate_active_step(
            step, move, snapshot, {'telemetry': {}}
        )


def test_autonomy_restart_reassesses_without_resetting_cycle_cargo():
    agent = RealtimeLLMAgent()
    agent._cycle.phase = 'WAITING_OPERATOR'
    agent._cycle.active_move = None
    agent._cycle.active_mission_id = ''
    agent._cycle.inbound_ids = ['4']
    agent._cycle.outbound_ids = ['6']
    agent._cycle.identical_failures = 3
    agent._cycle.failure_key = 'old failure'
    agent._cycle.last_error = 'YOLO에서 작업 구역 B-1을 찾지 못함'
    saved = []
    agent._save_cycle = lambda: saved.append(agent._cycle.phase)

    agent.stop_autonomous_policy()
    agent.start_autonomous_policy()

    assert agent._cycle.phase == 'REASSESSING_CURRENT_STATE'
    assert agent._cycle.inbound_ids == ['4']
    assert agent._cycle.outbound_ids == ['6']
    assert agent._cycle.identical_failures == 0
    assert agent._cycle.failure_key == ''
    assert saved == ['REASSESSING_CURRENT_STATE']
    snapshot = agent.snapshot()
    assert snapshot.enabled
    assert snapshot.phase == 'REASSESSING_CURRENT_STATE'
    assert '최신 DB·ROI·Fleet·ARM' in snapshot.last_decision


def test_autonomy_restart_preserves_inflight_move_for_live_reconciliation():
    agent = RealtimeLLMAgent()
    move = {
        'container_id': '4',
        '_step_index': 1,
        '_current_command_id': 'arm-running',
    }
    agent._cycle.phase = 'EXECUTING_MOVE'
    agent._cycle.active_move = move
    agent._save_cycle = lambda: None

    agent.stop_autonomous_policy()
    agent.start_autonomous_policy()

    assert agent._cycle.phase == 'EXECUTING_MOVE'
    assert agent._cycle.active_move is move


def test_outbound_clear_rejects_cargo_that_is_still_in_warehouse():
    agent = RealtimeLLMAgent()
    agent._cycle.outbound_ids = ['6']
    agent._cycle.outbound_seen_in_roi = True
    sent = []
    client = SimpleNamespace(send_inventory_movement=sent.append)

    submitted = agent._complete_outbound(client, {'cargos': [{
        'container_id': '6', 'location': 'A-1-1', 'floor': 3,
        'base_aruco_id': '8',
    }]})

    assert submitted == 0
    assert sent == []
    assert agent.snapshot().state == 'WAITING_FOR_DB_SYNC'
    assert 'A-1-1' in agent.snapshot().last_error


def test_exhausted_navigation_recovery_stops_without_replanning_loop():
    agent = RealtimeLLMAgent()
    agent._cycle.active_move = {
        'container_id': '6',
        'source_location': '선박-2',
        'destination_location': 'A-1-1',
    }
    agent._save_cycle = lambda: None

    agent._record_autonomous_failure(
        'agv1가 B-1에 도착하지 못해 이동 명령 0회 재전송 후 중단'
    )

    assert agent._cycle.phase == 'WAITING_OPERATOR'
    assert agent._cycle.identical_failures == 3
    assert agent.snapshot().state == 'WAITING_OPERATOR'


def test_arm_pick_failure_stops_after_same_step_resends_are_exhausted():
    agent = RealtimeLLMAgent()
    move = {
        'container_id': '6',
        'source_location': '선박-2',
        'destination_location': 'A-1-1',
        '_vehicle_id': 'agv1',
        '_step_index': 1,
        '_steps': [
            {'type': 'zone_navigation', 'zone': 'B-1'},
            {'type': 'arm1_pick_place', 'arm_id': 'arm1'},
        ],
        '_step_retry_counts': {'1': 2},
    }
    agent._cycle.active_move = move
    agent._cycle.active_moves = {'agv1': move}
    agent._save_cycle = lambda: None

    agent._record_autonomous_failure(
        "all station poses exhausted; missing ArUco=['pick']",
        move=move,
        vehicle_id='agv1',
    )

    assert agent._cycle.phase == 'WAITING_OPERATOR'
    assert agent._cycle.identical_failures == 3
    assert agent._cycle.active_moves == {}
    assert agent.snapshot().state == 'WAITING_OPERATOR'


def test_arm_marker_miss_schedules_retry_on_same_move_and_vehicle():
    agent = object.__new__(RealtimeLLMAgent)
    agent.arm_command_max_resends = 2
    agent.arm_command_retry_grace_sec = 2.0
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    move = {
        'container_id': '6',
        '_vehicle_id': 'agv1',
        '_step_index': 1,
        '_current_command_id': 'arm-failed',
        '_steps': [
            {'type': 'zone_navigation', 'zone': 'B-1'},
            {'type': 'arm1_pick_place', 'arm_id': 'arm1'},
        ],
        '_step_retry_counts': {},
    }
    agent._cycle.active_move = move
    agent._cycle.active_moves = {'agv1': move}
    agent._save_cycle = lambda: None
    updates = []
    agent._update_snapshot = lambda **kwargs: updates.append(kwargs)

    outcome, detail = agent._retry_failed_arm_step(
        object(), move, 'auto-test', fleet_status(), 1,
        "all station poses exhausted; missing ArUco=['pick']",
    )

    assert (outcome, detail) == ('waiting', '')
    assert move['_vehicle_id'] == 'agv1'
    assert move['_current_command_id'] == 'arm-failed'
    assert move['_arm_retry_not_before'] > time.time()
    assert '같은 ARM 작업 재시도 1/2' in updates[-1]['last_error']


def test_arm_marker_miss_retries_without_limit_by_default():
    agent = object.__new__(RealtimeLLMAgent)
    agent.arm_command_max_resends = -1
    agent.arm_command_retry_grace_sec = 2.0
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    move = {
        'container_id': '6',
        '_vehicle_id': 'agv1',
        '_step_index': 1,
        '_current_command_id': 'arm-failed-100',
        '_steps': [
            {'type': 'zone_navigation', 'zone': 'B-1'},
            {'type': 'arm1_pick_place', 'arm_id': 'arm1'},
        ],
        '_step_retry_counts': {'1': 100},
    }
    agent._cycle.active_move = move
    agent._cycle.active_moves = {'agv1': move}
    agent._save_cycle = lambda: None
    updates = []
    agent._update_snapshot = lambda **kwargs: updates.append(kwargs)

    outcome, detail = agent._retry_failed_arm_step(
        object(), move, 'auto-test', fleet_status(), 1,
        "all station poses exhausted; missing ArUco=['pick']",
    )

    assert (outcome, detail) == ('waiting', '')
    assert move['_current_command_id'] == 'arm-failed-100'
    assert move['_arm_retry_not_before'] > time.time()
    assert '같은 ARM 작업 재시도 101회' in updates[-1]['last_error']


def test_arm_retry_uses_a_new_mission_after_previous_mission_failed():
    class Client:
        def __init__(self):
            self.requests = []

        def send_arm_command(self, **kwargs):
            self.requests.append(kwargs)
            return {'command_id': 'arm-retry-command'}

    agent = object.__new__(RealtimeLLMAgent)
    agent.arm_command_max_resends = -1
    agent.arm_command_retry_grace_sec = 2.0
    agent._cycle = realtime_llm_agent.AutonomousCycle()
    move = {
        'container_id': '6',
        '_vehicle_id': 'agv1',
        '_step_index': 0,
        '_current_command_id': 'arm-failed',
        '_mission_id': 'auto-original',
        '_root_mission_id': 'auto-original',
        '_arm_retry_not_before': time.time() - 1.0,
        '_steps': [{
            'type': 'arm1_pick_place',
            'arm_id': 'arm1',
            'source_id': 6,
            'destination_id': 10,
            'vehicle_id': 'agv1',
            'final_for_vehicle': True,
        }],
        '_step_retry_counts': {},
    }
    agent._cycle.active_move = move
    agent._cycle.active_moves = {'agv1': move}
    agent._save_cycle = lambda: None
    agent._update_snapshot = lambda **_kwargs: None
    client = Client()

    outcome, detail = agent._retry_failed_arm_step(
        client, move, 'auto-original', fleet_status(), 0,
        "all station poses exhausted; missing ArUco=['pick']",
    )

    assert (outcome, detail) == ('waiting', '')
    retry_mission = client.requests[0]['mission_id']
    assert retry_mission.startswith('auto-original-arm-retry-1-')
    assert retry_mission != 'auto-original'
    assert move['_mission_id'] == retry_mission
    assert move['_current_command_id'] == 'arm-retry-command'


def test_internal_move_reconciliation_preserves_slot_floor_and_support():
    event = RealtimeLLMAgent._internal_move_sync_event(
        {
            'container_id': '1',
            'destination_location': 'A-2-1',
            'destination_floor': 2,
            'destination_base_aruco_id': '0',
        },
        {
            'operation_id': 'id-transfer-1-to-0',
            'command_id': 'arm-command-1',
            'mission_id': 'mission-1',
        },
    )

    assert event['operation_id'] == 'id-transfer-1-to-0'
    assert event['container_id'] == '1'
    assert event['source_location'] == ''
    assert event['destination_location'] == 'A-2-1'
    assert event['destination_floor'] == 2
    assert event['destination_base_aruco_id'] == '0'
