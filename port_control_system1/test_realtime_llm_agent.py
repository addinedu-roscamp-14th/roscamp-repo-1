"""Tests for event-driven realtime VLM supervision helpers."""

import json
from types import SimpleNamespace

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


def test_process_restart_starts_with_a_fresh_autonomous_cycle():
    store = PreviousProcessCycleStore()

    agent = RealtimeLLMAgent(cycle_store=store)

    assert store.load_calls == 0
    assert agent._cycle.phase == 'WAITING_FOR_INBOUND'
    assert agent._cycle.inbound_ids == []
    assert agent._cycle.outbound_ids == []
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


def test_exhausted_navigation_recovery_stops_without_replanning_loop():
    agent = RealtimeLLMAgent()
    agent._cycle.active_move = {
        'container_id': '6',
        'source_location': '선박-1',
        'destination_location': 'A-1-1',
    }
    agent._save_cycle = lambda: None

    agent._record_autonomous_failure(
        'agv1가 B-1에 도착하지 못해 이동 명령 0회 재전송 후 중단'
    )

    assert agent._cycle.phase == 'WAITING_OPERATOR'
    assert agent._cycle.identical_failures == 3
    assert agent.snapshot().state == 'WAITING_OPERATOR'
