"""Tests for event-driven realtime VLM supervision helpers."""

from realtime_llm_agent import (
    RealtimeLLMAgent,
    action_signature,
    observation_signature,
    supervision_command,
)


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
