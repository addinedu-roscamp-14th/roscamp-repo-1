"""Regression tests for the non-dispatching realtime inventory mode."""

from datetime import datetime, timezone

from inventory_client import InventoryClient
import realtime_llm_agent
from realtime_llm_agent import RealtimeLLMAgent


def snapshot():
    client = InventoryClient(
        connect=lambda **kwargs: None,
        now=lambda: datetime(2026, 8, 11, 3, 0, tzinfo=timezone.utc),
    )
    return client.snapshot_from_rows([
        ('C0', 'A-1-1', '0', '컨테이너', '', '11', 1),
    ])


def test_inventory_evaluation_never_constructs_control_client(monkeypatch):
    snapshot_id = snapshot().snapshot_id
    calls = []

    class SnapshotClient:
        @staticmethod
        def fetch_snapshot():
            return snapshot()

    class Planner:
        inventory_client = SnapshotClient()

        @staticmethod
        def plan_snapshot(objective, inventory, locations):
            calls.append((objective, inventory.snapshot_id, locations))
            return {
                'schema_version': '1.0',
                'plan_id': 'plan-test',
                'snapshot_id': inventory.snapshot_id,
                'objective': objective,
                'status': 'no_action',
                'moves': [],
                'summary': '이미 목표 상태',
                'error': '',
            }

    def forbidden_control_client(*args, **kwargs):
        raise AssertionError('inventory planning must not call central control')

    monkeypatch.setattr(
        realtime_llm_agent, 'CentralControlClient', forbidden_control_client
    )
    agent = RealtimeLLMAgent(
        inventory_planner=Planner(),
        location_loader=lambda: {'A-1-1': {}, 'B-1': {}},
    )

    result = agent._evaluate_inventory('C0 상태를 계속 확인')

    assert result['status'] == 'no_action'
    assert calls == [
        ('C0 상태를 계속 확인', snapshot_id, ['A-1-1', 'B-1'])
    ]
    state = agent.snapshot()
    assert state.dispatched_actions == 0
    assert state.state == 'MONITORING'


def test_same_snapshot_is_skipped_until_heartbeat(monkeypatch):
    snapshot_id = snapshot().snapshot_id
    class SnapshotClient:
        @staticmethod
        def fetch_snapshot():
            return snapshot()

    class Planner:
        inventory_client = SnapshotClient()

        @staticmethod
        def plan_snapshot(*args):
            raise AssertionError('unchanged snapshot should not be replanned')

    agent = RealtimeLLMAgent(inventory_planner=Planner())
    agent._last_inventory_snapshot_id = snapshot_id
    agent._last_evaluation_monotonic = __import__('time').monotonic()

    assert agent._evaluate_inventory('목표') is None


def test_db_plan_execution_dispatches_first_internal_move(monkeypatch):
    inventory = snapshot()
    plan = {
        'schema_version': '1.0',
        'plan_id': 'plan-execute',
        'snapshot_id': inventory.snapshot_id,
        'objective': 'C0를 A-1-2로 이동',
        'status': 'ready',
        'moves': [{
            'sequence': 1,
            'container_id': '0',
            'container_name': 'C0',
            'source_location': 'A-1-1',
            'source_floor': 1,
            'destination_location': 'A-1-2',
            'destination_floor': 1,
            'destination_base_aruco_id': '',
            'reason': '테스트',
        }],
        'summary': '한 건 이동',
        'error': '',
    }
    sent = []

    class SnapshotClient:
        @staticmethod
        def fetch_snapshot():
            return inventory

    class Planner:
        inventory_client = SnapshotClient()

        @staticmethod
        def plan_snapshot(*_args):
            raise AssertionError('matching initial plan must be used first')

    class Store:
        @staticmethod
        def load():
            return realtime_llm_agent.AutonomousCycle()

        @staticmethod
        def save(_cycle):
            return None

    class Control:
        @staticmethod
        def status():
            return {'telemetry': {
                'inventory_sync': {'state': 'READY', 'pending_count': 0},
                'vehicles': {},
                'arms': {'arm2': {}},
                'arm_results': {},
            }}

        @staticmethod
        def send_arm_command(**payload):
            sent.append(payload)
            return {'command_id': 'arm-db-plan-1'}

    monkeypatch.setattr(
        realtime_llm_agent, 'CentralControlClient',
        lambda timeout_sec=3.0: Control(),
    )
    agent = RealtimeLLMAgent(
        inventory_planner=Planner(), cycle_store=Store()
    )

    agent.start_inventory_plan_execution(plan['objective'], plan)
    result = agent._evaluate_inventory_execution()

    assert result == plan
    assert sent[0]['operation'] == 'transfer_by_id'
    assert sent[0]['source_id'] == 0
    assert sent[0]['destination_id'] == 12
    assert sent[0]['vehicle_id'] == ''
    state = agent.snapshot()
    assert state.mode == 'inventory_execute'
    assert state.state == 'EXECUTING'
    assert state.active_command == 'arm-db-plan-1'
