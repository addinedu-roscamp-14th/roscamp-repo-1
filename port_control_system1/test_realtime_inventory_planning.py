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
