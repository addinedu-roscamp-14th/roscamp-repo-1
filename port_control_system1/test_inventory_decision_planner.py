"""Tests for remote-inventory LLM planning and deterministic validation."""

from datetime import datetime, timezone

from inventory_client import InventoryClient, InventoryClientError
from inventory_decision_planner import InventoryDecisionPlanner


NOW = datetime(2026, 8, 11, 3, 0, 0, tzinfo=timezone.utc)
LOCATIONS = ['A-1-1', 'A-2-1', 'B-1', '임시 버퍼']


def make_snapshot(cargos):
    client = InventoryClient(connect=lambda **kwargs: None, now=lambda: NOW)
    rows = [(
        item['name'], item['location'], item['container_id'],
        item['cargo_type'], item['note'], item['base_aruco_id'], item['floor'],
    ) for item in cargos]
    return client.snapshot_from_rows(rows)


def cargo(name, container_id, location, floor, base=''):
    return {
        'name': name,
        'location': location,
        'container_id': container_id,
        'cargo_type': '컨테이너',
        'note': '',
        'base_aruco_id': base,
        'floor': floor,
    }


def move(sequence, container_id, name, source, source_floor, destination,
         destination_floor=1, base='', reason='목표 달성'):
    return {
        'sequence': sequence,
        'container_id': container_id,
        'container_name': name,
        'source_location': source,
        'source_floor': source_floor,
        'destination_location': destination,
        'destination_floor': destination_floor,
        'destination_base_aruco_id': base,
        'reason': reason,
    }


def test_single_move_plan_is_normalized():
    snapshot = make_snapshot([cargo('C0', '0', 'A-1-1', 1, '11')])
    planner = InventoryDecisionPlanner(llm=lambda prompt: {
        'status': 'ready',
        'moves': [move(1, '0', 'C0', 'A-1-1', 1, 'B-1')],
        'summary': 'C0 출고',
    })

    result = planner.plan_snapshot('C0를 B-1로 옮겨', snapshot, LOCATIONS)

    assert result['status'] == 'ready'
    assert result['snapshot_id'].startswith('sql-')
    assert result['moves'][0]['destination_location'] == 'B-1'
    assert result['error'] == ''


def test_yard_shuffle_can_move_blocker_then_restore_it():
    snapshot = make_snapshot([
        cargo('C0', '0', 'A-1-1', 1, '11'),
        cargo('C1', '1', 'A-1-1', 2, '0'),
    ])
    plan = {
        'status': 'ready',
        'moves': [
            move(1, '1', 'C1', 'A-1-1', 2, '임시 버퍼', reason='C0 접근 확보'),
            move(2, '0', 'C0', 'A-1-1', 1, 'B-1', reason='C0 출고'),
            move(3, '1', 'C1', '임시 버퍼', 1, 'B-1', 2, '0', '적층 복원'),
        ],
        'summary': '방해물을 치운 뒤 전체 목표 완료',
    }
    planner = InventoryDecisionPlanner(llm=lambda prompt: plan)

    result = planner.plan_snapshot('C0를 출고하고 C1을 위에 적재', snapshot, LOCATIONS)

    assert result['status'] == 'ready'
    assert [item['container_id'] for item in result['moves']] == ['1', '0', '1']


def test_moving_blocked_container_is_rejected_as_a_whole_plan():
    snapshot = make_snapshot([
        cargo('C0', '0', 'A-1-1', 1, '11'),
        cargo('C1', '1', 'A-1-1', 2, '0'),
    ])
    planner = InventoryDecisionPlanner(llm=lambda prompt: {
        'status': 'ready',
        'moves': [move(1, '0', 'C0', 'A-1-1', 1, 'B-1')],
        'summary': '',
    })

    result = planner.plan_snapshot('C0 출고', snapshot, LOCATIONS)

    assert result['status'] == 'error'
    assert result['moves'] == []
    assert 'blocked by' in result['error']


def test_unknown_container_and_bad_source_are_rejected():
    snapshot = make_snapshot([cargo('C0', '0', 'A-1-1', 1, '11')])
    unknown = InventoryDecisionPlanner(llm=lambda prompt: {
        'status': 'ready',
        'moves': [move(1, '9', 'C9', 'A-1-1', 1, 'B-1')],
    }).plan_snapshot('출고', snapshot, LOCATIONS)
    bad_source = InventoryDecisionPlanner(llm=lambda prompt: {
        'status': 'ready',
        'moves': [move(1, '0', 'C0', 'A-2-1', 1, 'B-1')],
    }).plan_snapshot('출고', snapshot, LOCATIONS)

    assert unknown['status'] == 'error'
    assert bad_source['status'] == 'error'
    assert unknown['moves'] == bad_source['moves'] == []


def test_fetch_failure_does_not_call_llm():
    calls = []

    class FailedClient:
        @staticmethod
        def fetch_snapshot():
            raise InventoryClientError('PostgreSQL offline')

    planner = InventoryDecisionPlanner(
        inventory_client=FailedClient(),
        llm=lambda prompt: calls.append(prompt),
    )

    result = planner.plan('C0 출고', LOCATIONS)

    assert result['status'] == 'error'
    assert result['moves'] == []
    assert calls == []


def test_no_action_has_stable_public_shape():
    snapshot = make_snapshot([cargo('C0', '0', 'B-1', 1)])
    planner = InventoryDecisionPlanner(llm=lambda prompt: {
        'status': 'no_action',
        'moves': [],
        'summary': '이미 목표 상태',
    })

    result = planner.plan_snapshot('C0를 B-1로', snapshot, LOCATIONS)

    assert set(result) == {
        'schema_version', 'plan_id', 'snapshot_id', 'objective', 'status',
        'moves', 'summary', 'error',
    }
    assert result['status'] == 'no_action'


def test_single_move_planning_uses_only_first_valid_llm_choice():
    snapshot = make_snapshot([cargo('C0', '0', 'A-1-1', 1, '11')])
    responses = [
        {
            'status': 'ready',
            'moves': [
                move(1, '0', 'C0', 'A-1-1', 1, 'B-1'),
                move(2, '0', 'C0', 'B-1', 1, '임시 버퍼'),
            ],
            'summary': '잘못된 다중 이동',
        },
        {
            'status': 'ready',
            'moves': [move(1, '0', 'C0', 'A-1-1', 1, 'B-1')],
            'summary': '교정된 단일 이동',
        },
    ]
    planner = InventoryDecisionPlanner(llm=lambda _prompt: responses.pop(0))

    result = planner.plan_single_move_snapshot(
        'C0 한 건 이동', snapshot, LOCATIONS
    )

    assert result['status'] == 'ready'
    assert len(result['moves']) == 1
    assert len(responses) == 1


def test_single_move_planning_retries_spurious_no_action():
    snapshot = make_snapshot([cargo('C0', '0', 'A-1-1', 1, '11')])
    responses = [
        {'status': 'no_action', 'moves': [], 'summary': '잘못된 대기'},
        {
            'status': 'ready',
            'moves': [move(1, '0', 'C0', 'A-1-1', 1, 'B-1')],
            'summary': '재시도 성공',
        },
    ]
    planner = InventoryDecisionPlanner(llm=lambda _prompt: responses.pop(0))

    result = planner.plan_single_move_snapshot(
        'C0 한 건 이동', snapshot, LOCATIONS
    )

    assert result['status'] == 'ready'
    assert len(result['moves']) == 1
    assert not responses


def test_single_move_normalizes_db_derived_mechanical_fields():
    snapshot = make_snapshot([
        cargo('C0', '0', 'A-1-1', 1, '11'),
        cargo('C1', '1', 'B-1', 1, ''),
    ])
    planner = InventoryDecisionPlanner(llm=lambda _prompt: {
        'status': 'ready',
        'moves': [
            move(
                7, '0', '잘못된 이름', '잘못된 출발지', 9,
                'B-1', 9, '잘못된 기반', 'LLM이 B-1을 선택',
            ),
            move(8, '1', 'C1', 'B-1', 1, '임시 버퍼'),
        ],
        'summary': '선택 자체는 유효함',
    })

    result = planner.plan_single_move_snapshot(
        '한 건 이동', snapshot, LOCATIONS
    )

    assert result['status'] == 'ready'
    assert result['moves'] == [{
        'sequence': 1,
        'container_id': '0',
        'container_name': 'C0',
        'source_location': 'A-1-1',
        'source_floor': 1,
        'destination_location': 'B-1',
        'destination_floor': 2,
        'destination_base_aruco_id': '1',
        'reason': 'LLM이 B-1을 선택',
    }]
