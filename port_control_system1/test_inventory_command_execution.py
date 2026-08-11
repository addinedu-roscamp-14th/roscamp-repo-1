"""DB-grounding checks for executable natural-language ARM plans."""

from llm_command_parser import inventory_workflow_issues


SNAPSHOT = {
    'schema_version': '1.0',
    'snapshot_id': 'sql-test',
    'generated_at': '2026-08-11T00:00:00Z',
    'cargos': [
        {
            'name': '컨테이너_C2',
            'location': 'A-2-1',
            'container_id': '2',
            'cargo_type': '컨테이너',
            'note': '',
            'base_aruco_id': '13',
            'floor': 1,
        },
        {
            'name': '컨테이너_C8',
            'location': 'A-2-1',
            'container_id': '8',
            'cargo_type': '특수화물',
            'note': '',
            'base_aruco_id': '2',
            'floor': 2,
        },
        {
            'name': '컨테이너_C6',
            'location': 'A-2-1',
            'container_id': '6',
            'cargo_type': '특수화물',
            'note': '',
            'base_aruco_id': '8',
            'floor': 3,
        },
        {
            'name': '컨테이너_C0',
            'location': '항구',
            'container_id': '0',
            'cargo_type': '컨테이너',
            'note': '',
            'base_aruco_id': '11',
            'floor': 1,
        },
    ],
}


def _load_plan(source_id):
    return {
        'execution_mode': 'sequential',
        'actions': [{
            'type': 'arm_load_to_trailer',
            'arm_id': 'arm2',
            'source_id': source_id,
            'vehicle_id': 'agv1',
            'final_for_vehicle': True,
        }],
    }


def test_location_only_load_accepts_db_top_container():
    issues = inventory_workflow_issues(
        'a-2-1에 있는 컨테이너를 amr1에 실어줘',
        _load_plan(6),
        SNAPSHOT,
    )

    assert issues == []


def test_location_only_load_rejects_guessed_source_zero():
    issues = inventory_workflow_issues(
        'a-2-1에 있는 컨테이너를 amr1에 실어줘',
        _load_plan(0),
        SNAPSHOT,
    )

    assert any('최상단' in issue and 'container_id=6' in issue for issue in issues)


def test_arm_load_fails_closed_without_inventory():
    issues = inventory_workflow_issues(
        'a-2-1에 있는 컨테이너를 amr2에 실어줘',
        _load_plan(6),
        None,
    )

    assert issues == ['ARM 상차 판단에 필요한 PostgreSQL 재고 스냅샷이 없음']


def test_omitted_arm_step_is_rejected_for_loading_request():
    issues = inventory_workflow_issues(
        'a-2-1 컨테이너를 amr1에 실은다음 b-1로 보내줘',
        {
            'actions': [{
                'type': 'visual_navigation',
                'detection_index': 0,
                'vehicle_id': 'agv1',
            }],
        },
        SNAPSHOT,
    )

    assert issues == ['사용자 상차 요청에 arm_load_to_trailer 단계가 없음']


def test_loading_request_without_db_blocks_even_when_llm_returns_unknown():
    issues = inventory_workflow_issues(
        'a-2-1 컨테이너를 amr1에 실은다음 b-1로 보내줘',
        {'actions': [{'type': 'unknown', 'reason': 'DB 없음'}]},
        None,
    )

    assert issues == ['ARM 상차 판단에 필요한 PostgreSQL 재고 스냅샷이 없음']


def test_container_zero_remains_a_valid_database_id():
    issues = inventory_workflow_issues(
        '0번 컨테이너를 amr1에 실어줘',
        _load_plan(0),
        SNAPSHOT,
    )

    assert issues == []


def test_arm1_dynamic_source_must_exist_in_database():
    issues = inventory_workflow_issues(
        'ARM1으로 42번 컨테이너를 9번 마커에 놓아줘',
        {
            'actions': [{
                'type': 'arm1_pick_place',
                'arm_id': 'arm1',
                'source_id': 42,
                'destination_id': 9,
            }],
        },
        SNAPSHOT,
    )

    assert any('ARM1 source_id=42가 DB에 존재하지 않음' in item for item in issues)


def test_arm1_location_request_requires_top_database_container():
    issues = inventory_workflow_issues(
        'ARM1으로 A-2-1의 컨테이너를 9번 마커에 놓아줘',
        {
            'actions': [{
                'type': 'arm1_pick_place',
                'arm_id': 'arm1',
                'source_id': 2,
                'destination_id': 9,
            }],
        },
        SNAPSHOT,
    )

    assert any('최상단 floor=3 아래에 있음' in item for item in issues)


def test_arm1_explicit_buried_source_is_rejected_from_db_stack():
    issues = inventory_workflow_issues(
        'ARM1으로 2번 컨테이너를 9번 마커에 놓아줘',
        {
            'actions': [{
                'type': 'arm1_pick_place',
                'arm_id': 'arm1',
                'source_id': 2,
                'destination_id': 9,
            }],
        },
        SNAPSHOT,
    )

    assert any('최상단 floor=3 아래에 있음' in item for item in issues)
