"""Tests for image forwarding to the Ollama-compatible VLM."""

import sys
import types

from llm_command_parser import (
    _finalize_navigation_result,
    _mentioned_vehicle_ids,
    _SYSTEM_PROMPT_TEMPLATE,
    normalize_navigation_result,
    parse_command_with_llm,
    resolve_execution_mode,
)


DETECTIONS = [
    {
        'detection_index': 0,
        'label': 'B-1',
        'bbox_xyxy': [440, 240, 540, 340],
        'center_xy': [490, 290],
        'heading_deg': -90.0,
    },
    {
        'detection_index': 1,
        'label': 'A-2',
        'bbox_xyxy': [200, 100, 260, 180],
        'center_xy': [230, 140],
    },
]


def test_image_is_forwarded_to_vlm(monkeypatch):
    """The current JPEG must be attached to the user message."""
    captured = {}

    class FakeClient:
        def __init__(self, host, timeout):
            captured['host'] = host
            captured['timeout'] = timeout

        def chat(self, **kwargs):
            captured['kwargs'] = kwargs
            return {
                'message': {
                    'content': (
                        '{"actions":[{"type":"pixel_navigation",'
                        '"target":{"x":100,"y":200},'
                        '"heading":{"x":150,"y":200}}]}'
                    )
                }
            }

    monkeypatch.setitem(
        sys.modules,
        'ollama',
        types.SimpleNamespace(Client=FakeClient),
    )
    result = parse_command_with_llm(
        '빈 공간으로 이동',
        [],
        [],
        [],
        image_jpeg=b'jpeg-bytes',
        image_width=640,
        image_height=480,
        yolo_detections=[
            {
                'detection_index': 0,
                'label': 'car_blue',
                'bbox_xyxy': [100, 120, 180, 200],
                'center_xy': [140, 160],
            }
        ],
    )

    user_message = captured['kwargs']['messages'][1]
    system_message = captured['kwargs']['messages'][0]
    assert user_message['images'] == [b'jpeg-bytes']
    assert captured['kwargs']['options']['num_ctx'] == 8192
    assert '"detection_index":0' in user_message['content']
    assert '"label":"car_blue"' in user_message['content']
    assert (
        '"execution_mode": "<parallel|sequential>"'
        in system_message['content']
    )
    assert '기본적으로 execution_mode를 "parallel"' in system_message['content']
    assert result['actions'][0]['type'] == 'pixel_navigation'


def test_llm_revises_arm_plan_when_a_zone_arrival_is_missing(monkeypatch):
    responses = [
        {
            'message': {
                'content': (
                    '{"execution_mode":"sequential","actions":['
                    '{"type":"arm_transfer_to_slot","arm_id":"arm2",'
                    '"destination_slot":"A-1-2","vehicle_id":"agv1",'
                    '"final_for_vehicle":true}]}'
                )
            }
        },
        {
            'message': {
                'content': (
                    '{"execution_mode":"sequential","actions":['
                    '{"type":"visual_navigation","detection_index":1,'
                    '"approach_side":"bottom","vehicle_id":"agv1"},'
                    '{"type":"arm_transfer_to_slot","arm_id":"arm2",'
                    '"destination_slot":"A-1-2","vehicle_id":"agv1",'
                    '"final_for_vehicle":true}]}'
                )
            }
        },
    ]
    captured = []

    class FakeClient:
        def __init__(self, **_kwargs):
            pass

        def chat(self, **kwargs):
            captured.append(kwargs)
            return responses[len(captured) - 1]

    monkeypatch.setitem(
        sys.modules,
        'ollama',
        types.SimpleNamespace(Client=FakeClient),
    )

    result = parse_command_with_llm(
        'amr1에 실려있는 1번 컨테이너를 a-1-2에 내려줘',
        [],
        [],
        [],
        yolo_detections=DETECTIONS,
    )

    assert len(captured) == 2
    assert '물리적 선행조건' in captured[1]['messages'][-1]['content']
    assert [action['type'] for action in result['actions']] == [
        'visual_navigation', 'arm_transfer_to_slot'
    ]


def test_independent_vehicle_actions_default_to_parallel():
    result = {
        'actions': [
            {'type': 'visual_navigation', 'vehicle_id': 'agv1'},
            {'type': 'visual_navigation', 'vehicle_id': 'agv2'},
        ]
    }

    assert (
        resolve_execution_mode('각 차량을 지정 구역으로 보내', result)
        == 'parallel'
    )


def test_explicit_sequence_or_same_vehicle_stays_sequential():
    distinct = {
        'actions': [
            {'type': 'visual_navigation', 'vehicle_id': 'agv1'},
            {'type': 'visual_navigation', 'vehicle_id': 'agv2'},
        ]
    }
    repeated = {
        'execution_mode': 'parallel',
        'actions': [
            {'type': 'visual_navigation', 'vehicle_id': 'agv1'},
            {'type': 'pixel_navigation', 'vehicle_id': 'agv1'},
        ],
    }

    assert (
        resolve_execution_mode('agv1 도착 후 agv2를 보내', distinct)
        == 'sequential'
    )
    assert resolve_execution_mode('두 목표로 보내', repeated) == 'sequential'


def test_all_vehicle_request_expands_one_navigation_action():
    result = normalize_navigation_result(
        '모든 차량을 B-1로 보내',
        {
            'actions': [
                {
                    'type': 'visual_navigation',
                    'detection_index': 0,
                    'approach_side': 'bottom',
                    'vehicle_id': '',
                }
            ]
        },
        DETECTIONS,
    )

    assert result['execution_mode'] == 'parallel'
    assert [action['vehicle_id'] for action in result['actions']] == [
        'agv1', 'agv2'
    ]


def test_idle_fleet_park_request_expands_to_both_vehicles():
    """"유휴 차량들" addresses the fleet, so both vehicles must be parked.

    Only one park_command used to be emitted, and an empty vehicle_id makes
    the dispatcher pick a single vehicle - so the second AGV never parked.
    """
    result = normalize_navigation_result(
        '유휴차량들은 주차해줘',
        {'actions': [{'type': 'park_command', 'vehicle_id': ''}]},
        DETECTIONS,
    )

    assert result['execution_mode'] == 'parallel'
    assert [action['vehicle_id'] for action in result['actions']] == [
        'agv1', 'agv2'
    ]
    assert all(
        action['type'] == 'park_command' for action in result['actions']
    )


def test_broad_harbor_command_recovers_unknown_as_b1_navigation():
    result = normalize_navigation_result(
        '노란 차 상차하러 보내줘',
        {'actions': [{'type': 'unknown', 'reason': '목적지 불명확'}]},
        DETECTIONS,
    )

    assert result == {
        'actions': [
            {
                'type': 'visual_navigation',
                'detection_index': 0,
                'approach_side': 'bottom',
                'vehicle_id': 'agv2',
            }
        ]
    }


def test_park_command_infers_vehicle_id_from_color():
    """Yellow is AMR 2 (agv2) and blue is AMR 1 (agv1)."""
    result = normalize_navigation_result(
        '노란 차 주차해줘',
        {'actions': [{'type': 'park_command', 'vehicle_id': ''}]},
        DETECTIONS,
    )

    assert result == {
        'actions': [
            {'type': 'park_command', 'vehicle_id': 'agv2'},
        ]
    }

    blue = normalize_navigation_result(
        '파란 차 주차해줘',
        {'actions': [{'type': 'park_command', 'vehicle_id': ''}]},
        DETECTIONS,
    )

    assert blue == {
        'actions': [
            {'type': 'park_command', 'vehicle_id': 'agv1'},
        ]
    }


def test_generic_park_word_is_not_repaired_into_b1_navigation():
    """'주차' alone must not be recovered as B-1 - that name is reserved
    for the real parking-spot command now, so an unknown result naming
    only 'park' (with no zone-specific wording) should stay unknown."""
    result = normalize_navigation_result(
        '주차해줘',
        {'actions': [{'type': 'unknown', 'reason': '목적지 불명확'}]},
        DETECTIONS,
    )

    assert result['actions'][0]['type'] == 'unknown'


def test_broad_a_zone_command_uses_visible_a_zone():
    result = normalize_navigation_result(
        '차량 한 대를 A구역에 대기시켜',
        {'actions': [{'type': 'unknown', 'reason': '구역 번호 없음'}]},
        DETECTIONS,
    )

    action = result['actions'][0]
    assert action['type'] == 'visual_navigation'
    assert action['detection_index'] == 1
    assert action['vehicle_id'] == ''


def test_visual_action_missing_fields_is_repaired_from_command():
    result = normalize_navigation_result(
        '파란 차를 항구로 이동',
        {
            'actions': [
                {
                    'type': 'visual_navigation',
                    'target_label': 'B-1',
                }
            ]
        },
        DETECTIONS,
    )

    action = result['actions'][0]
    assert action['detection_index'] == 0
    assert action['approach_side'] == 'bottom'
    assert action['vehicle_id'] == 'agv1'


def test_single_registered_harbor_travel_prefers_visible_b1():
    result = normalize_navigation_result(
        '항구로 이동',
        {'actions': [{'type': 'travel', 'stops': ['항구']}]},
        DETECTIONS,
    )

    action = result['actions'][0]
    assert action['type'] == 'visual_navigation'
    assert action['detection_index'] == 0


def test_multi_stop_travel_is_not_replaced():
    original = {
        'actions': [
            {
                'type': 'travel',
                'stops': ['창고 회차지점', '항구'],
            }
        ]
    }
    assert normalize_navigation_result(
        '창고 회차지점을 거쳐 항구로 이동',
        original,
        DETECTIONS,
    ) == original


def test_generic_or_negated_command_does_not_invent_target():
    unknown = {'actions': [{'type': 'unknown', 'reason': '목적지 없음'}]}

    assert normalize_navigation_result(
        '차량 상태를 보여줘',
        unknown,
        DETECTIONS,
    ) == unknown
    assert normalize_navigation_result(
        'B-1로 가지 마',
        unknown,
        DETECTIONS,
    ) == unknown
    assert normalize_navigation_result(
        'A구역이 어디인가?',
        unknown,
        DETECTIONS,
    ) == unknown


def test_stop_at_visible_zone_is_treated_as_destination_command():
    result = normalize_navigation_result(
        'B-1 앞에서 정지해줘',
        {'actions': [{'type': 'unknown', 'reason': '짧은 명령'}]},
        DETECTIONS,
    )

    assert result['actions'][0]['type'] == 'visual_navigation'
    assert result['actions'][0]['detection_index'] == 0


def test_visible_zone_transfer_overrides_legacy_inventory_action():
    detections = [
        {
            'detection_index': 0,
            'label': 'B-1',
            'bbox_xyxy': [440, 240, 540, 340],
            'center_xy': [490, 290],
            'heading_deg': -90.0,
        },
        {
            'detection_index': 1,
            'label': 'A-3',
            'bbox_xyxy': [100, 80, 180, 160],
            'center_xy': [140, 120],
        },
        {
            'detection_index': 2,
            'label': 'car_yellow',
            'bbox_xyxy': [240, 210, 280, 250],
            'center_xy': [260, 230],
        },
    ]
    result = normalize_navigation_result(
        'A-3 구역에 있는 컨테이너를 B-1로 옮길 거야',
        {
            'actions': [
                {
                    'type': 'cargo_bulk_by_type',
                    'cargo_type': '컨테이너',
                    'destination': '항구',
                }
            ]
        },
        detections,
    )

    assert result == {
        'actions': [
            {
                'type': 'visual_transfer',
                'source_detection_index': 1,
                'destination_detection_index': 0,
                'vehicle_id': '',
            }
        ]
    }


def test_visible_zone_transfer_repairs_unknown_response():
    result = normalize_navigation_result(
        'B-1에서 A-2로 컨테이너를 옮겨',
        {'actions': [{'type': 'unknown', 'reason': '불명확'}]},
        DETECTIONS,
    )

    assert result['actions'][0] == {
        'type': 'visual_transfer',
        'source_detection_index': 0,
        'destination_detection_index': 1,
        'vehicle_id': '',
    }


def test_same_arm_operations_are_always_sequential():
    result = {
        'execution_mode': 'parallel',
        'actions': [
            {
                'type': 'arm_scan_destinations',
                'arm_id': 'arm2',
            },
            {
                'type': 'arm_transfer_to_slot',
                'arm_id': 'arm2',
                'destination_slot': 'A-1-1',
            },
        ],
    }

    assert resolve_execution_mode('arm2로 스캔하고 A-1-1에 적재해', result) == (
        'sequential'
    )


def test_unknown_arm_command_is_not_repaired_as_vehicle_navigation():
    unknown = {
        'actions': [
            {'type': 'unknown', 'reason': '로봇팔 작업을 해석하지 못함'},
        ]
    }

    result = normalize_navigation_result(
        'arm2로 컨테이너를 A-2-1에 적재해',
        unknown,
        DETECTIONS,
    )

    assert result == unknown


def _one_vehicle_plan(vehicle_id='agv2'):
    return {
        'actions': [
            {
                'type': 'visual_navigation',
                'detection_index': 0,
                'approach_side': 'bottom',
                'vehicle_id': vehicle_id,
            }
        ]
    }


def test_naming_one_vehicle_survives_plural_wording():
    plan = _one_vehicle_plan()

    result = _finalize_navigation_result(
        '노란 차들을 항구로 보내줘', plan, plan['actions']
    )

    # "차들" reads as plural, but the colour names exactly one vehicle, so
    # fanning out would send the blue AMR somewhere nobody asked for.
    assert [action['vehicle_id'] for action in result['actions']] == ['agv2']


def test_exclusive_request_is_not_fanned_out():
    plan = _one_vehicle_plan()

    result = _finalize_navigation_result(
        'AMR2한테만 항구로 가라고 해', plan, plan['actions']
    )

    assert [action['vehicle_id'] for action in result['actions']] == ['agv2']


def test_unqualified_fleet_request_still_fans_out():
    plan = _one_vehicle_plan()

    result = _finalize_navigation_result(
        '모든 차량 주차해줘', plan, plan['actions']
    )

    assert [
        action['vehicle_id'] for action in result['actions']
    ] == ['agv1', 'agv2']


def test_naming_both_vehicles_still_fans_out():
    plan = _one_vehicle_plan()

    result = _finalize_navigation_result(
        'agv1과 agv2 차량들 모두 주차', plan, plan['actions']
    )

    assert [
        action['vehicle_id'] for action in result['actions']
    ] == ['agv1', 'agv2']


def test_vehicle_colour_mapping_matches_the_urdf():
    # pinky.urdf.xacro paints agv1 blue (0.12 0.42 0.92) and agv2 amber
    # (1.00 0.78 0.05). Every alias table has to agree with that.
    assert _mentioned_vehicle_ids('파란 차를 항구로') == {'agv1'}
    assert _mentioned_vehicle_ids('노란 차를 항구로') == {'agv2'}
    assert _mentioned_vehicle_ids('amr1 출발') == {'agv1'}
    assert _mentioned_vehicle_ids('amr2 출발') == {'agv2'}


def test_prompt_states_the_same_colour_mapping_as_the_alias_table():
    prompt = _SYSTEM_PROMPT_TEMPLATE

    # A prompt that contradicts the alias table made the VLM answer agv1 for
    # AMR2, so the two must never drift apart again.
    assert 'car_blue=agv1' in prompt
    assert 'car_yellow=agv2' in prompt
    assert 'car_yellow=agv1' not in prompt
    assert 'car_blue=agv2' not in prompt


def test_prompt_requires_complete_llm_cargo_workflow_for_both_amrs():
    prompt = _SYSTEM_PROMPT_TEMPLATE

    assert 'visual_navigation을 먼저 넣고' in prompt
    assert 'arm_transfer_to_slot' in prompt
    assert 'arm_load_to_trailer' in prompt
    assert 'AMR1(agv1)과 AMR2(agv2) 모두' in prompt


def test_prompt_exposes_arm1_dynamic_pick_place_contract():
    prompt = _SYSTEM_PROMPT_TEMPLATE

    assert '"arm1_pick_place", "arm_id": "arm1"' in prompt
    assert 'launch 설정값이 아니라 사용자 목표와 현재' in prompt
    assert '"source_id": <0..49>' in prompt
    assert '"destination_id": <0..49>' in prompt
    assert 'ARM1은 아직 중앙 서비스 계약이 없으므로' not in prompt


def test_llm_generated_arrival_then_unload_plan_is_preserved():
    result = normalize_navigation_result(
        'AMR1의 컨테이너를 A-1-2에 내려줘',
        {
            'execution_mode': 'sequential',
            'actions': [
                {
                    'type': 'visual_navigation',
                    'detection_index': 1,
                    'approach_side': 'bottom',
                    'vehicle_id': 'agv1',
                },
                {
                    'type': 'arm_transfer_to_slot',
                    'arm_id': 'arm2',
                    'destination_slot': 'A-1-2',
                    'vehicle_id': 'agv1',
                    'final_for_vehicle': True,
                },
            ],
        },
        DETECTIONS,
    )

    assert result['execution_mode'] == 'sequential'
    assert [action['type'] for action in result['actions']] == [
        'visual_navigation', 'arm_transfer_to_slot'
    ]
    assert all(
        action.get('vehicle_id') == 'agv1'
        for action in result['actions']
    )
