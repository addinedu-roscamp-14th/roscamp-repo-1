"""Tests for image forwarding to the Ollama-compatible VLM."""

import sys
import types

from llm_command_parser import (
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
    assert '"detection_index":0' in user_message['content']
    assert '"label":"car_blue"' in user_message['content']
    assert (
        '"execution_mode": "<parallel|sequential>"'
        in system_message['content']
    )
    assert '기본적으로 execution_mode를 "parallel"' in system_message['content']
    assert result['actions'][0]['type'] == 'pixel_navigation'


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
                'vehicle_id': 'agv1',
            }
        ]
    }


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
    assert action['vehicle_id'] == 'agv2'


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
