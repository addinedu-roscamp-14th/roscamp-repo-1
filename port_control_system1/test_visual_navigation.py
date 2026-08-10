"""Tests for deterministic pixel calculation from YOLO detections."""

import pytest

from visual_navigation import (
    VisualNavigationError,
    compact_detections,
    is_reciprocal_zone_exchange,
    resolve_detection_approach,
    select_nearest_visible_vehicle,
    validate_pixel_navigation,
    zone_mode_for_label,
)


SUMMARY = {
    'detections': [
        {
            'label': 'car_blue',
            'confidence': 0.91,
            'bbox_xyxy': [200, 150, 300, 250],
            'heading_deg': -12.5,
            'mask_xy': [[200, 150], [300, 150], [300, 250]],
        },
        {
            'label': 'trailer',
            'confidence': 0.87,
            'bbox_xyxy': [400, 100, 500, 220],
        },
    ]
}


def test_compact_detections_removes_large_masks():
    compact = compact_detections(SUMMARY)
    assert compact[0]['detection_index'] == 0
    assert compact[0]['center_xy'] == [250.0, 200.0]
    assert 'mask_xy' not in compact[0]


def test_resolve_detection_approach_uses_bbox_not_llm_coordinates():
    target, heading, selected = resolve_detection_approach(
        {
            'type': 'visual_navigation',
            'detection_index': 0,
            'approach_side': 'left',
        },
        SUMMARY,
        640,
        480,
        clearance_px=50,
    )
    assert target == {'x': 150.0, 'y': 200.0}
    assert heading == {'x': 250.0, 'y': 200.0}
    assert selected['label'] == 'car_blue'


def test_direct_target_inside_detection_is_rejected():
    with pytest.raises(VisualNavigationError, match='객체 내부'):
        validate_pixel_navigation(
            {'x': 250, 'y': 200},
            {'x': 350, 'y': 200},
            640,
            480,
            SUMMARY,
        )


def test_b1_parking_uses_zone_center_and_segmentation_heading():
    summary = {
        'detections': [
            {
                'label': 'B-1',
                'confidence': 0.95,
                'bbox_xyxy': [440, 240, 540, 340],
                'heading_deg': -90.0,
            },
        ],
    }
    target, heading, selected = resolve_detection_approach(
        {
            'type': 'visual_navigation',
            'detection_index': 0,
            'approach_side': 'bottom',
        },
        summary,
        640,
        480,
    )
    assert selected['label'] == 'B-1'
    assert target == {'x': 490.0, 'y': 290.0}
    assert heading['x'] == pytest.approx(490.0)
    assert heading['y'] == pytest.approx(240.0)


def test_a_zone_heading_always_points_image_up():
    summary = {
        'detections': [
            {
                'label': 'A-2',
                'confidence': 0.9,
                'bbox_xyxy': [200, 200, 260, 260],
            },
        ],
    }
    target, heading, selected = resolve_detection_approach(
        {'type': 'visual_navigation', 'detection_index': 0, 'approach_side': 'top'},
        summary,
        640,
        480,
    )
    assert selected['label'] == 'A-2'
    assert target == {'x': 230.0, 'y': 230.0}
    assert heading == {'x': 230.0, 'y': 180.0}


def test_a_zone_labels_share_the_same_mode():
    assert zone_mode_for_label('A-1') == 'parking_a'
    assert zone_mode_for_label('A-2') == 'parking_a'
    assert zone_mode_for_label('A-3') == 'parking_a'
    assert zone_mode_for_label('B-1') == 'parking_b1'
    assert zone_mode_for_label('car_yellow') == 'direct'


def test_b1_parking_is_rejected_when_vehicle_occupies_center():
    summary = {
        'detections': [
            {
                'label': 'B-1',
                'bbox_xyxy': [440, 240, 540, 340],
                'heading_deg': 0.0,
            },
            {
                'label': 'car_yellow',
                'bbox_xyxy': [470, 270, 510, 310],
            },
        ],
    }
    with pytest.raises(VisualNavigationError, match='객체 내부'):
        resolve_detection_approach(
            {
                'type': 'visual_navigation',
                'detection_index': 0,
                'approach_side': 'left',
            },
            summary,
            640,
            480,
        )


def test_transfer_selects_nearest_live_vehicle_to_source_zone():
    summary = {
        'detections': [
            {
                'label': 'A-3',
                'bbox_xyxy': [80, 60, 180, 160],
            },
            {
                'label': 'car_yellow',
                'bbox_xyxy': [190, 120, 230, 160],
            },
            {
                'label': 'car_blue',
                'bbox_xyxy': [430, 300, 470, 340],
            },
        ]
    }
    source = compact_detections(summary)[0]

    # car_yellow sits closest to A-3, and yellow is AMR 2 (agv2).
    assert select_nearest_visible_vehicle(source, summary) == 'agv2'
    assert (
        select_nearest_visible_vehicle(source, summary, {'agv1'})
        == 'agv1'
    )


def test_reciprocal_occupied_zone_exchange_runs_independently():
    summary = {
        'detections': [
            {'label': 'B-1', 'bbox_xyxy': [440, 240, 540, 340]},
            {'label': 'A-3', 'bbox_xyxy': [100, 80, 180, 160]},
        ]
    }
    actions = [
        {
            'type': 'visual_navigation',
            'detection_index': 0,
            'approach_side': 'bottom',
            'vehicle_id': 'agv1',
        },
        {
            'type': 'visual_navigation',
            'detection_index': 1,
            'approach_side': 'bottom',
            'vehicle_id': 'agv2',
        },
    ]

    assert is_reciprocal_zone_exchange(
        actions,
        summary,
        'B-1:agv2;A:agv1',
    )
    assert not is_reciprocal_zone_exchange(
        actions,
        summary,
        'B-1:FREE;A:agv1',
    )
