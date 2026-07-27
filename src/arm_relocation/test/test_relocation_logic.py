"""Unit tests for marker face and relocation decisions."""

import math

from arm.container_pick_coordinator import quaternion_from_rpy_degrees
from arm_relocation.container_pick_place_relocation import (
    classify_marker_face,
    ContainerPickPlaceRelocation,
    marker_normal_angle_from_vertical_deg,
    relocation_destination,
    select_topmost_stacked_marker,
)


def test_top_marker_normal_is_parallel_to_base_vertical():
    quaternion = quaternion_from_rpy_degrees(0.0, 0.0, 25.0)

    assert math.isclose(
        marker_normal_angle_from_vertical_deg(quaternion), 0.0, abs_tol=1e-7
    )
    assert classify_marker_face(quaternion, 30.0, 60.0)[0] == 'top'


def test_side_marker_normal_is_horizontal():
    quaternion = quaternion_from_rpy_degrees(90.0, 0.0, 0.0)

    assert math.isclose(
        marker_normal_angle_from_vertical_deg(quaternion),
        90.0,
        abs_tol=1e-7,
    )
    assert classify_marker_face(quaternion, 30.0, 60.0)[0] == 'side'


def test_topmost_selection_uses_face_xy_stack_and_height():
    pick = {
        'id': 1,
        'face': 'side',
        'translation': [0.10, 0.10, 0.03],
    }
    selected = select_topmost_stacked_marker(
        [
            {
                'id': 4,
                'face': 'top',
                'area_px': 900.0,
                'translation': [0.11, 0.11, 0.08],
            },
            {
                'id': 5,
                'face': 'top',
                'area_px': 300.0,
                'translation': [0.11, 0.10, 0.14],
            },
            {
                'id': 6,
                'face': 'top',
                'area_px': 1200.0,
                'translation': [0.24, 0.10, 0.20],
            },
            {
                'id': 7,
                'face': 'side',
                'area_px': 1500.0,
                'translation': [0.10, 0.10, 0.22],
            },
        ],
        pick,
        xy_tolerance_m=0.08,
    )

    assert selected['id'] == 5
    assert math.isclose(selected['height_above_pick_m'], 0.11)


def test_topmost_selection_can_exclude_destination_markers():
    pick = {
        'id': 1,
        'face': 'side',
        'translation': [0.0, 0.0, 0.0],
    }
    selected = select_topmost_stacked_marker(
        [
            {
                'id': 2,
                'face': 'top',
                'area_px': 100.0,
                'translation': [0.01, 0.0, 0.20],
            },
            {
                'id': 8,
                'face': 'top',
                'area_px': 100.0,
                'translation': [0.01, 0.0, 0.10],
            },
        ],
        pick,
        xy_tolerance_m=0.08,
        excluded_ids=(2,),
    )

    assert selected['id'] == 8


def test_merge_retains_top_and_side_observations_with_same_id():
    side = {
        'id': 4,
        'face': 'side',
        'area_px': 100.0,
        'translation': [0.10, 0.10, 0.04],
    }
    top = {
        'id': 4,
        'face': 'top',
        'area_px': 100.0,
        'translation': [0.10, 0.10, 0.07],
    }

    merged = ContainerPickPlaceRelocation.merge_observations(
        [side], [top]
    )

    assert [(item['id'], item['face']) for item in merged] == [
        (4, 'side'),
        (4, 'top'),
    ]


def test_commanded_marker_goes_to_place_and_blocker_to_empty():
    assert relocation_destination(7, 7) == 'place'
    assert relocation_destination(8, 7) == 'empty'
