"""Tests for declared stack planning and marker orientation."""

import math

from arm.container_pick_coordinator import quaternion_from_rpy_degrees
from arm_relocation2.stack_pick_place_coordinator import (
    StackMove,
    apply_completed_move,
    calculate_stack_plan,
    marker_normal_angle_deg,
    parse_stack,
)
import pytest


def test_requested_three_level_example():
    """Move blockers top-down, stacking each on the previous blocker."""
    plan = calculate_stack_plan([1, 2, 3, 4], [5], 2, 6)

    assert plan == [
        StackMove(4, 5, 'empty'),
        StackMove(3, 4, 'empty'),
        StackMove(2, 6, 'final_place'),
    ]


def test_empty_stack_may_already_contain_a_container():
    """Use the declared current top of a non-empty Empty stack."""
    plan = calculate_stack_plan([1, 2, 3], [5, 9], 2, 6)

    assert plan == [
        StackMove(3, 9, 'empty'),
        StackMove(2, 6, 'final_place'),
    ]


def test_completed_plan_updates_source_and_empty_states():
    """Track only moves that have physically completed."""
    source = [1, 2, 3, 4]
    empty = [5]
    plan = calculate_stack_plan(source, empty, 2, 6)

    for move in plan:
        apply_completed_move(source, empty, move)

    assert source == [1]
    assert empty == [5, 4, 3]


def test_top_target_moves_directly_to_final_place():
    """Do not create unnecessary Empty moves for an exposed target."""
    assert calculate_stack_plan([1, 2, 3], [5], 3, 6) == [
        StackMove(3, 6, 'final_place'),
    ]


def test_stack_string_is_json():
    """Accept launch-friendly JSON strings."""
    assert parse_stack('[1, 2, 3]', 'source_stack') == [1, 2, 3]


@pytest.mark.parametrize(
    'source, empty, target, message',
    [
        ([1, 2, 2], [5], 2, 'duplicate'),
        ([1, 2], [2, 5], 2, 'disjoint'),
        ([1, 2], [5], 1, 'base marker'),
        ([1, 2], [5], 8, 'not in source_stack'),
        ([1, 2, 6], [5], 2, 'outside both declared stacks'),
    ],
)
def test_invalid_declarations_are_rejected(
    source, empty, target, message
):
    """Reject plans that cannot describe distinct physical markers."""
    with pytest.raises(ValueError, match=message):
        calculate_stack_plan(source, empty, target, 6)


def test_top_marker_normal_is_parallel_to_base_z():
    """Recognize a horizontal marker independently of its yaw."""
    quaternion = quaternion_from_rpy_degrees(0.0, 0.0, 47.0)

    assert math.isclose(
        marker_normal_angle_deg(quaternion), 0.0, abs_tol=1e-7
    )
