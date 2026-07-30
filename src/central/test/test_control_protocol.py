"""Tests for external central-control command validation."""

from central.control_protocol import (
    CommandValidationError,
    validate_pixel_goal,
)

import pytest


def test_valid_pixel_goal_is_normalized():
    """Valid integer and float coordinates are normalized."""
    goal = validate_pixel_goal(
        {
            'command_id': 'job-42',
            'target': {'x': 100, 'y': 200},
            'heading': {'x': 130.5, 'y': 200},
        },
        image_width=640,
        image_height=480,
        minimum_heading_distance_px=10.0,
    )

    assert goal.command_id == 'job-42'
    assert goal.mode == 'direct'
    assert goal.requested_vehicle_id == ''
    assert goal.zone_id == ''
    assert goal.target.x == 100.0
    assert goal.heading.x == 130.5


def test_b1_parking_mode_is_accepted():
    goal = validate_pixel_goal(
        {
            'mode': 'parking_b1',
            'target': {'x': 100, 'y': 200},
            'heading': {'x': 130, 'y': 200},
        },
        image_width=640,
        image_height=480,
        minimum_heading_distance_px=10.0,
    )
    assert goal.mode == 'parking_b1'
    assert goal.zone_id == 'B-1'


def test_vehicle_selection_is_normalized():
    goal = validate_pixel_goal(
        {
            'vehicle_id': '/agv2/',
            'target': {'x': 100, 'y': 200},
            'heading': {'x': 130, 'y': 200},
        },
        image_width=640,
        image_height=480,
        minimum_heading_distance_px=10.0,
    )
    assert goal.requested_vehicle_id == 'agv2'


def test_unknown_vehicle_is_rejected():
    with pytest.raises(CommandValidationError, match='vehicle_id'):
        validate_pixel_goal(
            {
                'vehicle_id': 'agv3',
                'target': {'x': 100, 'y': 200},
                'heading': {'x': 130, 'y': 200},
            },
            image_width=640,
            image_height=480,
            minimum_heading_distance_px=10.0,
        )


def test_unknown_goal_mode_is_rejected():
    with pytest.raises(CommandValidationError, match='mode must be'):
        validate_pixel_goal(
            {
                'mode': 'parking_anywhere',
                'target': {'x': 100, 'y': 200},
                'heading': {'x': 130, 'y': 200},
            },
            image_width=640,
            image_height=480,
            minimum_heading_distance_px=10.0,
        )


@pytest.mark.parametrize(
    'point',
    [
        {'x': -1, 'y': 20},
        {'x': 640, 'y': 20},
        {'x': 20, 'y': 480},
        {'x': float('nan'), 'y': 20},
    ],
)
def test_target_must_be_a_finite_in_frame_pixel(point):
    """Out-of-frame and non-finite target coordinates are rejected."""
    with pytest.raises(CommandValidationError):
        validate_pixel_goal(
            {
                'target': point,
                'heading': {'x': 100, 'y': 100},
            },
            image_width=640,
            image_height=480,
            minimum_heading_distance_px=10.0,
        )


def test_heading_must_not_overlap_target():
    """Heading requires enough pixel separation to define a direction."""
    with pytest.raises(
        CommandValidationError,
        match='heading must be at least',
    ):
        validate_pixel_goal(
            {
                'target': {'x': 100, 'y': 100},
                'heading': {'x': 105, 'y': 100},
            },
            image_width=640,
            image_height=480,
            minimum_heading_distance_px=10.0,
        )


def test_boolean_is_not_accepted_as_a_coordinate():
    """Python booleans must not pass numeric coordinate validation."""
    with pytest.raises(CommandValidationError, match='must be a number'):
        validate_pixel_goal(
            {
                'target': {'x': True, 'y': 100},
                'heading': {'x': 130, 'y': 100},
            },
            image_width=640,
            image_height=480,
            minimum_heading_distance_px=10.0,
        )
