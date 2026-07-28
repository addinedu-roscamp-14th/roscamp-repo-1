"""Validation helpers for central-control HTTP commands."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Any, Mapping


class CommandValidationError(ValueError):
    """Raised when an external control command is malformed or unsafe."""


@dataclass(frozen=True)
class Pixel:
    """Validated image pixel."""

    x: float
    y: float


@dataclass(frozen=True)
class PixelGoal:
    """Validated target and heading pixels."""

    command_id: str | None
    requested_vehicle_id: str
    zone_id: str
    mode: str
    target: Pixel
    heading: Pixel


def _finite_number(value: Any, field_name: str) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise CommandValidationError(f'{field_name} must be a number')
    number = float(value)
    if not math.isfinite(number):
        raise CommandValidationError(f'{field_name} must be finite')
    return number


def _pixel(
    value: Any,
    field_name: str,
    image_width: int,
    image_height: int,
) -> Pixel:
    if not isinstance(value, Mapping):
        raise CommandValidationError(
            f'{field_name} must be an object containing x and y'
        )
    x = _finite_number(value.get('x'), f'{field_name}.x')
    y = _finite_number(value.get('y'), f'{field_name}.y')
    if not 0.0 <= x < image_width:
        raise CommandValidationError(
            f'{field_name}.x must be within [0, {image_width})'
        )
    if not 0.0 <= y < image_height:
        raise CommandValidationError(
            f'{field_name}.y must be within [0, {image_height})'
        )
    return Pixel(x=x, y=y)


def validate_pixel_goal(
    payload: Any,
    image_width: int,
    image_height: int,
    minimum_heading_distance_px: float,
) -> PixelGoal:
    """Validate one AI-generated camera-pixel navigation command."""
    if not isinstance(payload, Mapping):
        raise CommandValidationError('request body must be a JSON object')
    if image_width <= 0 or image_height <= 0:
        raise ValueError('image dimensions must be positive')
    if minimum_heading_distance_px <= 0.0:
        raise ValueError('minimum heading distance must be positive')

    command_id = payload.get('command_id')
    if command_id is not None:
        if not isinstance(command_id, str) or not command_id.strip():
            raise CommandValidationError(
                'command_id must be a non-empty string'
            )
        command_id = command_id.strip()
        if len(command_id) > 128:
            raise CommandValidationError(
                'command_id must not exceed 128 characters'
            )

    mode = payload.get('mode', 'direct')
    if mode not in ('direct', 'parking_b1'):
        raise CommandValidationError(
            'mode must be direct or parking_b1'
        )

    requested_vehicle_id = payload.get('vehicle_id', '')
    if requested_vehicle_id is None:
        requested_vehicle_id = ''
    if not isinstance(requested_vehicle_id, str):
        raise CommandValidationError('vehicle_id must be a string')
    requested_vehicle_id = requested_vehicle_id.strip().strip('/')
    if requested_vehicle_id not in ('', 'agv1', 'agv2'):
        raise CommandValidationError(
            'vehicle_id must be empty, agv1, or agv2'
        )

    zone_id = payload.get('zone_id', '')
    if zone_id is None:
        zone_id = ''
    if not isinstance(zone_id, str):
        raise CommandValidationError('zone_id must be a string')
    zone_id = zone_id.strip().upper()
    if mode == 'parking_b1' and not zone_id:
        zone_id = 'B-1'
    if zone_id not in ('', 'B-1'):
        raise CommandValidationError('zone_id must be empty or B-1')

    target = _pixel(
        payload.get('target'),
        'target',
        image_width,
        image_height,
    )
    heading = _pixel(
        payload.get('heading'),
        'heading',
        image_width,
        image_height,
    )
    distance = math.hypot(
        heading.x - target.x,
        heading.y - target.y,
    )
    if distance < minimum_heading_distance_px:
        raise CommandValidationError(
            'heading must be at least '
            f'{minimum_heading_distance_px:.1f}px from target'
        )

    return PixelGoal(
        command_id=command_id,
        requested_vehicle_id=requested_vehicle_id,
        zone_id=zone_id,
        mode=mode,
        target=target,
        heading=heading,
    )
