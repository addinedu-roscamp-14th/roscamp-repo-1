"""Tests for operator manual inventory position editing."""

from types import SimpleNamespace

import pytest

from cargo_dispatch_tool import (
    MANUAL_INVENTORY_LOCATIONS,
    validate_manual_inventory_position,
)


def cargo(container_id, location, floor, name='cargo'):
    return SimpleNamespace(
        container_id=str(container_id),
        location=location,
        floor=floor,
        name=name,
    )


def test_manual_locations_match_autonomous_canonical_contract():
    assert MANUAL_INVENTORY_LOCATIONS == (
        'A-1-1', 'A-1-2', 'A-2-1', 'A-2-2', 'A-3-1', 'A-3-2',
        'AMR1', 'AMR2',
        '선박-1', '선박-2', '선박-3', '선박-4', '선박-5', '선박-6',
        '출항완료',
    )


def test_floor_one_uses_registered_location_marker():
    assert validate_manual_inventory_position('6', 'A-1-1', 1, []) == '11'
    assert validate_manual_inventory_position('6', '선박-6', 1, []) == '23'
    assert validate_manual_inventory_position('6', 'AMR2', 1, []) == '9'


def test_upper_floor_uses_container_below_as_base():
    records = [
        cargo('1', 'A-1-1', 1),
        cargo('4', 'A-1-1', 2),
    ]

    assert validate_manual_inventory_position(
        '6', 'A-1-1', 3, records
    ) == '4'


def test_rejects_duplicate_floor_and_unsupported_stack():
    records = [cargo('1', 'A-1-1', 1)]

    with pytest.raises(ValueError, match='이미 컨테이너'):
        validate_manual_inventory_position('6', 'A-1-1', 1, records)
    with pytest.raises(ValueError, match='2층 컨테이너'):
        validate_manual_inventory_position('6', 'A-1-1', 3, records)


def test_trailer_and_departed_locations_are_floor_one_only():
    with pytest.raises(ValueError, match='1층만'):
        validate_manual_inventory_position('6', 'AMR1', 2, [])
    with pytest.raises(ValueError, match='1층만'):
        validate_manual_inventory_position('6', '출항완료', 2, [])
