"""Tests for the fixed autonomous policy and deterministic move compiler."""

import pytest

from autonomous_inventory import (
    AutonomousCycle,
    AutonomousPolicyError,
    CycleStore,
    choose_policy,
    compile_move,
    validate_first_move,
)


def snapshot(*cargos):
    return {'cargos': list(cargos)}


def cargo(container_id, location, floor=1, base=''):
    return {
        'name': f'C{container_id}', 'container_id': str(container_id),
        'location': location, 'floor': floor, 'base_aruco_id': base,
    }


def test_inbound_is_unloaded_before_outbound():
    cycle = AutonomousCycle()
    state = snapshot(
        cargo(1, '선박-1'),
        cargo(2, 'A-1-1', 3),
    )
    phase, objective = choose_policy(cycle, state, True)
    assert phase == 'UNLOADING_INBOUND'
    assert '[1]' in objective


def test_outbound_is_not_reclassified_as_inbound():
    cycle = AutonomousCycle(inbound_ids=['1'], outbound_ids=['6'])
    phase, _objective = choose_policy(
        cycle, snapshot(cargo(6, '선박-1')), True
    )
    assert phase == 'WAITING_FOR_CLEAR'


def test_third_floor_outbound_starts_after_inbound_cycle():
    cycle = AutonomousCycle(inbound_ids=['1'])
    phase, objective = choose_policy(
        cycle, snapshot(cargo(6, 'A-2-1', 3)), False
    )
    assert phase == 'LOADING_OUTBOUND'
    assert '[6]' in objective


def test_validate_rejects_blocked_lower_container():
    state = snapshot(cargo(1, 'A-1-1', 1), cargo(2, 'A-1-1', 2))
    with pytest.raises(AutonomousPolicyError, match='blocked'):
        validate_first_move({
            'container_id': '1', 'source_location': 'A-1-1',
            'destination_location': '선박-1', 'destination_floor': 1,
        }, state)


def test_ship_to_warehouse_compiles_for_both_trailers():
    move = {
        'container_id': '6', 'source_location': '선박-1',
        'destination_location': 'A-1-2', 'destination_floor': 1,
    }
    agv1 = compile_move(move, 'agv1')
    agv2 = compile_move(move, 'agv2')
    assert [step['type'] for step in agv1] == [
        'zone_navigation', 'arm1_pick_place', 'zone_navigation',
        'arm_transfer_to_slot', 'park_command',
    ]
    assert agv1[1]['destination_id'] == 10
    assert agv2[1]['destination_id'] == 9


def test_warehouse_to_ship_uses_cached_ship_marker():
    steps = compile_move({
        'container_id': '6', 'source_location': 'A-2-1',
        'destination_location': '선박-6', 'destination_floor': 1,
    }, 'agv1')
    assert steps[1]['type'] == 'arm_load_to_trailer'
    assert steps[3]['source_id'] == 10
    assert steps[3]['destination_id'] == 23


def test_cycle_store_survives_process_restart(tmp_path):
    store = CycleStore(str(tmp_path / 'cycle.json'))
    cycle = AutonomousCycle(outbound_ids=['6'], phase='WAITING_FOR_CLEAR')
    store.save(cycle)
    restored = store.load()
    assert restored.cycle_id == cycle.cycle_id
    assert restored.outbound_ids == ['6']
    assert restored.phase == 'WAITING_FOR_CLEAR'
