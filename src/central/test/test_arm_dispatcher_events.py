"""Tests for ARM2 operation-level terminal event handling."""

from central.arm_dispatcher import is_terminal_event


def test_intermediate_completed_state_is_not_terminal():
    event = {
        'phase': 'SOURCE_LOCKED',
        'state': 'COMPLETED',
        'progress': 20,
    }

    assert not is_terminal_event(event)


def test_final_completed_and_failed_phases_are_terminal():
    assert is_terminal_event({'phase': 'COMPLETED', 'state': 'COMPLETED'})
    assert is_terminal_event({'phase': 'FAILED', 'state': 'FAILED'})
    assert is_terminal_event({'phase': 'STOPPED', 'state': 'COMPLETED'})
