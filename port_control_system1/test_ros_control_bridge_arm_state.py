"""Tests for structured robot-arm telemetry in the dashboard bridge."""

from types import SimpleNamespace

from ros_control_bridge import RosControlBridge


def arm_message(arm_id='arm2', state=3, **overrides):
    values = {
        'arm_id': arm_id,
        'state': state,
        'state_text': 'TRANSFER_RUNNING',
        'ready': False,
        'current_command_id': 'arm-command-7',
        'current_mission_id': 'mission-2',
        'current_operation': 'transfer_to_slot',
        'operation_id': 'operation-9',
        'phase': 'PLACE',
        'progress': 80.0,
        'last_error': '',
        'telemetry_age_sec': 0.4,
    }
    values.update(overrides)
    return SimpleNamespace(**values)


def test_arm_state_is_exposed_in_dashboard_snapshot():
    bridge = RosControlBridge()

    bridge._on_arm_state(arm_message())

    snapshot = bridge.snapshot()
    assert len(snapshot.arm_states) == 1
    arm = snapshot.arm_states[0]
    assert arm.arm_id == 'arm2'
    assert arm.state == 3
    assert arm.current_operation == 'transfer_to_slot'
    assert arm.phase == 'PLACE'
    assert arm.progress == 80.0
    assert arm.telemetry_age_sec == 0.4


def test_arm_states_are_kept_separately_and_sorted():
    bridge = RosControlBridge()

    bridge._on_arm_state(arm_message('arm2', state=2, state_text='READY'))
    bridge._on_arm_state(arm_message(
        'arm1',
        state=0,
        state_text='ARM1 service contract is not configured',
        last_error='not configured',
    ))

    assert [arm.arm_id for arm in bridge.snapshot().arm_states] == [
        'arm1', 'arm2'
    ]
    assert bridge.snapshot().arm_states[0].last_error == 'not configured'


def test_unknown_arm_id_is_ignored():
    bridge = RosControlBridge()

    bridge._on_arm_state(arm_message('arm99'))

    assert bridge.snapshot().arm_states == ()
