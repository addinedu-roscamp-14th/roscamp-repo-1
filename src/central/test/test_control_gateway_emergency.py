"""Fleet emergency behavior shared by AMRs and both robot arms."""

import threading
import types

import pytest

from central.control_gateway import CentralControlGateway
from central.control_protocol import CommandValidationError


class _DoneFuture:
    def done(self):
        return True

    def result(self):
        return types.SimpleNamespace(success=True, message='latched')


class _ServiceClient:
    def wait_for_service(self, timeout_sec=0.0):
        return True

    def call_async(self, _request):
        return _DoneFuture()


def _gateway():
    gateway = object.__new__(CentralControlGateway)
    gateway._lock = threading.Lock()
    gateway._dispatch_lock = threading.Lock()
    gateway._emergency_targets = set()
    gateway._telemetry = {'vehicles': {}}
    gateway._fleet_emergency_client = _ServiceClient()
    gateway._vehicle_emergency_clients = {
        'agv1': _ServiceClient(),
        'agv2': _ServiceClient(),
    }
    gateway._arm_resume_clients = {
        'arm1': _ServiceClient(),
        'arm2': _ServiceClient(),
    }
    return gateway


def test_fleet_emergency_stops_both_arms_and_latches_arm_gate():
    gateway = _gateway()
    stopped = []
    gateway.stop_arm = lambda arm_id: (
        stopped.append(arm_id)
        or {'accepted': True, 'message': 'stopped'}
    )

    result = gateway.set_emergency('fleet', True)

    assert stopped == ['arm1', 'arm2']
    assert result['accepted'] is True
    assert result['arm_commands_blocked'] is True
    assert result['arms']['arm1']['stopped'] is True
    assert result['arms']['arm2']['stopped'] is True


def test_arm_dispatch_is_rejected_while_emergency_is_latched():
    gateway = _gateway()
    gateway._emergency_targets.add('fleet')

    with pytest.raises(CommandValidationError, match='emergency stop'):
        gateway.dispatch_arm_command({
            'arm_id': 'arm1',
            'operation': 'pick_place',
        })


def test_emergency_release_resumes_both_dispatchers_without_arm_motion():
    gateway = _gateway()
    gateway._emergency_targets.add('fleet')
    stopped = []
    resumed = []
    gateway.stop_arm = lambda arm_id: stopped.append(arm_id)
    gateway.resume_arm = lambda arm_id: (
        resumed.append(arm_id)
        or {'accepted': True, 'message': 'ready'}
    )

    result = gateway.set_emergency('fleet', False)

    assert stopped == []
    assert resumed == ['arm1', 'arm2']
    assert result['arm_commands_blocked'] is False
    assert result['arms']['arm1']['resumed'] is True
    assert result['arms']['arm2']['resumed'] is True
