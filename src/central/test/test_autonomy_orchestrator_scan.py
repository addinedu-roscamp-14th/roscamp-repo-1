"""Tests for initial ARM destination-scan state handling."""

import json
import threading
from types import SimpleNamespace

from central.autonomy_orchestrator import AutonomyOrchestrator
from porter_interfaces.msg import PortEvent


class CompletedFuture:
    """Return a completed action result with the requested outcome."""

    def __init__(self, success, message=''):
        self._value = SimpleNamespace(
            result=SimpleNamespace(success=success, message=message)
        )

    def result(self):
        """Return the fake action result."""
        return self._value


def make_orchestrator():
    """Construct the scan-state subset without starting a ROS node."""
    orchestrator = object.__new__(AutonomyOrchestrator)
    orchestrator.lock = threading.Lock()
    orchestrator.scan_requested = True
    orchestrator.scan_retry_pending = False
    orchestrator.arm2_cache_ready = False
    orchestrator.arm2_scan_retry_sec = 10.0
    orchestrator.arm2_scan_retry_not_before = 0.0
    orchestrator.statuses = []
    orchestrator._publish_status = (
        lambda state, mission_id='', **extra:
        orchestrator.statuses.append((state, mission_id, extra))
    )
    return orchestrator


def test_successful_arm2_scan_marks_initial_cache_ready():
    """A successful initial scan must stop retries and expose readiness."""
    orchestrator = make_orchestrator()

    orchestrator._on_scan_result('arm2-init', CompletedFuture(True))

    assert orchestrator.arm2_cache_ready
    assert not orchestrator.scan_requested
    assert not orchestrator.scan_retry_pending
    assert orchestrator.statuses[-1][0] == 'WAITING_FOR_CARGO_POLICY'


def test_failed_arm2_scan_is_retriable():
    """A failed destination scan must release the in-flight retry gate."""
    orchestrator = make_orchestrator()

    orchestrator._on_scan_result(
        'arm2-init', CompletedFuture(False, 'marker missing')
    )

    assert not orchestrator.arm2_cache_ready
    assert not orchestrator.scan_requested
    assert orchestrator.scan_retry_pending
    assert orchestrator.arm2_scan_retry_not_before > 0.0
    assert orchestrator.statuses[-1][0] == 'ARM2_SCAN_RETRY'


def test_vessel_departure_does_not_duplicate_an_active_arm2_scan():
    """Departure must retain the in-flight gate until its result arrives."""
    orchestrator = make_orchestrator()
    orchestrator.active_mission_id = 'port-1'
    orchestrator.arrival_event_id = 'arrival-1'
    orchestrator.inbound_scan_requested = True
    orchestrator.inbound_scan_pending = True
    orchestrator.get_parameter = (
        lambda _name: SimpleNamespace(value=True)
    )
    message = SimpleNamespace(event_type=PortEvent.VESSEL_DEPARTED)

    orchestrator._on_port_event(message)

    assert orchestrator.scan_requested
    assert not orchestrator.scan_retry_pending
    assert not orchestrator.inbound_scan_requested
    assert not orchestrator.inbound_scan_pending


def test_new_arrival_during_scan_keeps_follow_up_scan_pending():
    """A cargo addition must not be lost while the prior scan is finishing."""
    orchestrator = make_orchestrator()
    orchestrator.inbound_scan_requested = True
    orchestrator.inbound_scan_pending = True
    orchestrator.arrival_event_id = 'arrival-new'
    orchestrator.arm1_cache_ready = True

    orchestrator._on_inbound_scan_result(
        'port-1', 'arrival-old', CompletedFuture(True)
    )

    assert not orchestrator.inbound_scan_requested
    assert orchestrator.inbound_scan_pending
    assert orchestrator.statuses[-1][0] == 'INBOUND_RESCAN_PENDING'


def test_completed_latest_arrival_scan_clears_pending_state():
    """The latest cargo scan completion must close the pending request."""
    orchestrator = make_orchestrator()
    orchestrator.inbound_scan_requested = True
    orchestrator.inbound_scan_pending = True
    orchestrator.arrival_event_id = 'arrival-new'
    orchestrator.arm1_cache_ready = True

    orchestrator._on_inbound_scan_result(
        'port-1', 'arrival-new', CompletedFuture(True)
    )

    assert not orchestrator.inbound_scan_pending
    assert orchestrator.statuses[-1][0] == 'INBOUND_SCAN_COMPLETE'


def test_cargo_added_event_rescans_without_replacing_port_mission():
    """Added cargo must rescan within the current vessel mission."""
    orchestrator = make_orchestrator()
    orchestrator.active_mission_id = 'port-existing'
    orchestrator.arrival_event_id = 'arrival-old'
    orchestrator.inbound_scan_requested = False
    orchestrator.inbound_scan_pending = False
    orchestrator.arm1_cache_ready = True
    orchestrator.arm2_cache_ready = True
    requested = []
    orchestrator.get_parameter = (
        lambda _name: SimpleNamespace(value=True)
    )
    orchestrator._request_arm1_inbound_scan = requested.append
    message = SimpleNamespace(
        event_type=PortEvent.VESSEL_ARRIVED,
        event_id='arrival-added',
        details_json=json.dumps({'change_type': 'CARGO_ADDED'}),
    )

    orchestrator._on_port_event(message)

    assert orchestrator.active_mission_id == 'port-existing'
    assert orchestrator.arrival_event_id == 'arrival-added'
    assert orchestrator.inbound_scan_pending
    assert requested == ['port-existing']
    assert orchestrator.statuses[-1][0] == 'CARGO_ADDED'
