"""Tests for per-command ARM1 ArUco target selection."""

import threading
import types

import arm_pick_place.coordinator as coordinator_module
from arm_pick_place.coordinator import HomographyPickPlace
from arm_pick_place.dual_aruco_pose_publisher import DualArucoPosePublisher


def test_coordinator_rejects_invalid_dynamic_target_pairs():
    assert HomographyPickPlace._validate_target_ids(2, 9) == ''
    assert 'different' in HomographyPickPlace._validate_target_ids(2, 2)
    assert '0..49' in HomographyPickPlace._validate_target_ids(-1, 9)
    assert '0..49' in HomographyPickPlace._validate_target_ids(2, 50)
    assert 'disabled' in HomographyPickPlace._validate_target_ids(2, 18)


def test_detector_switches_role_frames_to_requested_ids():
    detector = object.__new__(DualArucoPosePublisher)
    detector.marker_frames = {
        2: 'arm/pick_marker',
        8: 'arm/place_marker',
    }
    detector.rejection_counts = {2: 0, 8: 0}
    detector.last_detected_ids = (2, 8)
    messages = []
    detector.get_logger = lambda: types.SimpleNamespace(
        info=messages.append
    )
    request = types.SimpleNamespace(pick_id=6, place_id=9)
    response = types.SimpleNamespace(accepted=False, message='')

    result = detector.configure_targets(request, response)

    assert result.accepted
    assert detector.marker_frames == {
        6: 'arm/pick_marker',
        9: 'arm/place_marker',
    }
    assert detector.rejection_counts == {6: 0, 9: 0}
    assert detector.last_detected_ids is None


def test_detector_rejects_target_change_during_detection():
    detector = object.__new__(DualArucoPosePublisher)
    detector.detection_enabled = True
    request = types.SimpleNamespace(pick_id=6, place_id=9)
    response = types.SimpleNamespace(accepted=False, message='')

    result = detector.configure_targets(request, response)

    assert not result.accepted
    assert 'active detection' in result.message


def test_explicit_ship_scan_runs_even_when_cache_is_complete(monkeypatch):
    """A central restart must trigger physical scanning, not a cache no-op."""
    coordinator = object.__new__(HomographyPickPlace)
    coordinator.command_lock = threading.Lock()
    coordinator.saved_marker_poses = {
        marker_id: object() for marker_id in range(19, 24)
    }
    coordinator.motion_thread = None
    coordinator.stop_event = threading.Event()
    coordinator.external_stop_requested = True
    started = []

    class FakeThread:
        def __init__(self, target, daemon):
            self.target = target
            self.daemon = daemon

        def start(self):
            started.append(self.target)

        def is_alive(self):
            return False

    monkeypatch.setattr(coordinator_module.threading, 'Thread', FakeThread)
    response = types.SimpleNamespace(success=False, message='')

    result = coordinator.scan_ship_destinations(None, response)

    assert result.success
    assert 'fresh two-view' in result.message
    assert started == [coordinator._run_ship_destination_scan]
    assert not coordinator.external_stop_requested
