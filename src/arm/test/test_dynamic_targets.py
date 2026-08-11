"""Tests for per-command ARM1 ArUco target selection."""

import types

from arm_pick_place.coordinator import HomographyPickPlace
from arm_pick_place.dual_aruco_pose_publisher import DualArucoPosePublisher


def test_coordinator_rejects_invalid_dynamic_target_pairs():
    assert HomographyPickPlace._validate_target_ids(2, 9) == ''
    assert 'different' in HomographyPickPlace._validate_target_ids(2, 2)
    assert '0..49' in HomographyPickPlace._validate_target_ids(-1, 9)
    assert '0..49' in HomographyPickPlace._validate_target_ids(2, 50)


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
