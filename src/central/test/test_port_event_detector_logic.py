"""Pure state-helper tests for port arrival event detection."""

from collections import deque

from central.port_event_detector import PortEventDetector


def test_stable_object_count_increase_requests_rescan():
    """A persistent added object must be treated as new inbound cargo."""
    counts = deque([1, 2, 2, 2], maxlen=5)

    result = PortEventDetector._stable_count_increase(counts, 1, 3)

    assert result == 2


def test_single_frame_count_spike_does_not_request_rescan():
    """A transient detector spike must not move the robot arm."""
    counts = deque([1, 1, 2, 1, 1], maxlen=5)

    result = PortEventDetector._stable_count_increase(counts, 1, 3)

    assert result is None


def test_same_occupied_scene_does_not_request_duplicate_rescan():
    """The already-scanned object count must remain idempotent."""
    counts = deque([2, 2, 2, 2, 2], maxlen=5)

    result = PortEventDetector._stable_count_increase(counts, 2, 3)

    assert result is None
