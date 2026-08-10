"""Tests for camera repeatability statistics."""

from arm2.arm2_camera_repeatability_monitor import (
    classify_pixel_delta,
    summarize_centers,
)

import numpy as np


def test_project_pixel_thresholds():
    """Project thresholds include their documented boundary values."""
    assert classify_pixel_delta(2.0) == 'stable'
    assert classify_pixel_delta(3.0) == 'warning'
    assert classify_pixel_delta(5.0) == 'warning'
    assert classify_pixel_delta(5.01) == 'unstable'


def test_summary_uses_median_and_marker_scale():
    """Summary resists an outlier and converts pixels using marker size."""
    points = [
        [300.0, 294.0, 68.0],
        [300.2, 293.8, 68.2],
        [320.0, 310.0, 68.4],
    ]
    result = summarize_centers(points, [300.0, 294.0], 26.0)
    assert np.isclose(result['median_u_px'], 300.2)
    assert np.isclose(result['median_v_px'], 294.0)
    assert np.isclose(result['delta_px'], 0.2)
    assert np.isclose(result['mm_per_px'], 26.0 / 68.2)
    assert result['status'] == 'stable'
