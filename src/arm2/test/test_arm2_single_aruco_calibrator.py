"""Tests for single ArUco camera-calibration helpers."""

from arm2.arm2_single_aruco_calibrator import marker_object_points
import numpy as np


def test_marker_object_points_are_26_mm_square():
    points = marker_object_points(0.026)
    assert points.shape == (4, 3)
    lengths = [
        np.linalg.norm(points[(index + 1) % 4] - points[index])
        for index in range(4)
    ]
    assert np.allclose(lengths, 0.026)
    assert np.allclose(points[:, 2], 0.0)
