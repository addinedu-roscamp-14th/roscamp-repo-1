"""Tests for floor homography fitting without ROS or hardware."""

from arm_floor_calibration.calibration_math import (
    apply_homography,
    fit_homography,
)

import numpy as np

import pytest


def test_fit_recovers_known_projective_mapping():
    """Recover a known projective transform from exact pairs."""
    source = np.array([
        [-0.20, -0.10], [0.20, -0.10], [0.20, 0.10], [-0.20, 0.10],
        [0.00, 0.00], [0.12, 0.04],
    ])
    expected = np.array([
        [1.02, 0.03, 0.011],
        [-0.02, 0.98, -0.007],
        [0.04, -0.02, 1.0],
    ])
    target = apply_homography(expected, source)
    fit = fit_homography(source, target)

    assert fit.inlier_count == len(source)
    assert fit.rmse_m < 1e-8
    assert np.allclose(fit.matrix, expected, atol=1e-7)


def test_fit_rejects_nearly_collinear_samples():
    """Reject sample layouts that cannot constrain a useful H."""
    source = np.array([
        [0.0, 0.0], [0.1, 0.0001], [0.2, 0.0002], [0.3, 0.0003]
    ])
    with pytest.raises(ValueError, match='nearly collinear'):
        fit_homography(source, source)


def test_fit_requires_four_pairs():
    """Require the mathematical minimum correspondence count."""
    points = np.array([[0.0, 0.0], [0.1, 0.0], [0.0, 0.1]])
    with pytest.raises(ValueError, match='at least four'):
        fit_homography(points, points)
