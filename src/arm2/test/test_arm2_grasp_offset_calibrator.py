"""Tests for arm2 grasp-offset teaching calculations."""

from arm2.arm2_grasp_offset_calibrator import calculate_taught_offset
from arm2.arm2_grasp_offset_calibrator import normalize_quaternion
import numpy as np


def test_calculate_taught_offset_uses_base_frame_difference():
    """The taught offset is TCP position minus marker position."""
    marker_rotation = [0.0, 0.0, 0.0, 1.0]
    tcp_rotation = normalize_quaternion([0.1, -0.2, 0.3, 0.9])
    samples = []
    for noise in (-0.0001, 0.0, 0.0001):
        samples.append({
            'marker_position': [0.10 + noise, -0.20, 0.05],
            'marker_rotation': marker_rotation,
            'tcp_position': [0.08 + noise, -0.21, 0.02],
            'tcp_rotation': tcp_rotation,
        })

    result = calculate_taught_offset(samples)

    assert np.allclose(
        result['grasp_offset_xyz_m'], [-0.02, -0.01, -0.03]
    )
    assert np.allclose(result['offset_std_m'], [0.0, 0.0, 0.0])
    assert result['reference_marker_yaw_deg'] == 0.0


def test_quaternion_average_accepts_equivalent_signs():
    """Equivalent positive and negative quaternions do not cancel out."""
    samples = [
        {
            'marker_position': [0.0, 0.0, 0.0],
            'marker_rotation': [0.0, 0.0, 0.0, 1.0],
            'tcp_position': [0.0, 0.0, 0.0],
            'tcp_rotation': [0.0, 0.0, 0.0, 1.0],
        },
        {
            'marker_position': [0.0, 0.0, 0.0],
            'marker_rotation': [0.0, 0.0, 0.0, -1.0],
            'tcp_position': [0.0, 0.0, 0.0],
            'tcp_rotation': [0.0, 0.0, 0.0, -1.0],
        },
    ]

    result = calculate_taught_offset(samples)

    assert np.allclose(result['grasp_offset_rpy_deg'], [0.0, 0.0, 0.0])
