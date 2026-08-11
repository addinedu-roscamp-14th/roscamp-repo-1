"""Rigid-transform analysis for URDF and controller coordinate samples."""

from dataclasses import dataclass

import numpy as np
from scipy.spatial.transform import Rotation


def transform_matrix(translation, quaternion_xyzw):
    """Return a homogeneous transform from translation and XYZW quaternion."""
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_quat(
        np.asarray(quaternion_xyzw, dtype=np.float64)
    ).as_matrix()
    matrix[:3, 3] = np.asarray(translation, dtype=np.float64)
    return matrix


def controller_coords_matrix(coords):
    """Convert pymycobot millimetre/degree XYZ-RPY into a transform."""
    values = np.asarray(coords, dtype=np.float64)
    if values.shape != (6,) or not np.all(np.isfinite(values)):
        raise ValueError('controller coords must contain six finite values')
    matrix = np.eye(4, dtype=np.float64)
    matrix[:3, :3] = Rotation.from_euler(
        'xyz', values[3:], degrees=True
    ).as_matrix()
    matrix[:3, 3] = values[:3] / 1000.0
    return matrix


def flange_to_controller(urdf_base_to_flange, controller_base_pose):
    """Return the fixed-frame candidate URDF-flange to controller."""
    return np.linalg.inv(urdf_base_to_flange) @ controller_base_pose


def mean_transform(matrices):
    """Return a translation/rotation mean for homogeneous transforms."""
    values = np.asarray(matrices, dtype=np.float64)
    if values.ndim != 3 or values.shape[1:] != (4, 4):
        raise ValueError('matrices must have shape (N, 4, 4)')
    if len(values) == 0:
        raise ValueError('at least one transform is required')
    result = np.eye(4, dtype=np.float64)
    result[:3, 3] = np.mean(values[:, :3, 3], axis=0)
    result[:3, :3] = Rotation.from_matrix(
        values[:, :3, :3]
    ).mean().as_matrix()
    return result


def transform_delta(reference, measured):
    """Return measured expressed relative to reference."""
    return np.linalg.inv(reference) @ measured


def rotation_angle_degrees(matrix):
    """Return the shortest rotation angle represented by a transform."""
    return float(
        np.degrees(Rotation.from_matrix(matrix[:3, :3]).magnitude())
    )


@dataclass(frozen=True)
class AnalysisThresholds:
    """Thresholds used only to label a diagnostic result."""

    consistent_translation_mm: float = 5.0
    consistent_rotation_deg: float = 3.0
    marginal_translation_mm: float = 10.0
    marginal_rotation_deg: float = 5.0


def summarize_transforms(matrices, thresholds=None):
    """Summarize fixed-transform consistency and classify the result."""
    if thresholds is None:
        thresholds = AnalysisThresholds()
    values = np.asarray(matrices, dtype=np.float64)
    average = mean_transform(values)
    translation_errors_mm = []
    rotation_errors_deg = []
    for value in values:
        delta = transform_delta(average, value)
        translation_errors_mm.append(
            float(np.linalg.norm(delta[:3, 3]) * 1000.0)
        )
        rotation_errors_deg.append(rotation_angle_degrees(delta))
    translation_errors_mm = np.asarray(translation_errors_mm)
    rotation_errors_deg = np.asarray(rotation_errors_deg)
    max_translation = float(np.max(translation_errors_mm))
    max_rotation = float(np.max(rotation_errors_deg))
    if (
        max_translation <= thresholds.consistent_translation_mm
        and max_rotation <= thresholds.consistent_rotation_deg
    ):
        classification = 'CONSISTENT'
    elif (
        max_translation <= thresholds.marginal_translation_mm
        and max_rotation <= thresholds.marginal_rotation_deg
    ):
        classification = 'MARGINAL'
    else:
        classification = 'INCONSISTENT'
    return {
        'mean_matrix': average,
        'translation_errors_mm': translation_errors_mm,
        'rotation_errors_deg': rotation_errors_deg,
        'translation_rms_mm': float(np.sqrt(np.mean(
            translation_errors_mm ** 2
        ))),
        'translation_max_mm': max_translation,
        'rotation_rms_deg': float(np.sqrt(np.mean(
            rotation_errors_deg ** 2
        ))),
        'rotation_max_deg': max_rotation,
        'classification': classification,
    }


def transform_components(matrix):
    """Return serializable translation, quaternion, and XYZ Euler values."""
    rotation = Rotation.from_matrix(matrix[:3, :3])
    return {
        'translation_m': matrix[:3, 3].tolist(),
        'quaternion_xyzw': rotation.as_quat().tolist(),
        'rpy_deg': rotation.as_euler('xyz', degrees=True).tolist(),
    }
