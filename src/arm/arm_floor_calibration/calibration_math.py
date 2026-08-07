"""Pure math helpers for fitting and applying floor homographies."""

from dataclasses import dataclass

import cv2

import numpy as np


@dataclass(frozen=True)
class HomographyFit:
    """A fitted XY homography and its directly measured residuals."""

    matrix: np.ndarray
    inlier_mask: np.ndarray
    residuals_m: np.ndarray

    @property
    def inlier_count(self):
        """Return the number of RANSAC inliers."""
        return int(np.count_nonzero(self.inlier_mask))

    @property
    def rmse_m(self):
        """Return inlier root-mean-square error in metres."""
        selected = self.residuals_m[self.inlier_mask]
        return float(np.sqrt(np.mean(selected ** 2)))

    @property
    def max_error_m(self):
        """Return the largest inlier error in metres."""
        return float(np.max(self.residuals_m[self.inlier_mask]))


def _xy_array(points, name):
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 2:
        raise ValueError(f'{name} must have shape (N, 2)')
    if not np.all(np.isfinite(values)):
        raise ValueError(f'{name} contains a non-finite value')
    return values


def _validate_geometry(points, name):
    unique = np.unique(np.round(points, decimals=9), axis=0)
    if len(unique) < 4:
        raise ValueError(f'{name} needs at least four unique points')
    centered = points - np.mean(points, axis=0)
    singular_values = np.linalg.svd(centered, compute_uv=False)
    if singular_values[0] < 1e-9:
        raise ValueError(f'{name} has no measurable spread')
    if singular_values[1] / singular_values[0] < 0.02:
        raise ValueError(
            f'{name} is nearly collinear; spread samples over the work area'
        )


def apply_homography(matrix, points):
    """Project one or more metric XY points through a 3x3 homography."""
    values = _xy_array(points, 'points')
    homography = np.asarray(matrix, dtype=np.float64)
    if homography.shape != (3, 3) or not np.all(np.isfinite(homography)):
        raise ValueError('matrix must be a finite 3x3 array')
    homogeneous = np.column_stack((values, np.ones(len(values))))
    projected = (homography @ homogeneous.T).T
    scale = projected[:, 2]
    if np.any(np.abs(scale) < 1e-12):
        raise ValueError('homography projects a point to infinity')
    return projected[:, :2] / scale[:, None]


def fit_homography(source_xy, target_xy, ransac_threshold_m=0.003):
    """Fit marker-XY to taught-tool-XY with metric RANSAC residuals."""
    source = _xy_array(source_xy, 'source_xy')
    target = _xy_array(target_xy, 'target_xy')
    if len(source) != len(target):
        raise ValueError('source_xy and target_xy must have equal length')
    if len(source) < 4:
        raise ValueError('at least four correspondence pairs are required')
    if not np.isfinite(ransac_threshold_m) or ransac_threshold_m <= 0.0:
        raise ValueError('ransac_threshold_m must be positive')
    _validate_geometry(source, 'source_xy')
    _validate_geometry(target, 'target_xy')

    method = cv2.RANSAC if len(source) > 4 else 0
    matrix, mask = cv2.findHomography(
        source,
        target,
        method=method,
        ransacReprojThreshold=float(ransac_threshold_m),
    )
    if matrix is None or not np.all(np.isfinite(matrix)):
        raise ValueError('OpenCV could not compute a finite homography')
    if abs(float(matrix[2, 2])) < 1e-12:
        raise ValueError('homography normalization is singular')
    matrix = matrix / matrix[2, 2]
    inliers = (
        np.ones(len(source), dtype=bool)
        if mask is None else mask.reshape(-1).astype(bool)
    )
    if np.count_nonzero(inliers) < 4:
        raise ValueError('RANSAC retained fewer than four inliers')
    predicted = apply_homography(matrix, source)
    residuals = np.linalg.norm(predicted - target, axis=1)
    return HomographyFit(matrix, inliers, residuals)
