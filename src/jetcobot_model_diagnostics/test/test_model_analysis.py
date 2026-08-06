import numpy as np
from scipy.spatial.transform import Rotation

from jetcobot_model_diagnostics.model_analysis import (
    controller_coords_matrix,
    flange_to_controller,
    summarize_transforms,
    transform_matrix,
)


def test_constant_flange_to_controller_is_consistent():
    fixed = transform_matrix(
        [0.018, -0.002, 0.004],
        Rotation.from_euler(
            'xyz', [-90.0, -45.0, -90.0], degrees=True
        ).as_quat(),
    )
    candidates = []
    for translation, rpy in (
        ([0.1, -0.2, 0.3], [10.0, 20.0, 30.0]),
        ([0.2, -0.1, 0.2], [-40.0, 5.0, 80.0]),
        ([-0.1, 0.1, 0.25], [100.0, -20.0, -45.0]),
    ):
        flange = transform_matrix(
            translation,
            Rotation.from_euler('xyz', rpy, degrees=True).as_quat(),
        )
        controller = flange @ fixed
        candidates.append(flange_to_controller(flange, controller))
    summary = summarize_transforms(candidates)
    assert summary['classification'] == 'CONSISTENT'
    assert summary['translation_max_mm'] < 1e-9
    assert summary['rotation_max_deg'] < 1e-9


def test_controller_coords_units_are_mm_and_degrees():
    matrix = controller_coords_matrix(
        [100.0, -200.0, 300.0, 0.0, 0.0, 90.0]
    )
    assert np.allclose(matrix[:3, 3], [0.1, -0.2, 0.3])
    assert np.allclose(
        matrix[:3, :3],
        Rotation.from_euler('xyz', [0.0, 0.0, 90.0], degrees=True)
        .as_matrix(),
    )


def test_pose_dependent_offset_is_inconsistent():
    matrices = [
        transform_matrix([0.0, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        transform_matrix([0.020, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
        transform_matrix([-0.020, 0.0, 0.0], [0.0, 0.0, 0.0, 1.0]),
    ]
    summary = summarize_transforms(matrices)
    assert summary['classification'] == 'INCONSISTENT'
    assert summary['translation_max_mm'] > 10.0
