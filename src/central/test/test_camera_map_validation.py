from pathlib import Path

from central.camera_to_map_bridge import (
    CameraToMapBridge,
    validate_calibration_map,
    waiting_point_behind_target,
)

import numpy as np
import pytest
import yaml


def write_map(tmp_path, width=207, height=113):
    pgm_path = tmp_path / 'current_map.pgm'
    pgm_path.write_bytes(
        f'P5\n# generated test map\n{width} {height}\n255\n'.encode('ascii')
    )
    yaml_path = tmp_path / 'current_map.yaml'
    yaml_path.write_text(
        yaml.safe_dump({
            'image': pgm_path.name,
            'resolution': 0.01,
            'origin': [-0.198, -0.517, 0.0],
        }),
        encoding='utf-8',
    )
    return yaml_path


def calibration_for(map_yaml):
    return {
        'map': {
            'yaml': str(map_yaml),
            'width': 207,
            'height': 113,
            'resolution': 0.01,
            'origin': [-0.198, -0.517, 0.0],
        },
    }


def test_matching_map_is_accepted(tmp_path):
    map_yaml = write_map(tmp_path)
    calibration = calibration_for(map_yaml)

    validate_calibration_map(calibration, Path('/tmp/calibration.yaml'))


def test_replaced_map_is_rejected(tmp_path):
    map_yaml = write_map(tmp_path)
    calibration = calibration_for(map_yaml)
    calibration['map']['width'] = 205
    calibration['map']['height'] = 231
    calibration['map']['origin'] = [-1.366, -1.903, 0.0]

    with pytest.raises(ValueError, match='does not match the current SLAM map'):
        validate_calibration_map(calibration, Path('/tmp/calibration.yaml'))


def test_b1_offset_moves_toward_camera_image_left():
    bridge = object.__new__(CameraToMapBridge)
    bridge.homography = np.eye(3)

    offset_x, offset_y = bridge.camera_left_map_offset(
        [320.0, 240.0],
        0.15,
    )

    assert offset_x == pytest.approx(-0.15)
    assert offset_y == pytest.approx(0.0)


def test_b1_offset_moves_toward_camera_image_down():
    bridge = object.__new__(CameraToMapBridge)
    bridge.homography = np.eye(3)

    offset_x, offset_y = bridge.camera_down_map_offset(
        [320.0, 240.0],
        0.03,
    )

    assert offset_x == pytest.approx(0.0)
    assert offset_y == pytest.approx(0.03)


def test_waiting_point_is_behind_final_heading():
    waiting_x, waiting_y = waiting_point_behind_target(
        1.0,
        2.0,
        np.pi / 2.0,
        0.25,
    )

    assert waiting_x == pytest.approx(1.0)
    assert waiting_y == pytest.approx(1.75)
