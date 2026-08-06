"""Tests for cumulative calibration state without ROS hardware."""

from types import SimpleNamespace

from arm_floor_calibration.calibration_math import fit_homography
from arm_floor_calibration.floor_calibrator import FloorCalibrator

import numpy as np

import pytest


def xy_samples(level=1, station='station_a'):
    """Return four valid correspondence dictionaries."""
    source = np.array([
        [-0.10, -0.20],
        [0.10, -0.20],
        [0.10, 0.00],
        [-0.10, 0.00],
    ])
    target = source + np.array([0.012, -0.007])
    return [
        {
            'floor': level,
            'station': station,
            'marker_xyz_m': [float(x), float(y), 0.02],
            'marker_yaw_deg': -90.0,
            'marker_std_m': [0.001, 0.001, 0.001],
            'taught_command_xy_m': [float(tx), float(ty)],
        }
        for (x, y), (tx, ty) in zip(source, target)
    ]


def bare_calibrator(path):
    """Construct only the persistence state of the ROS node."""
    calibrator = object.__new__(FloorCalibrator)
    calibrator.output_file = path
    calibrator.base_frame = 'arm/base_link'
    calibrator.marker_frame = 'arm/target_marker'
    calibrator.command_frame = 'arm/controller_coords'
    calibrator.ransac_threshold = 0.003
    calibrator.pending_marker = None
    calibrator.xy_samples = []
    calibrator.z_samples = {'pick': {}, 'place': {}}
    calibrator.fits = {}
    return calibrator


def test_existing_yaml_is_loaded_and_preserved(tmp_path):
    """Restarting the node retains old samples, Z values, and fitted H."""
    path = tmp_path / 'floor_calibration.yaml'
    original = bare_calibrator(path)
    original.xy_samples = xy_samples(1)
    source = [item['marker_xyz_m'][:2] for item in original.xy_samples]
    target = [item['taught_command_xy_m'] for item in original.xy_samples]
    original.fits[1] = fit_homography(source, target)
    original.z_samples['pick'][1] = [
        {'station': 'station_a', 'z_m': 0.101}
    ]
    original.z_samples['place'][1] = [
        {'station': 'station_a', 'z_m': 0.122}
    ]
    original.z_samples['pick']['agv_1'] = [
        {'station': 'station_agv', 'z_m': 0.085}
    ]
    original._save()

    restored = bare_calibrator(path)
    assert restored._load_existing() is True

    assert len(restored.xy_samples) == 4
    assert 1 in restored.fits
    assert restored.z_samples['place'][1][0]['z_m'] == pytest.approx(0.122)
    assert restored.z_samples['pick']['agv_1'][0]['z_m'] == pytest.approx(
        0.085
    )
    document = restored._document()['floor_calibration']['floors']
    assert list(document) == [0, 1, 2, 3, 'agv_0', 'agv_1']
    assert 'pick_z_samples' not in document[0]
    assert 'pick_z_samples' not in document['agv_0']
    assert document['agv_1']['pick_z_m'] == pytest.approx(0.085)


def test_active_surface_keeps_floor_commands_and_adds_agv():
    """Existing active_floor workflow remains valid alongside AGV mode."""
    calibrator = object.__new__(FloorCalibrator)
    values = {
        'active_surface': 'floor',
        'active_floor': 0,
        'active_station': 'station_a',
    }
    calibrator.parameter = values.__getitem__
    assert calibrator.active_labels() == (0, 'station_a')

    values['active_surface'] = 'agv'
    values['active_floor'] = 0
    values['active_station'] = 'station_agv'
    assert calibrator.active_labels() == ('agv_0', 'station_agv')
    values['active_floor'] = 1
    assert calibrator.active_labels() == ('agv_1', 'station_agv')

    values['active_floor'] = 2
    with pytest.raises(ValueError, match='0 or 1'):
        calibrator.active_labels()


def test_legacy_agv_level_migrates_to_agv_one():
    """The prior undivided AGV key remains readable without data loss."""
    assert FloorCalibrator._normalize_level('agv') == 'agv_1'


def test_floor_zero_rejects_z_capture():
    """Floor 0 supplies geometry but never owns Pick/Place Z values."""
    calibrator = object.__new__(FloorCalibrator)
    calibrator.active_labels = lambda: (0, 'station_a')
    response = SimpleNamespace(success=None, message='')

    calibrator._capture_z('place', response)

    assert response.success is False
    assert 'geometry-only' in response.message


def test_fit_supports_floor_zero_and_agv(tmp_path):
    """Both new geometry levels can produce independent homographies."""
    calibrator = bare_calibrator(tmp_path / 'floor_calibration.yaml')
    calibrator.xy_samples = (
        xy_samples(0, 'station_a')
        + xy_samples('agv_0', 'station_agv')
        + xy_samples('agv_1', 'station_agv')
    )
    response = SimpleNamespace(success=None, message='')

    calibrator.fit_and_save(None, response)

    assert response.success is True
    assert 0 in calibrator.fits
    assert 'agv_0' in calibrator.fits
    assert 'agv_1' in calibrator.fits
    assert 'floor 0:' in response.message
    assert 'floor agv_0:' in response.message
    assert 'floor agv_1:' in response.message


def test_agv_pending_marker_can_be_saved_as_xy_pair(tmp_path):
    """AGV's string level label survives the two-stage XY capture flow."""
    calibrator = bare_calibrator(tmp_path / 'floor_calibration.yaml')
    calibrator.pending_marker = {
        'floor': 'agv_1',
        'station': 'station_agv',
        'xyz_m': np.array([0.12, -0.24, 0.03]),
        'yaw_deg': 90.0,
        'std_m': np.array([0.001, 0.001, 0.001]),
    }
    transform = SimpleNamespace(
        transform=SimpleNamespace(
            translation=SimpleNamespace(x=0.11, y=-0.23)
        )
    )
    calibrator._lookup = lambda _frame: (transform, 0)
    response = SimpleNamespace(success=None, message='')

    calibrator.capture_xy_pair(None, response)

    assert response.success is True
    assert calibrator.xy_samples[0]['floor'] == 'agv_1'
    assert calibrator.pending_marker is None


def test_delete_active_group_removes_only_selected_station(tmp_path):
    """Group deletion preserves other stations and invalidates level H."""
    calibrator = bare_calibrator(tmp_path / 'floor_calibration.yaml')
    station_a = xy_samples(1, 'station_a')
    station_b = xy_samples(1, 'station_b')
    calibrator.xy_samples = station_a + station_b
    source = [item['marker_xyz_m'][:2] for item in station_a]
    target = [item['taught_command_xy_m'] for item in station_a]
    calibrator.fits[1] = fit_homography(source, target)
    calibrator.z_samples['pick'][1] = [
        {'station': 'station_a', 'z_m': 0.101},
        {'station': 'station_b', 'z_m': 0.102},
    ]
    calibrator.z_samples['place'][1] = [
        {'station': 'station_b', 'z_m': 0.123}
    ]
    calibrator.active_labels = lambda: (1, 'station_b')
    response = SimpleNamespace(success=None, message='')

    calibrator.delete_active_group(None, response)

    assert response.success is True
    assert len(calibrator.xy_samples) == 4
    assert all(
        item['station'] == 'station_a'
        for item in calibrator.xy_samples
    )
    assert calibrator.z_samples['pick'][1] == [
        {'station': 'station_a', 'z_m': 0.101}
    ]
    assert 1 not in calibrator.z_samples['place']
    assert 1 not in calibrator.fits
    assert 'XY=4' in response.message
