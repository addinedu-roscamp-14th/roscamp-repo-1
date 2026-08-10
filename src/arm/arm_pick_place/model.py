"""Hardware-independent calibration, classification and sequence logic."""

import json
import math
from dataclasses import dataclass
from pathlib import Path

import cv2

import numpy as np

import yaml


@dataclass(frozen=True)
class Station:
    """One station-specific observation pose."""

    name: str
    joint_angles_deg: tuple[float, ...]
    timeout_sec: float
    calibration_surface: str


@dataclass(frozen=True)
class MarkerObservation:
    """A marker pose stabilized in the robot base frame."""

    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float
    station: str


@dataclass(frozen=True)
class FloorData:
    """One floor's trained geometry and manipulation heights."""

    number: int | str
    homography: np.ndarray
    marker_points: np.ndarray
    taught_points: np.ndarray
    plane_coefficients: np.ndarray
    plane_max_training_error_m: float
    pick_z_m: float | None
    place_z_m: float | None
    homography_inlier_count: int
    homography_sample_count: int


@dataclass(frozen=True)
class CalibratedTarget:
    """A marker converted to XY while retaining its detected marker floor."""

    marker_floor: int | str
    station: str
    x_m: float
    y_m: float
    yaw_deg: float
    raw: MarkerObservation


@dataclass(frozen=True)
class MotionStep:
    """One direct API action."""

    action: str
    pose: tuple[float, ...] | None = None


def wrap_degrees(value):
    """Wrap an angle to [-180, 180)."""
    return (float(value) + 180.0) % 360.0 - 180.0


def select_symmetric_yaw(nominal_yaw_deg, reference_yaw_deg):
    """Choose nominal yaw or its 180-degree gripper-symmetric branch."""
    nominal = wrap_degrees(nominal_yaw_deg)
    symmetric = wrap_degrees(nominal + 180.0)
    nominal_delta = abs(wrap_degrees(nominal - reference_yaw_deg))
    symmetric_delta = abs(wrap_degrees(symmetric - reference_yaw_deg))
    if symmetric_delta < nominal_delta:
        return symmetric, 180.0, symmetric_delta
    return nominal, 0.0, nominal_delta


def safe_z_candidates(
    configured_z_m,
    step_m,
    maximum_lowering_steps,
    minimum_z_m,
):
    """Return descending safe-Z candidates that retain minimum clearance."""
    configured = float(configured_z_m)
    step = float(step_m)
    minimum = float(minimum_z_m)
    attempts = int(maximum_lowering_steps)
    if not all(math.isfinite(value) for value in (configured, step, minimum)):
        raise ValueError('safe Z candidate inputs must be finite')
    if step <= 0.0:
        raise ValueError('safe_z_lowering_step_m must be positive')
    if attempts < 0:
        raise ValueError('maximum_safe_z_lowering_steps must be non-negative')
    return tuple(
        candidate
        for index in range(attempts + 1)
        if (candidate := configured - index * step) >= minimum - 1e-9
    )


def parse_stations(value):
    """Parse and validate the extensible station observation JSON."""
    try:
        raw = json.loads(value)
    except (TypeError, json.JSONDecodeError) as exc:
        raise ValueError(f'stations_json is invalid JSON: {exc}') from exc
    if not isinstance(raw, list) or not raw:
        raise ValueError('stations_json must be a non-empty JSON list')
    stations = []
    names = set()
    for index, item in enumerate(raw):
        if not isinstance(item, dict):
            raise ValueError(f'station {index} must be a JSON object')
        name = str(item.get('name', '')).strip()
        angles = item.get('joint_angles_deg')
        timeout = item.get('timeout_sec')
        surface = str(
            item.get('calibration_surface', 'station')
        ).strip().lower()
        if not name or name in names:
            raise ValueError(f'station {index} has an empty/duplicate name')
        if not isinstance(angles, list) or len(angles) != 6:
            raise ValueError(f'{name}.joint_angles_deg must have six values')
        angles = tuple(float(item) for item in angles)
        timeout = float(timeout)
        if not all(math.isfinite(item) for item in angles):
            raise ValueError(f'{name} contains a non-finite joint angle')
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError(f'{name}.timeout_sec must be positive')
        if surface not in ('station', 'agv'):
            raise ValueError(
                f'{name}.calibration_surface must be station or agv'
            )
        names.add(name)
        stations.append(Station(name, angles, timeout, surface))
    return tuple(stations)


def _finite_array(value, shape, name):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.all(np.isfinite(array)):
        raise ValueError(f'{name} must be a finite array with shape {shape}')
    return array


def _fit_plane(marker_points, floor):
    design = np.column_stack((
        marker_points[:, :2], np.ones(len(marker_points))
    ))
    coefficients, _, rank, _ = np.linalg.lstsq(
        design, marker_points[:, 2], rcond=None
    )
    if rank < 3:
        raise ValueError(f'floor {floor} marker points cannot fit a Z plane')
    residuals = np.abs(design @ coefficients - marker_points[:, 2])
    return coefficients, float(np.max(residuals))


def load_floor_calibration(path):
    """Load the collector YAML and derive a marker XYZ plane per floor."""
    source = Path(path).expanduser().resolve()
    try:
        document = yaml.safe_load(source.read_text(encoding='utf-8'))
        root = document['floor_calibration']
        raw_floors = root['floors']
    except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
        raise ValueError(f'cannot load calibration {source}: {exc}') from exc
    floors = {}
    required_levels = (1, 2, 3)
    for number in (0, 1, 2, 3, 'agv_0', 'agv_1'):
        try:
            entry = raw_floors[number]
        except KeyError:
            try:
                entry = raw_floors[str(number)]
            except KeyError as exc:
                if number not in required_levels:
                    continue
                raise ValueError(f'calibration has no floor {number}') from exc
        samples = entry.get('xy_samples', [])
        if number not in required_levels and not samples:
            continue
        if len(samples) < 4:
            raise ValueError(f'floor {number} needs at least four XY samples')
        marker_points = _finite_array(
            [item['marker_xyz_m'] for item in samples],
            (len(samples), 3),
            f'floor {number} marker points',
        )
        taught_points = _finite_array(
            [item['taught_command_xy_m'] for item in samples],
            (len(samples), 2),
            f'floor {number} taught points',
        )
        homography = _finite_array(
            entry['homography_marker_xy_to_command_xy'],
            (3, 3),
            f'floor {number} homography',
        )
        if abs(float(homography[2, 2])) < 1e-12:
            raise ValueError(f'floor {number} homography is singular')
        homography = homography / homography[2, 2]
        if number in (0, 'agv_0'):
            pick_z = None
            place_z = None
        else:
            try:
                pick_z = float(entry['pick_z_m'])
                place_z = float(entry['place_z_m'])
            except KeyError as exc:
                raise ValueError(
                    f'floor {number} requires Pick and Place Z'
                ) from exc
            if not all(math.isfinite(value) for value in (pick_z, place_z)):
                raise ValueError(
                    f'floor {number} Z contains a non-finite value'
                )
        plane, max_error = _fit_plane(marker_points, number)
        metrics = entry.get('homography_metrics', {})
        floors[number] = FloorData(
            number=number,
            homography=homography,
            marker_points=marker_points,
            taught_points=taught_points,
            plane_coefficients=plane,
            plane_max_training_error_m=max_error,
            pick_z_m=pick_z,
            place_z_m=place_z,
            homography_inlier_count=int(metrics.get('inlier_count', 0)),
            homography_sample_count=int(
                metrics.get('sample_count', len(samples))
            ),
        )
    return floors


def calibration_levels_for_surface(floors, surface):
    """Return only levels allowed at one configured observation pose."""
    normalized = str(surface).strip().lower()
    if normalized == 'station':
        return {
            level: data for level, data in floors.items()
            if isinstance(level, int) and not isinstance(level, bool)
        }
    if normalized == 'agv':
        return {
            level: data for level, data in floors.items()
            if isinstance(level, str) and level.startswith('agv_')
        }
    raise ValueError(f'unsupported calibration surface: {surface}')


def destination_level(support_level):
    """Map a detected support marker level to the resulting load level."""
    if isinstance(support_level, int) and not isinstance(support_level, bool):
        return support_level + 1
    if support_level == 'agv_0':
        return 'agv_1'
    if support_level == 'agv_1':
        return 'agv_2'
    raise ValueError(f'unsupported place support level {support_level}')


def floor_plane_errors(observation, floors):
    """Return absolute marker-Z residual for every calibrated floor plane."""
    vector = np.array(
        [observation.x_m, observation.y_m, 1.0], dtype=np.float64
    )
    return {
        number: abs(
            float(np.dot(floor.plane_coefficients, vector))
            - observation.z_m
        )
        for number, floor in floors.items()
    }


def classify_floor(
    observation,
    floors,
    maximum_error_m=0.010,
    minimum_separation_m=0.015,
):
    """Classify a marker by its distance to each learned TF XYZ plane."""
    errors = floor_plane_errors(observation, floors)
    if len(errors) < 2:
        raise ValueError(
            'floor classification requires at least two calibrated levels'
        )
    ordered = sorted(errors.items(), key=lambda item: item[1])
    best_floor, best_error = ordered[0]
    separation = ordered[1][1] - best_error
    if best_error > maximum_error_m:
        raise ValueError(
            f'marker is outside every floor plane: best=floor {best_floor}, '
            f'error={best_error * 1000.0:.1f} mm'
        )
    if separation < minimum_separation_m:
        raise ValueError(
            f'floor classification is ambiguous: best=floor {best_floor}, '
            f'gap={separation * 1000.0:.1f} mm, errors={errors}'
        )
    return best_floor, errors


def _inside_training_area(point, marker_points, margin_m):
    hull = cv2.convexHull(marker_points[:, :2].astype(np.float32))
    distance = cv2.pointPolygonTest(
        hull,
        (float(point[0]), float(point[1])),
        True,
    )
    return float(distance) >= -float(margin_m), float(distance)


def calibrate_target(
    observation,
    floors,
    maximum_floor_error_m=0.010,
    minimum_floor_separation_m=0.015,
    maximum_extrapolation_m=0.015,
    command_margin_m=0.020,
):
    """Classify floor, guard extrapolation, and apply its XY homography."""
    floor_number, errors = classify_floor(
        observation,
        floors,
        maximum_floor_error_m,
        minimum_floor_separation_m,
    )
    floor = floors[floor_number]
    raw_xy = np.array([observation.x_m, observation.y_m])
    inside, hull_distance = _inside_training_area(
        raw_xy, floor.marker_points, maximum_extrapolation_m
    )
    if not inside:
        raise ValueError(
            f'floor {floor_number} marker is too far outside the calibrated '
            f'XY area: {abs(hull_distance) * 1000.0:.1f} mm'
        )
    homogeneous = floor.homography @ np.array([
        observation.x_m, observation.y_m, 1.0
    ])
    if abs(float(homogeneous[2])) < 1e-8:
        raise ValueError('homography denominator is too close to zero')
    corrected = homogeneous[:2] / homogeneous[2]
    lower = np.min(floor.taught_points, axis=0) - command_margin_m
    upper = np.max(floor.taught_points, axis=0) + command_margin_m
    if np.any(corrected < lower) or np.any(corrected > upper):
        raise ValueError(
            f'corrected XY is outside the taught command area: '
            f'xy={corrected.tolist()}, '
            f'limits={lower.tolist()}..{upper.tolist()}'
        )
    target = CalibratedTarget(
        marker_floor=floor_number,
        station=observation.station,
        x_m=float(corrected[0]),
        y_m=float(corrected[1]),
        yaw_deg=float(observation.yaw_deg),
        raw=observation,
    )
    return target, errors


def target_with_command_yaw(target, command_yaw_deg, yaw_offset_deg):
    """Return a target whose generated tool pose uses an explicit yaw."""
    return CalibratedTarget(
        marker_floor=target.marker_floor,
        station=target.station,
        x_m=target.x_m,
        y_m=target.y_m,
        yaw_deg=wrap_degrees(command_yaw_deg - yaw_offset_deg),
        raw=target.raw,
    )


def tool_pose(target, z_m, yaw_offset_deg):
    """Create the required direct-controller [XYZ, RPY] pose."""
    return (
        target.x_m,
        target.y_m,
        float(z_m),
        -180.0,
        0.0,
        wrap_degrees(target.yaw_deg + yaw_offset_deg),
    )


def tool_pose_with_rpy(target, z_m, rpy_deg):
    """Create a target XY/Z while preserving an explicitly supplied RPY."""
    if len(rpy_deg) != 3:
        raise ValueError('rpy_deg must contain roll, pitch, and yaw')
    values = tuple(float(value) for value in rpy_deg)
    if not all(math.isfinite(value) for value in values):
        raise ValueError('rpy_deg contains a non-finite value')
    return (
        target.x_m,
        target.y_m,
        float(z_m),
        *values,
    )


def build_pick_place_steps(
    pick,
    place,
    floors,
    pick_safe_z_m,
    place_safe_z_m,
    yaw_offset_deg=-45.0,
):
    """Build Pick/Place with independent safe heights for both roles."""
    pick_floor = pick.marker_floor
    support_floor = place.marker_floor
    destination_floor = destination_level(support_floor)
    if pick_floor not in floors or floors[pick_floor].pick_z_m is None:
        raise ValueError(f'pick floor {pick_floor} has no taught Pick Z')
    if destination_floor not in floors:
        raise ValueError(
            f'place support floor {support_floor} creates unsupported '
            f'destination floor {destination_floor}'
        )
    if floors[destination_floor].place_z_m is None:
        raise ValueError(
            f'destination floor {destination_floor} has no taught Place Z'
        )
    return (
        MotionStep('gripper_open'),
        MotionStep('move', tool_pose(pick, pick_safe_z_m, yaw_offset_deg)),
        MotionStep(
            'move',
            tool_pose(
                pick, floors[pick_floor].pick_z_m, yaw_offset_deg
            ),
        ),
        MotionStep('gripper_close'),
        MotionStep('move', tool_pose(pick, pick_safe_z_m, yaw_offset_deg)),
        MotionStep('move', tool_pose(place, place_safe_z_m, yaw_offset_deg)),
        MotionStep(
            'move',
            tool_pose(
                place,
                floors[destination_floor].place_z_m,
                yaw_offset_deg,
            ),
        ),
        MotionStep('gripper_open_after_stop'),
        MotionStep('move', tool_pose(place, place_safe_z_m, yaw_offset_deg)),
    )


def build_split_pick_place_steps(
    pick,
    place,
    floors,
    pick_safe_z_m,
    place_safe_z_m,
    current_rpy_deg,
    yaw_offset_deg=-45.0,
):
    """Split safe XY translation and in-place target-RPY rotation."""
    pick_floor = pick.marker_floor
    support_floor = place.marker_floor
    destination_floor = destination_level(support_floor)
    if pick_floor not in floors or floors[pick_floor].pick_z_m is None:
        raise ValueError(f'pick floor {pick_floor} has no taught Pick Z')
    if destination_floor not in floors:
        raise ValueError(
            f'place support floor {support_floor} creates unsupported '
            f'destination floor {destination_floor}'
        )
    if floors[destination_floor].place_z_m is None:
        raise ValueError(
            f'destination floor {destination_floor} has no taught Place Z'
        )
    pick_target = tool_pose(pick, pick_safe_z_m, yaw_offset_deg)
    place_target = tool_pose(place, place_safe_z_m, yaw_offset_deg)
    pick_target_rpy = pick_target[3:]
    return (
        MotionStep('gripper_open'),
        # Reach the Pick XY without simultaneously requesting a new RPY.
        MotionStep(
            'move',
            tool_pose_with_rpy(pick, pick_safe_z_m, current_rpy_deg),
        ),
        MotionStep('move', pick_target),
        MotionStep(
            'move',
            tool_pose(
                pick, floors[pick_floor].pick_z_m, yaw_offset_deg
            ),
        ),
        MotionStep('gripper_close'),
        MotionStep('move', pick_target),
        # Preserve the Pick RPY while translating to the Place XY.
        MotionStep(
            'move',
            tool_pose_with_rpy(place, place_safe_z_m, pick_target_rpy),
        ),
        MotionStep('move', place_target),
        MotionStep(
            'move',
            tool_pose(
                place,
                floors[destination_floor].place_z_m,
                yaw_offset_deg,
            ),
        ),
        MotionStep('gripper_open_after_stop'),
        MotionStep('move', place_target),
    )
