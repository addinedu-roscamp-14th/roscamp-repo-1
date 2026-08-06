"""Collect marker/tool correspondences and taught floor heights from TF."""

import math
import threading
from collections import deque
from datetime import datetime, timezone
from pathlib import Path

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.time import Time

from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformException, TransformListener

import yaml

from .calibration_math import (
    HomographyFit,
    apply_homography,
    fit_homography,
)


CALIBRATION_LEVELS = (0, 1, 2, 3, 'agv_0', 'agv_1')
Z_CAPABLE_LEVELS = (1, 2, 3, 'agv_1')


def yaw_degrees(rotation):
    """Extract wrapped yaw in degrees from a geometry quaternion."""
    sin_yaw = 2.0 * (
        rotation.w * rotation.z + rotation.x * rotation.y
    )
    cos_yaw = 1.0 - 2.0 * (
        rotation.y * rotation.y + rotation.z * rotation.z
    )
    return (math.degrees(math.atan2(sin_yaw, cos_yaw)) + 180.0) % 360.0 - 180.0


class FloorCalibrator(Node):
    """Expose explicit Trigger services for a safe, two-stage teach flow."""

    def __init__(self):
        """Initialize TF sampling buffers and calibration services."""
        super().__init__('floor_calibrator')
        self._declare_parameters()
        self.base_frame = str(self.parameter('base_frame'))
        self.marker_frame = str(self.parameter('marker_frame'))
        self.command_frame = str(self.parameter('command_frame'))
        self.sample_count = int(self.parameter('marker_sample_count'))
        self.sample_window = float(self.parameter('marker_sample_window_sec'))
        self.max_age = float(self.parameter('max_tf_age_sec'))
        self.max_std = float(self.parameter('max_marker_std_m'))
        self.ransac_threshold = float(
            self.parameter('ransac_threshold_m')
        )
        self.output_file = Path(
            str(self.parameter('output_file'))
        ).expanduser().resolve()
        self._validate_parameters()

        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.marker_history = deque(maxlen=max(100, self.sample_count * 10))
        self.last_marker_stamp = None
        self.lock = threading.Lock()
        self.pending_marker = None
        self.xy_samples = []
        self.z_samples = {'pick': {}, 'place': {}}
        self.fits = {}
        loaded = self._load_existing()

        self.create_timer(0.05, self.collect_marker)
        self._service('capture_marker', self.capture_marker)
        self._service('capture_xy_pair', self.capture_xy_pair)
        self._service('capture_pick_z', self.capture_pick_z)
        self._service('capture_place_z', self.capture_place_z)
        self._service('fit_and_save', self.fit_and_save)
        self._service('undo_last_xy', self.undo_last_xy)
        self._service('delete_active_group', self.delete_active_group)
        self._service('show_status', self.show_status)
        self.get_logger().info(
            'Floor calibration ready. No robot motion is commanded by this '
            'node. Use manual_jog in another terminal.'
        )
        self.get_logger().info(
            f'Frames: {self.base_frame} -> marker={self.marker_frame}, '
            f'tool={self.command_frame}; output={self.output_file}'
        )
        if loaded:
            self.get_logger().info(
                f'Loaded existing calibration: XY pairs='
                f'{len(self.xy_samples)}, fitted_levels='
                f'{sorted(self.fits, key=str)}'
            )

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'arm/base_link')
        self.declare_parameter('marker_frame', 'arm/target_marker')
        self.declare_parameter('command_frame', 'arm/controller_coords')
        self.declare_parameter('active_floor', 1)
        self.declare_parameter('active_surface', 'floor')
        self.declare_parameter('active_station', 'station_a')
        self.declare_parameter('marker_sample_count', 10)
        self.declare_parameter('marker_sample_window_sec', 2.0)
        self.declare_parameter('max_tf_age_sec', 1.0)
        self.declare_parameter('max_marker_std_m', 0.003)
        self.declare_parameter('ransac_threshold_m', 0.003)
        self.declare_parameter(
            'output_file', 'calibration_results/floor_calibration.yaml'
        )

    def parameter(self, name):
        """Return the current ROS parameter value."""
        return self.get_parameter(name).value

    def _validate_parameters(self):
        if self.sample_count < 3:
            raise ValueError('marker_sample_count must be at least 3')
        for name, value in (
            ('marker_sample_window_sec', self.sample_window),
            ('max_tf_age_sec', self.max_age),
            ('max_marker_std_m', self.max_std),
            ('ransac_threshold_m', self.ransac_threshold),
        ):
            if not math.isfinite(value) or value <= 0.0:
                raise ValueError(f'{name} must be positive')

    def _service(self, suffix, callback):
        self.create_service(
            Trigger, f'/arm/floor_calibration/{suffix}', callback
        )

    def active_labels(self):
        """Return validated runtime level and station labels."""
        surface = str(self.parameter('active_surface')).strip().lower()
        floor = int(self.parameter('active_floor'))
        station = str(self.parameter('active_station')).strip()
        if surface == 'floor':
            if floor not in (0, 1, 2, 3):
                raise ValueError('active_floor must be 0, 1, 2, or 3')
            level = floor
        elif surface == 'agv':
            if floor not in (0, 1):
                raise ValueError(
                    'active_floor must be 0 or 1 for active_surface=agv'
                )
            level = f'agv_{floor}'
        else:
            raise ValueError('active_surface must be floor or agv')
        if not station:
            raise ValueError('active_station must not be empty')
        return level, station

    @staticmethod
    def _normalize_level(value):
        """Normalize YAML integer/string level keys."""
        if isinstance(value, bool):
            raise ValueError(f'invalid calibration level: {value!r}')
        if isinstance(value, int) and value in (0, 1, 2, 3):
            return value
        text = str(value).strip().lower()
        # Version 1 briefly stored one undivided AGV group. Treat it as the
        # loaded-container level so old data is preserved during migration.
        if text == 'agv':
            return 'agv_1'
        if text in ('agv_0', 'agv_1'):
            return text
        if text in ('0', '1', '2', '3'):
            return int(text)
        raise ValueError(f'invalid calibration level: {value!r}')

    @staticmethod
    def _restore_fit(entry, samples, level):
        """Rebuild an in-memory HomographyFit from persisted YAML."""
        if 'homography_marker_xy_to_command_xy' not in entry:
            return None
        matrix = np.asarray(
            entry['homography_marker_xy_to_command_xy'],
            dtype=np.float64,
        )
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            raise ValueError(f'level {level} has an invalid homography')
        if abs(float(matrix[2, 2])) < 1e-12:
            raise ValueError(f'level {level} homography is singular')
        matrix = matrix / matrix[2, 2]
        if not samples:
            raise ValueError(
                f'level {level} has a homography without XY samples'
            )
        metrics = entry.get('homography_metrics', {})
        count = len(samples)
        mask = np.asarray(
            metrics.get('inlier_mask', [1] * count), dtype=bool
        ).reshape(-1)
        residuals_value = metrics.get('all_residuals_m')
        if residuals_value is None:
            source = [item['marker_xyz_m'][:2] for item in samples]
            target = np.asarray([
                item['taught_command_xy_m'] for item in samples
            ], dtype=np.float64)
            predicted = apply_homography(matrix, source)
            residuals = np.linalg.norm(predicted - target, axis=1)
        else:
            residuals = np.asarray(
                residuals_value, dtype=np.float64
            ).reshape(-1)
        if len(mask) != count or len(residuals) != count:
            raise ValueError(
                f'level {level} homography metrics do not match '
                f'{count} XY samples'
            )
        if not np.all(np.isfinite(residuals)):
            raise ValueError(
                f'level {level} homography residuals are non-finite'
            )
        if np.count_nonzero(mask) < 4:
            raise ValueError(
                f'level {level} homography has fewer than four inliers'
            )
        return HomographyFit(matrix, mask, residuals)

    def _load_existing(self):
        """Load prior samples, Z values, and fits without modifying disk."""
        if not self.output_file.exists():
            return False
        try:
            document = yaml.safe_load(
                self.output_file.read_text(encoding='utf-8')
            )
            root = document['floor_calibration']
            stored_levels = root['floors']
        except (OSError, TypeError, KeyError, yaml.YAMLError) as exc:
            raise ValueError(
                f'cannot load existing calibration '
                f'{self.output_file}: {exc}'
            ) from exc
        if not isinstance(stored_levels, dict):
            raise ValueError('existing calibration floors must be a mapping')

        xy_samples = []
        z_samples = {'pick': {}, 'place': {}}
        fits = {}
        seen = set()
        for raw_level, raw_entry in stored_levels.items():
            level = self._normalize_level(raw_level)
            if level in seen:
                raise ValueError(f'duplicate calibration level {level}')
            seen.add(level)
            if not isinstance(raw_entry, dict):
                raise ValueError(f'level {level} entry must be a mapping')
            level_xy = []
            for raw_sample in raw_entry.get('xy_samples', []):
                if not isinstance(raw_sample, dict):
                    raise ValueError(
                        f'level {level} contains an invalid XY sample'
                    )
                sample = dict(raw_sample)
                sample_level = self._normalize_level(
                    sample.get('floor', level)
                )
                if sample_level != level:
                    raise ValueError(
                        f'level {level} contains sample for {sample_level}'
                    )
                sample['floor'] = level
                level_xy.append(sample)
                xy_samples.append(sample)
            for kind in ('pick', 'place'):
                taught = raw_entry.get(f'{kind}_z_samples', [])
                if taught and level not in Z_CAPABLE_LEVELS:
                    raise ValueError(
                        f'level {level} may not contain {kind} Z samples'
                    )
                if taught:
                    z_samples[kind][level] = [
                        dict(item) for item in taught
                    ]
            fit = self._restore_fit(raw_entry, level_xy, level)
            if fit is not None:
                fits[level] = fit
        self.xy_samples = xy_samples
        self.z_samples = z_samples
        self.fits = fits
        return True

    @staticmethod
    def _stamp_ns(transform):
        return (
            int(transform.header.stamp.sec) * 1_000_000_000
            + int(transform.header.stamp.nanosec)
        )

    def _lookup(self, child_frame):
        transform = self.buffer.lookup_transform(
            self.base_frame, child_frame, Time()
        )
        stamp_ns = self._stamp_ns(transform)
        age = (self.get_clock().now().nanoseconds - stamp_ns) / 1e9
        if age < -0.1 or age > self.max_age:
            raise RuntimeError(
                f'{child_frame} TF is stale: age={age:.3f} s'
            )
        return transform, stamp_ns

    def collect_marker(self):
        """Append a fresh marker transform to the stability window."""
        try:
            transform, stamp_ns = self._lookup(self.marker_frame)
        except (TransformException, RuntimeError):
            return
        if stamp_ns == self.last_marker_stamp:
            return
        translation = transform.transform.translation
        sample = {
            'stamp_ns': stamp_ns,
            'xyz_m': np.array([
                translation.x, translation.y, translation.z
            ], dtype=np.float64),
            'yaw_deg': yaw_degrees(transform.transform.rotation),
        }
        with self.lock:
            self.marker_history.append(sample)
            self.last_marker_stamp = stamp_ns

    def _stable_marker(self):
        now_ns = self.get_clock().now().nanoseconds
        with self.lock:
            recent = [
                item for item in self.marker_history
                if (now_ns - item['stamp_ns']) / 1e9 <= self.sample_window
            ][-self.sample_count:]
        if len(recent) < self.sample_count:
            raise RuntimeError(
                f'need {self.sample_count} fresh marker samples; '
                f'currently {len(recent)}'
            )
        xyz = np.asarray([item['xyz_m'] for item in recent])
        std = np.std(xyz, axis=0)
        if float(np.max(std)) > self.max_std:
            raise RuntimeError(
                'marker is not stable: xyz std mm='
                f'{[round(value * 1000.0, 2) for value in std]}'
            )
        yaws = np.radians([item['yaw_deg'] for item in recent])
        mean_yaw = math.degrees(math.atan2(
            float(np.mean(np.sin(yaws))),
            float(np.mean(np.cos(yaws))),
        ))
        return {
            'xyz_m': np.mean(xyz, axis=0),
            'std_m': std,
            'yaw_deg': mean_yaw,
            'sample_count': len(recent),
        }

    def capture_marker(self, _request, response):
        """Freeze a stable marker observation before the robot is jogged."""
        try:
            level, station = self.active_labels()
            marker = self._stable_marker()
            self.pending_marker = {
                'floor': level,
                'station': station,
                **marker,
            }
            xyz = marker['xyz_m']
            response.success = True
            response.message = (
                f'frozen level={level}, station={station}, marker xyz='
                f'[{xyz[0]:.5f}, {xyz[1]:.5f}, {xyz[2]:.5f}] m, '
                f'yaw={marker["yaw_deg"]:.2f} deg. '
                'Now jog the tool to the true marker center and call '
                'capture_xy_pair.'
            )
        except (RuntimeError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def capture_xy_pair(self, _request, response):
        """Pair the frozen marker XY with the currently taught tool XY."""
        if self.pending_marker is None:
            response.success = False
            response.message = 'capture_marker must succeed first'
            return response
        try:
            transform, _ = self._lookup(self.command_frame)
            translation = transform.transform.translation
            pending = self.pending_marker
            sample = {
                'floor': self._normalize_level(pending['floor']),
                'station': str(pending['station']),
                'marker_xyz_m': [
                    float(value) for value in pending['xyz_m']
                ],
                'marker_yaw_deg': float(pending['yaw_deg']),
                'marker_std_m': [
                    float(value) for value in pending['std_m']
                ],
                'taught_command_xy_m': [
                    float(translation.x), float(translation.y)
                ],
            }
            self.xy_samples.append(sample)
            self.fits.pop(sample['floor'], None)
            self.pending_marker = None
            self._save()
            count = sum(
                item['floor'] == sample['floor']
                for item in self.xy_samples
            )
            response.success = True
            response.message = (
                f'pair saved for level {sample["floor"]}: '
                f'{count} total; taught xy='
                f'{sample["taught_command_xy_m"]}. '
                'Reposition the container before the next pair.'
            )
        except (OSError, RuntimeError, TransformException) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def _capture_z(self, kind, response):
        try:
            level, station = self.active_labels()
            if level not in Z_CAPABLE_LEVELS:
                raise ValueError(
                    f'level {level} is geometry-only; teach destination '
                    'floor/AGV level 1 for its Place Z'
                )
            transform, _ = self._lookup(self.command_frame)
            z_m = float(transform.transform.translation.z)
            self.z_samples[kind].setdefault(level, []).append({
                'station': station,
                'z_m': z_m,
            })
            self._save()
            values = [
                item['z_m'] for item in self.z_samples[kind][level]
            ]
            response.success = True
            response.message = (
                f'{kind} level {level} Z saved: {z_m:.5f} m; '
                f'n={len(values)}, mean={np.mean(values):.5f} m'
            )
        except (OSError, RuntimeError, TransformException, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def capture_pick_z(self, _request, response):
        """Store the current command-frame Z as a pick height."""
        return self._capture_z('pick', response)

    def capture_place_z(self, _request, response):
        """Store the current command-frame Z as a place height."""
        return self._capture_z('place', response)

    def fit_and_save(self, _request, response):
        """Fit every floor with enough data and persist the result."""
        try:
            fitted = []
            errors = []
            for floor in CALIBRATION_LEVELS:
                samples = [
                    item for item in self.xy_samples
                    if item['floor'] == floor
                ]
                if not samples:
                    continue
                if len(samples) < 4:
                    errors.append(
                        f'floor {floor}: need {4 - len(samples)} more pair(s)'
                    )
                    continue
                source = [item['marker_xyz_m'][:2] for item in samples]
                target = [item['taught_command_xy_m'] for item in samples]
                try:
                    fit = fit_homography(
                        source, target, self.ransac_threshold
                    )
                except ValueError as exc:
                    errors.append(f'floor {floor}: {exc}')
                    continue
                self.fits[floor] = fit
                fitted.append(
                    f'floor {floor}: {fit.inlier_count}/{len(samples)} '
                    f'inliers, rmse={fit.rmse_m * 1000.0:.2f} mm, '
                    f'max={fit.max_error_m * 1000.0:.2f} mm'
                )
            self._save()
            response.success = bool(fitted) and not errors
            response.message = '; '.join(fitted + errors) or 'no XY samples'
        except OSError as exc:
            response.success = False
            response.message = str(exc)
        return response

    def undo_last_xy(self, _request, response):
        """Remove the most recently stored XY correspondence."""
        if not self.xy_samples:
            response.success = False
            response.message = 'there is no XY pair to remove'
            return response
        removed = self.xy_samples.pop()
        self.fits.pop(removed['floor'], None)
        try:
            self._save()
        except OSError as exc:
            response.success = False
            response.message = str(exc)
            return response
        response.success = True
        response.message = (
            f'removed last XY pair: floor={removed["floor"]}, '
            f'station={removed["station"]}'
        )
        return response

    def delete_active_group(self, _request, response):
        """Delete all samples for the selected level and station."""
        try:
            level, station = self.active_labels()
            xy_before = len(self.xy_samples)
            self.xy_samples = [
                item for item in self.xy_samples
                if not (
                    item['floor'] == level
                    and item['station'] == station
                )
            ]
            removed_xy = xy_before - len(self.xy_samples)
            removed_z = {}
            for kind in ('pick', 'place'):
                existing = self.z_samples[kind].get(level, [])
                retained = [
                    item for item in existing
                    if item['station'] != station
                ]
                removed_z[kind] = len(existing) - len(retained)
                if retained:
                    self.z_samples[kind][level] = retained
                else:
                    self.z_samples[kind].pop(level, None)
            pending_removed = bool(
                self.pending_marker is not None
                and self.pending_marker['floor'] == level
                and self.pending_marker['station'] == station
            )
            if pending_removed:
                self.pending_marker = None
            removed_total = (
                removed_xy + removed_z['pick'] + removed_z['place']
            )
            if removed_total == 0 and not pending_removed:
                response.success = False
                response.message = (
                    f'no data for level={level}, station={station}'
                )
                return response
            # Any H fitted with the removed samples is now stale.
            self.fits.pop(level, None)
            self._save()
            response.success = True
            response.message = (
                f'deleted level={level}, station={station}: '
                f'XY={removed_xy}, pick_z={removed_z["pick"]}, '
                f'place_z={removed_z["place"]}, '
                f'pending_marker={pending_removed}; '
                f'level {level} H invalidated, call fit_and_save'
            )
        except (OSError, ValueError) as exc:
            response.success = False
            response.message = str(exc)
        return response

    def show_status(self, _request, response):
        """Report sample counts and fitted floors."""
        xy_counts = {
            floor: sum(item['floor'] == floor for item in self.xy_samples)
            for floor in CALIBRATION_LEVELS
        }
        z_counts = {
            kind: {
                floor: len(self.z_samples[kind].get(floor, []))
                for floor in CALIBRATION_LEVELS
            }
            for kind in ('pick', 'place')
        }
        response.success = True
        response.message = (
            f'XY pairs={xy_counts}; Z samples={z_counts}; '
            f'pending_marker={self.pending_marker is not None}; '
            f'fitted_levels={sorted(self.fits, key=str)}; '
            f'file={self.output_file}'
        )
        return response

    def _document(self):
        floors = {}
        for floor in CALIBRATION_LEVELS:
            xy = [item for item in self.xy_samples if item['floor'] == floor]
            entry = {'xy_samples': xy}
            if floor in Z_CAPABLE_LEVELS:
                for kind in ('pick', 'place'):
                    taught = self.z_samples[kind].get(floor, [])
                    entry[f'{kind}_z_samples'] = taught
                    if taught:
                        values = np.asarray([
                            item['z_m'] for item in taught
                        ])
                        entry[f'{kind}_z_m'] = float(np.mean(values))
                        entry[f'{kind}_z_std_m'] = float(np.std(values))
            fit = self.fits.get(floor)
            if fit is not None:
                entry['homography_marker_xy_to_command_xy'] = (
                    fit.matrix.tolist()
                )
                entry['homography_metrics'] = {
                    'sample_count': len(xy),
                    'inlier_count': fit.inlier_count,
                    'rmse_m': fit.rmse_m,
                    'max_error_m': fit.max_error_m,
                    'inlier_mask': fit.inlier_mask.astype(int).tolist(),
                    'all_residuals_m': fit.residuals_m.tolist(),
                }
            floors[floor] = entry
        return {
            'floor_calibration': {
                'format_version': 1,
                'updated_utc': datetime.now(timezone.utc).isoformat(),
                'frames': {
                    'base': self.base_frame,
                    'marker': self.marker_frame,
                    'command': self.command_frame,
                },
                'ransac_threshold_m': self.ransac_threshold,
                'floors': floors,
            }
        }

    def _save(self):
        self.output_file.parent.mkdir(parents=True, exist_ok=True)
        temporary = self.output_file.with_suffix(
            self.output_file.suffix + '.tmp'
        )
        with temporary.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(
                self._document(), stream, sort_keys=False,
                allow_unicode=True,
            )
        temporary.replace(self.output_file)


def main(args=None):
    """Run the floor calibration ROS node."""
    rclpy.init(args=args)
    node = None
    try:
        node = FloorCalibrator()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
