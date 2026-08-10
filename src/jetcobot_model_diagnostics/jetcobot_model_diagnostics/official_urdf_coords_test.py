"""Capture and compare JetCobot controller coordinates with official URDF FK."""

import csv
from datetime import datetime
import json
import math
from pathlib import Path
import threading
import time

import numpy as np
from pymycobot.mycobot280 import MyCobot280
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .model_analysis import (
    AnalysisThresholds,
    controller_coords_matrix,
    flange_to_controller,
    mean_transform,
    rotation_angle_degrees,
    summarize_transforms,
    transform_components,
    transform_delta,
    transform_matrix,
)


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]
CSV_FIELDS = [
    'sample',
    'captured_at',
    *[f'j{index}_deg' for index in range(1, 7)],
    'coord_x_mm',
    'coord_y_mm',
    'coord_z_mm',
    'coord_rx_deg',
    'coord_ry_deg',
    'coord_rz_deg',
    'tf_x_m',
    'tf_y_m',
    'tf_z_m',
    'tf_qx',
    'tf_qy',
    'tf_qz',
    'tf_qw',
    'x_x_m',
    'x_y_m',
    'x_z_m',
    'x_qx',
    'x_qy',
    'x_qz',
    'x_qw',
]


def valid_six(values):
    """Return whether a controller response contains six finite numbers."""
    if not isinstance(values, (list, tuple)) or len(values) != 6:
        return False
    try:
        return bool(np.all(np.isfinite(np.asarray(values, dtype=float))))
    except (TypeError, ValueError):
        return False


def average_controller_poses(coord_samples):
    """Average controller positions and rotations without Euler wrap errors."""
    values = np.asarray(coord_samples, dtype=np.float64)
    result = np.zeros(6, dtype=np.float64)
    result[:3] = np.median(values[:, :3], axis=0)
    rotation = Rotation.from_euler(
        'xyz', values[:, 3:], degrees=True
    ).mean()
    result[3:] = rotation.as_euler('xyz', degrees=True)
    return result


class OfficialUrdfCoordsTest(Node):
    """Own the serial port and capture synchronized controller/URDF samples."""

    def __init__(self):
        super().__init__('official_urdf_coords_test')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter(
            'joint_states_topic', '/official_urdf_test/joint_states'
        )
        self.declare_parameter('base_frame', 'official/base_link')
        self.declare_parameter('flange_frame', 'official/6_Link')
        self.declare_parameter('read_count', 5)
        self.declare_parameter('read_interval_sec', 0.08)
        self.declare_parameter('tf_timeout_sec', 3.0)
        self.declare_parameter('max_joint_spread_deg', 0.5)
        self.declare_parameter('max_translation_spread_mm', 2.0)
        self.declare_parameter('max_rotation_spread_deg', 2.0)
        self.declare_parameter('station_a_j1_max_deg', -67.5)
        self.declare_parameter('station_b_j1_min_deg', -22.5)
        self.declare_parameter(
            'output_csv',
            '~/poter_ws/test_results/official_urdf_coords_samples.csv',
        )
        self.declare_parameter('consistent_translation_mm', 5.0)
        self.declare_parameter('consistent_rotation_deg', 3.0)
        self.declare_parameter('marginal_translation_mm', 10.0)
        self.declare_parameter('marginal_rotation_deg', 5.0)

        self.serial_port = str(
            self.get_parameter('serial_port').value
        )
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.flange_frame = str(
            self.get_parameter('flange_frame').value
        )
        self.read_count = int(self.get_parameter('read_count').value)
        self.read_interval = float(
            self.get_parameter('read_interval_sec').value
        )
        self.tf_timeout = float(
            self.get_parameter('tf_timeout_sec').value
        )
        self.max_joint_spread = float(
            self.get_parameter('max_joint_spread_deg').value
        )
        self.max_translation_spread = float(
            self.get_parameter('max_translation_spread_mm').value
        )
        self.max_rotation_spread = float(
            self.get_parameter('max_rotation_spread_deg').value
        )
        self.station_a_j1_max = float(
            self.get_parameter('station_a_j1_max_deg').value
        )
        self.station_b_j1_min = float(
            self.get_parameter('station_b_j1_min_deg').value
        )
        self.output_csv = Path(
            str(self.get_parameter('output_csv').value)
        ).expanduser().resolve()
        self.thresholds = AnalysisThresholds(
            consistent_translation_mm=float(
                self.get_parameter('consistent_translation_mm').value
            ),
            consistent_rotation_deg=float(
                self.get_parameter('consistent_rotation_deg').value
            ),
            marginal_translation_mm=float(
                self.get_parameter('marginal_translation_mm').value
            ),
            marginal_rotation_deg=float(
                self.get_parameter('marginal_rotation_deg').value
            ),
        )
        if self.read_count < 3:
            raise ValueError('read_count must be at least 3')
        if self.read_interval < 0.0 or self.tf_timeout <= 0.0:
            raise ValueError('read interval/TF timeout are invalid')
        if self.station_a_j1_max >= self.station_b_j1_min:
            raise ValueError('station J1 boundaries overlap')

        self.output_csv.parent.mkdir(parents=True, exist_ok=True)
        self.samples = []
        self.operation_lock = threading.Lock()
        self.service_group = ReentrantCallbackGroup()
        self.publisher = self.create_publisher(
            JointState, self.joint_states_topic, 10
        )
        self.status_publisher = self.create_publisher(
            String, '/official_urdf_test/status', 10
        )
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.create_service(
            Trigger,
            '/official_urdf_test/capture',
            self.capture,
            callback_group=self.service_group,
        )
        self.create_service(
            Trigger,
            '/official_urdf_test/analyze',
            self.analyze,
            callback_group=self.service_group,
        )
        self.create_service(
            Trigger,
            '/official_urdf_test/clear',
            self.clear,
            callback_group=self.service_group,
        )

        self._load_existing_samples()
        self.get_logger().info(
            f'Opening JetCobot read-only diagnostics on '
            f'{self.serial_port} @ {self.baud_rate}'
        )
        self.robot = MyCobot280(self.serial_port, self.baud_rate)
        time.sleep(1.0)
        self._publish_status(
            'READY: robot will not move; '
            f'loaded_samples={len(self.samples)}, output={self.output_csv}'
        )

    def _publish_status(self, text):
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        self.get_logger().info(text)

    def _load_existing_samples(self):
        if not self.output_csv.exists():
            return
        try:
            with self.output_csv.open(
                'r', encoding='utf-8', newline=''
            ) as stream:
                for row in csv.DictReader(stream):
                    matrix = transform_matrix(
                        [
                            float(row['x_x_m']),
                            float(row['x_y_m']),
                            float(row['x_z_m']),
                        ],
                        [
                            float(row['x_qx']),
                            float(row['x_qy']),
                            float(row['x_qz']),
                            float(row['x_qw']),
                        ],
                    )
                    angles = np.asarray([
                        float(row[f'j{index}_deg'])
                        for index in range(1, 7)
                    ])
                    self.samples.append({
                        'sample': int(row['sample']),
                        'angles': angles,
                        'x_matrix': matrix,
                    })
        except Exception as exc:
            raise RuntimeError(
                f'failed to load existing CSV {self.output_csv}: {exc}'
            ) from exc

    def _station_name(self, j1_degrees):
        if j1_degrees <= self.station_a_j1_max:
            return 'station_a'
        if j1_degrees >= self.station_b_j1_min:
            return 'station_b'
        return 'middle'

    def _read_stable_hardware(self):
        moving = self.robot.is_moving()
        if moving == 1:
            raise RuntimeError('robot is moving; wait until it stops')
        angle_samples = []
        coord_samples = []
        for _ in range(self.read_count):
            angles = self.robot.get_angles()
            coords = self.robot.get_coords()
            if not valid_six(angles):
                raise RuntimeError(f'invalid get_angles response: {angles}')
            if not valid_six(coords):
                raise RuntimeError(f'invalid get_coords response: {coords}')
            angle_samples.append([float(value) for value in angles])
            coord_samples.append([float(value) for value in coords])
            if self.read_interval:
                time.sleep(self.read_interval)
        angle_values = np.asarray(angle_samples)
        coord_values = np.asarray(coord_samples)
        joint_spread = float(np.max(np.ptp(angle_values, axis=0)))
        translation_spread = float(
            np.max(np.ptp(coord_values[:, :3], axis=0))
        )
        rotations = Rotation.from_euler(
            'xyz', coord_values[:, 3:], degrees=True
        )
        mean_rotation = rotations.mean()
        rotation_spread = max(
            float(np.degrees(
                (mean_rotation.inv() * value).magnitude()
            ))
            for value in rotations
        )
        if joint_spread > self.max_joint_spread:
            raise RuntimeError(
                f'joint readings unstable: spread={joint_spread:.3f}deg'
            )
        if translation_spread > self.max_translation_spread:
            raise RuntimeError(
                'controller XYZ unstable: '
                f'spread={translation_spread:.3f}mm'
            )
        if rotation_spread > self.max_rotation_spread:
            raise RuntimeError(
                'controller rotation unstable: '
                f'spread={rotation_spread:.3f}deg'
            )
        return (
            np.median(angle_values, axis=0),
            average_controller_poses(coord_values),
        )

    def _publish_joints_and_get_tf(self, angles):
        if self.publisher.get_subscription_count() == 0:
            raise RuntimeError(
                f'no robot_state_publisher subscribes to '
                f'{self.joint_states_topic}'
            )
        published_stamp_ns = 0
        for _ in range(5):
            message = JointState()
            now = self.get_clock().now()
            published_stamp_ns = now.nanoseconds
            message.header.stamp = now.to_msg()
            message.name = list(JOINT_NAMES)
            message.position = [
                math.radians(float(value)) for value in angles
            ]
            message.velocity = [0.0] * 6
            message.effort = [0.0] * 6
            self.publisher.publish(message)
            time.sleep(0.05)

        deadline = time.monotonic() + self.tf_timeout
        last_error = 'transform not received'
        while time.monotonic() < deadline:
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame,
                    self.flange_frame,
                    Time(),
                    timeout=Duration(seconds=0.1),
                )
                stamp_ns = (
                    transform.header.stamp.sec * 1_000_000_000
                    + transform.header.stamp.nanosec
                )
                if stamp_ns >= published_stamp_ns:
                    return transform
                last_error = (
                    f'stale TF stamp {stamp_ns} < {published_stamp_ns}'
                )
            except TransformException as exc:
                last_error = str(exc)
            time.sleep(0.05)
        raise RuntimeError(
            f'fresh {self.base_frame} -> {self.flange_frame} '
            f'unavailable: {last_error}'
        )

    @staticmethod
    def _tf_matrix(transform):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return transform_matrix(
            [translation.x, translation.y, translation.z],
            [rotation.x, rotation.y, rotation.z, rotation.w],
        )

    def _append_csv(
        self, sample_number, captured_at, angles, coords, urdf, x_matrix
    ):
        tf_components = transform_components(urdf)
        x_components = transform_components(x_matrix)
        row = {
            'sample': sample_number,
            'captured_at': captured_at,
        }
        row.update({
            f'j{index}_deg': float(value)
            for index, value in enumerate(angles, start=1)
        })
        for name, value in zip(
            (
                'coord_x_mm', 'coord_y_mm', 'coord_z_mm',
                'coord_rx_deg', 'coord_ry_deg', 'coord_rz_deg',
            ),
            coords,
        ):
            row[name] = float(value)
        for name, value in zip(
            ('tf_x_m', 'tf_y_m', 'tf_z_m'),
            tf_components['translation_m'],
        ):
            row[name] = float(value)
        for name, value in zip(
            ('tf_qx', 'tf_qy', 'tf_qz', 'tf_qw'),
            tf_components['quaternion_xyzw'],
        ):
            row[name] = float(value)
        for name, value in zip(
            ('x_x_m', 'x_y_m', 'x_z_m'),
            x_components['translation_m'],
        ):
            row[name] = float(value)
        for name, value in zip(
            ('x_qx', 'x_qy', 'x_qz', 'x_qw'),
            x_components['quaternion_xyzw'],
        ):
            row[name] = float(value)
        write_header = not self.output_csv.exists()
        with self.output_csv.open(
            'a', encoding='utf-8', newline=''
        ) as stream:
            writer = csv.DictWriter(stream, fieldnames=CSV_FIELDS)
            if write_header:
                writer.writeheader()
            writer.writerow(row)

    def capture(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'another diagnostic operation is active'
            return response
        try:
            angles, coords = self._read_stable_hardware()
            transform = self._publish_joints_and_get_tf(angles)
            urdf_matrix = self._tf_matrix(transform)
            controller_matrix = controller_coords_matrix(coords)
            x_matrix = flange_to_controller(
                urdf_matrix, controller_matrix
            )
            sample_number = (
                max(
                    (item['sample'] for item in self.samples),
                    default=0,
                )
                + 1
            )
            captured_at = datetime.now().astimezone().isoformat()
            self._append_csv(
                sample_number,
                captured_at,
                angles,
                coords,
                urdf_matrix,
                x_matrix,
            )
            self.samples.append({
                'sample': sample_number,
                'angles': angles,
                'x_matrix': x_matrix,
            })
            components = transform_components(x_matrix)
            station = self._station_name(float(angles[0]))
            text = (
                f'CAPTURED sample={sample_number}, station={station}, '
                f'j1={angles[0]:.2f}deg, '
                '6_Link->controller translation_mm='
                f'{np.round(np.asarray(components["translation_m"]) * 1000, 2).tolist()}, '
                'rpy_deg='
                f'{np.round(components["rpy_deg"], 2).tolist()}'
            )
            self._publish_status(text)
            response.success = True
            response.message = text
        except Exception as exc:
            response.success = False
            response.message = f'capture failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.operation_lock.release()
        return response

    def _station_summary(self, station_name):
        selected = [
            item['x_matrix']
            for item in self.samples
            if self._station_name(float(item['angles'][0]))
            == station_name
        ]
        if not selected:
            return None
        summary = summarize_transforms(selected, self.thresholds)
        return {
            'count': len(selected),
            'mean_matrix': summary['mean_matrix'],
            'translation_rms_mm': summary['translation_rms_mm'],
            'translation_max_mm': summary['translation_max_mm'],
            'rotation_rms_deg': summary['rotation_rms_deg'],
            'rotation_max_deg': summary['rotation_max_deg'],
        }

    def analyze(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'another diagnostic operation is active'
            return response
        try:
            if len(self.samples) < 3:
                raise RuntimeError(
                    f'at least 3 samples required; have {len(self.samples)}'
                )
            matrices = [item['x_matrix'] for item in self.samples]
            summary = summarize_transforms(matrices, self.thresholds)
            mean_components = transform_components(
                summary['mean_matrix']
            )
            station_results = {}
            for name in ('station_a', 'middle', 'station_b'):
                station = self._station_summary(name)
                if station is not None:
                    station_results[name] = station

            station_delta = None
            if (
                'station_a' in station_results
                and 'station_b' in station_results
            ):
                delta = transform_delta(
                    station_results['station_a']['mean_matrix'],
                    station_results['station_b']['mean_matrix'],
                )
                station_delta = {
                    'translation_mm': float(
                        np.linalg.norm(delta[:3, 3]) * 1000.0
                    ),
                    'rotation_deg': rotation_angle_degrees(delta),
                }

            report = {
                'generated_at': datetime.now().astimezone().isoformat(),
                'samples': len(self.samples),
                'classification': summary['classification'],
                'mean_6_link_to_controller': mean_components,
                'translation_rms_mm': summary['translation_rms_mm'],
                'translation_max_mm': summary['translation_max_mm'],
                'rotation_rms_deg': summary['rotation_rms_deg'],
                'rotation_max_deg': summary['rotation_max_deg'],
                'stations': {},
                'station_a_to_b_delta': station_delta,
            }
            for name, station in station_results.items():
                report['stations'][name] = {
                    key: value
                    for key, value in station.items()
                    if key != 'mean_matrix'
                }
                report['stations'][name]['mean'] = (
                    transform_components(station['mean_matrix'])
                )

            report_path = self.output_csv.with_name(
                self.output_csv.stem + '_analysis.json'
            )
            with report_path.open('w', encoding='utf-8') as stream:
                json.dump(report, stream, indent=2)
                stream.write('\n')

            station_text = ''
            if station_delta is not None:
                station_text = (
                    ', station_A-B='
                    f'{station_delta["translation_mm"]:.2f}mm/'
                    f'{station_delta["rotation_deg"]:.2f}deg'
                )
            text = (
                f'{summary["classification"]}: samples={len(self.samples)}, '
                f'translation rms/max='
                f'{summary["translation_rms_mm"]:.2f}/'
                f'{summary["translation_max_mm"]:.2f}mm, '
                f'rotation rms/max='
                f'{summary["rotation_rms_deg"]:.2f}/'
                f'{summary["rotation_max_deg"]:.2f}deg'
                f'{station_text}, report={report_path}'
            )
            self._publish_status(text)
            response.success = True
            response.message = text
        except Exception as exc:
            response.success = False
            response.message = f'analysis failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.operation_lock.release()
        return response

    def clear(self, _request, response):
        if not self.operation_lock.acquire(blocking=False):
            response.success = False
            response.message = 'another diagnostic operation is active'
            return response
        try:
            backup = None
            if self.output_csv.exists():
                suffix = datetime.now().strftime('%Y%m%d_%H%M%S')
                backup = self.output_csv.with_suffix(
                    self.output_csv.suffix + f'.bak_{suffix}'
                )
                self.output_csv.rename(backup)
            self.samples.clear()
            text = (
                'CLEARED in-memory samples'
                if backup is None
                else f'CLEARED; previous CSV backed up to {backup}'
            )
            self._publish_status(text)
            response.success = True
            response.message = text
        except Exception as exc:
            response.success = False
            response.message = f'clear failed: {exc}'
            self.get_logger().error(response.message)
        finally:
            self.operation_lock.release()
        return response


def main(args=None):
    """Run diagnostics with concurrent TF and service callback processing."""
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = OfficialUrdfCoordsTest()
        executor.add_node(node)
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
