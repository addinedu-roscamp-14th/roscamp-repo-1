"""Teach the arm2 marker-to-TCP grasp offset from a stationary pose."""

from collections import deque
import math
import os
from pathlib import Path
import tempfile

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener
import yaml


def normalize_quaternion(values):
    """Return a normalized XYZW quaternion."""
    quaternion = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero')
    return quaternion / norm


def mean_quaternion(quaternions):
    """Average quaternions after resolving their equivalent signs."""
    reference = normalize_quaternion(quaternions[0])
    aligned = []
    for values in quaternions:
        quaternion = normalize_quaternion(values)
        if float(np.dot(reference, quaternion)) < 0.0:
            quaternion = -quaternion
        aligned.append(quaternion)
    return normalize_quaternion(np.mean(aligned, axis=0))


def quaternion_to_rpy_degrees(quaternion):
    """Convert an XYZW quaternion to roll, pitch, yaw in degrees."""
    x, y, z, w = normalize_quaternion(quaternion)
    roll = math.atan2(
        2.0 * (w * x + y * z),
        1.0 - 2.0 * (x * x + y * y),
    )
    pitch_value = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_value)))
    yaw = math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )
    return np.degrees([roll, pitch, yaw])


def calculate_taught_offset(samples):
    """Calculate base-frame grasp offset and taught orientations."""
    marker_positions = np.asarray(
        [sample['marker_position'] for sample in samples],
        dtype=np.float64,
    )
    tcp_positions = np.asarray(
        [sample['tcp_position'] for sample in samples],
        dtype=np.float64,
    )
    marker_rotation = mean_quaternion(
        [sample['marker_rotation'] for sample in samples]
    )
    tcp_rotation = mean_quaternion(
        [sample['tcp_rotation'] for sample in samples]
    )
    marker_position = np.mean(marker_positions, axis=0)
    tcp_position = np.mean(tcp_positions, axis=0)
    offset_samples = tcp_positions - marker_positions
    return {
        'grasp_offset_xyz_m': tcp_position - marker_position,
        'grasp_offset_rpy_deg': quaternion_to_rpy_degrees(tcp_rotation),
        'reference_marker_yaw_deg': float(
            quaternion_to_rpy_degrees(marker_rotation)[2]
        ),
        'marker_std_m': np.std(marker_positions, axis=0),
        'tcp_std_m': np.std(tcp_positions, axis=0),
        'offset_std_m': np.std(offset_samples, axis=0),
    }


class Arm2GraspOffsetCalibrator(Node):
    """Collect paired marker/TCP transforms and save the taught offset."""

    def __init__(self):
        """Initialize TF sampling and the offset capture service."""
        super().__init__('arm2_grasp_offset_calibrator')
        self.declare_parameter('base_frame', 'arm2/base_link')
        self.declare_parameter('marker_frame', 'arm2/container_marker')
        self.declare_parameter('tcp_frame', 'arm2/TCP')
        self.declare_parameter(
            'output_yaml', 'config/arm2/arm2_container_pick.yaml'
        )
        self.declare_parameter('sample_count', 20)
        self.declare_parameter('max_offset_std_m', 0.003)
        self.declare_parameter('max_tf_age_sec', 0.5)

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_frame = str(self.get_parameter('marker_frame').value)
        self.tcp_frame = str(self.get_parameter('tcp_frame').value)
        self.output_yaml = Path(
            str(self.get_parameter('output_yaml').value)
        ).expanduser().resolve()
        self.sample_count = max(
            5, int(self.get_parameter('sample_count').value)
        )
        self.max_offset_std = float(
            self.get_parameter('max_offset_std_m').value
        )
        self.max_tf_age = float(self.get_parameter('max_tf_age_sec').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.marker_samples = deque(maxlen=self.sample_count)
        self.tcp_samples = deque(maxlen=self.sample_count)
        self.latched_marker = None
        self.last_marker_error = 'waiting for marker TF'
        self.last_tcp_error = 'waiting for TCP TF'
        self.create_timer(0.05, self.collect_sample)
        self.create_service(
            Trigger,
            '/arm2/capture_grasp_marker',
            self.capture_marker,
        )
        self.create_service(
            Trigger,
            '/arm2/capture_grasp_offset',
            self.capture_offset,
        )
        self.get_logger().info(
            'Hold marker still and call /arm2/capture_grasp_marker; then '
            'teach the exact TCP pose and call /arm2/capture_grasp_offset'
        )

    @staticmethod
    def transform_values(transform):
        """Extract translation and XYZW rotation arrays from a transform."""
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        return (
            np.array([translation.x, translation.y, translation.z]),
            np.array([rotation.x, rotation.y, rotation.z, rotation.w]),
        )

    def collect_sample(self):
        """Collect fresh marker and TCP samples independently."""
        try:
            marker = self.tf_buffer.lookup_transform(
                self.base_frame, self.marker_frame, Time()
            )
        except TransformException as exc:
            self.last_marker_error = str(exc)
            self.marker_samples.clear()
        else:
            marker_age = (
                self.get_clock().now() - Time.from_msg(marker.header.stamp)
            ).nanoseconds / 1e9
            if marker_age > self.max_tf_age:
                self.last_marker_error = f'stale marker TF: {marker_age:.3f}s'
                self.marker_samples.clear()
            else:
                marker_position, marker_rotation = self.transform_values(marker)
                self.marker_samples.append({
                    'position': marker_position,
                    'rotation': marker_rotation,
                })

        try:
            tcp = self.tf_buffer.lookup_transform(
                self.base_frame, self.tcp_frame, Time()
            )
        except TransformException as exc:
            self.last_tcp_error = str(exc)
            self.tcp_samples.clear()
        else:
            tcp_age = (
                self.get_clock().now() - Time.from_msg(tcp.header.stamp)
            ).nanoseconds / 1e9
            if tcp_age > self.max_tf_age:
                self.last_tcp_error = f'stale TCP TF: {tcp_age:.3f}s'
                self.tcp_samples.clear()
            else:
                tcp_position, tcp_rotation = self.transform_values(tcp)
                self.tcp_samples.append({
                    'position': tcp_position,
                    'rotation': tcp_rotation,
                })

    def capture_marker(self, _request, response):
        """Latch a stable container marker pose before teaching the TCP."""
        if len(self.marker_samples) < self.sample_count:
            response.success = False
            response.message = (
                f'need {self.sample_count - len(self.marker_samples)} more '
                f'marker samples; last error: {self.last_marker_error}'
            )
            return response
        positions = np.asarray([
            sample['position'] for sample in self.marker_samples
        ])
        max_std = float(np.max(np.std(positions, axis=0)))
        if max_std > self.max_offset_std:
            response.success = False
            response.message = (
                f'marker is not stable: std={max_std:.4f}m exceeds '
                f'{self.max_offset_std:.4f}m'
            )
            return response
        self.latched_marker = {
            'position': np.mean(positions, axis=0),
            'rotation': mean_quaternion([
                sample['rotation'] for sample in self.marker_samples
            ]),
        }
        self.tcp_samples.clear()
        position = np.round(self.latched_marker['position'], 6).tolist()
        response.success = True
        response.message = (
            f'marker locked at {position}; keep container fixed and teach TCP'
        )
        self.get_logger().info(response.message)
        return response

    def capture_offset(self, _request, response):
        """Validate samples and save a taught grasp configuration."""
        if self.latched_marker is None:
            response.success = False
            response.message = 'capture /arm2/capture_grasp_marker first'
            return response
        if len(self.tcp_samples) < self.sample_count:
            response.success = False
            response.message = (
                f'need {self.sample_count - len(self.tcp_samples)} more TCP '
                f'samples; last error: {self.last_tcp_error}'
            )
            return response

        samples = [{
            'marker_position': self.latched_marker['position'],
            'marker_rotation': self.latched_marker['rotation'],
            'tcp_position': sample['position'],
            'tcp_rotation': sample['rotation'],
        } for sample in self.tcp_samples]
        result = calculate_taught_offset(samples)
        max_std = float(np.max(result['offset_std_m']))
        if max_std > self.max_offset_std:
            response.success = False
            response.message = (
                f'pose is not stable: offset std={max_std:.4f}m exceeds '
                f'{self.max_offset_std:.4f}m'
            )
            return response

        try:
            self.save_result(result)
        except (OSError, ValueError, yaml.YAMLError) as exc:
            response.success = False
            response.message = f'failed to save offset: {exc}'
            return response

        xyz = np.round(result['grasp_offset_xyz_m'], 6).tolist()
        rpy = np.round(result['grasp_offset_rpy_deg'], 3).tolist()
        yaw = result['reference_marker_yaw_deg']
        response.success = True
        response.message = (
            f'saved offset_xyz_m={xyz}, grasp_rpy_deg={rpy}, '
            f'reference_marker_yaw_deg={yaw:.3f}; full pick remains locked'
        )
        self.get_logger().info(response.message)
        return response

    def save_result(self, result):
        """Atomically update the arm2 ROS parameter YAML."""
        if not self.output_yaml.is_file():
            raise ValueError(
                f'configuration file not found: {self.output_yaml}'
            )
        with self.output_yaml.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
        node_key = '/arm2/arm2_container_pick_coordinator'
        try:
            parameters = document[node_key]['ros__parameters']
        except (KeyError, TypeError) as exc:
            raise ValueError(
                f'configuration does not contain {node_key}/ros__parameters'
            ) from exc

        parameters['grasp_offset_xyz_m'] = [
            round(float(value), 6)
            for value in result['grasp_offset_xyz_m']
        ]
        parameters['grasp_offset_rpy_deg'] = [
            round(float(value), 3)
            for value in result['grasp_offset_rpy_deg']
        ]
        parameters['reference_marker_yaw_deg'] = round(
            float(result['reference_marker_yaw_deg']), 3
        )
        parameters['offsets_configured'] = True
        parameters['allow_full_pick'] = False

        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = None
        try:
            with tempfile.NamedTemporaryFile(
                mode='w',
                encoding='utf-8',
                dir=self.output_yaml.parent,
                prefix=f'.{self.output_yaml.name}.',
                suffix='.tmp',
                delete=False,
            ) as stream:
                temporary_path = Path(stream.name)
                yaml.safe_dump(document, stream, sort_keys=False)
            os.replace(temporary_path, self.output_yaml)
        finally:
            if temporary_path is not None and temporary_path.exists():
                temporary_path.unlink()


def main(args=None):
    """Run the grasp-offset teaching node."""
    rclpy.init(args=args)
    node = Arm2GraspOffsetCalibrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
