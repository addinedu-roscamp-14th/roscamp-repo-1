"""Teach the container grasp from the robot's current physical TCP pose."""

import argparse
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener
from std_srvs.srv import Trigger
import yaml


def quaternion_to_rpy_degrees(quaternion):
    """Convert an XYZW quaternion to roll, pitch and yaw in degrees."""
    x, y, z, w = [float(value) for value in quaternion]
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(value) for value in (roll, pitch, yaw)]


def mean_quaternion(quaternions):
    """Average same-hemisphere XYZW quaternions."""
    aligned = []
    reference = np.asarray(quaternions[0], dtype=float)
    for quaternion in quaternions:
        value = np.asarray(quaternion, dtype=float)
        if float(np.dot(value, reference)) < 0.0:
            value = -value
        aligned.append(value)
    result = np.mean(aligned, axis=0)
    return result / np.linalg.norm(result)


class GraspTeacher(Node):
    """Sample the marker and TCP transforms without commanding motion."""

    def __init__(self, base_frame, marker_frame, tcp_frame):
        super().__init__('teach_container_grasp')
        self.base_frame = base_frame
        self.marker_frame = marker_frame
        self.tcp_frame = tcp_frame
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.hand_start_client = self.create_client(
            Trigger, '/arm2/hand_guiding/start'
        )
        self.hand_finish_client = self.create_client(
            Trigger, '/arm2/hand_guiding/finish'
        )
        self.gripper_open_client = self.create_client(
            Trigger, '/arm2/gripper/open'
        )

    def open_gripper(self):
        """Open the gripper before physical grasp positioning."""
        if not self.gripper_open_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('gripper open service is unavailable')
        future = self.gripper_open_client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=45.0)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else 'no response'
            raise RuntimeError(f'failed to open gripper: {detail}')
        print(f'[GRIPPER] {response.message}')

    def set_hand_guiding(self, enabled):
        """Start or finish physical hand guiding through the serial owner."""
        client = self.hand_start_client if enabled else self.hand_finish_client
        name = 'start' if enabled else 'finish'
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'hand-guiding {name} service is unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response is not None else 'no response'
            raise RuntimeError(f'hand-guiding {name} failed: {detail}')
        print(f'[HAND GUIDING] {response.message}')

    def sample(self, count, timeout):
        """Return synchronized-enough transform samples from the live TF tree."""
        marker_samples = []
        tcp_samples = []
        deadline = time.monotonic() + timeout
        while len(marker_samples) < count and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                marker = self.buffer.lookup_transform(
                    self.base_frame, self.marker_frame, Time()
                )
                tcp = self.buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, Time()
                )
            except TransformException:
                continue
            marker_samples.append(marker.transform)
            tcp_samples.append(tcp.transform)
            time.sleep(0.05)
        if len(marker_samples) < count:
            raise RuntimeError(
                f'only received {len(marker_samples)}/{count} TF samples; '
                'keep the marker visible and the launch running'
            )
        return marker_samples, tcp_samples

    def sample_tcp(self, count, timeout):
        """Sample only the measured TCP after the marker has been cached."""
        samples = []
        deadline = time.monotonic() + timeout
        while len(samples) < count and time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.1)
            try:
                tcp = self.buffer.lookup_transform(
                    self.base_frame, self.tcp_frame, Time()
                )
            except TransformException:
                continue
            samples.append(tcp.transform)
            time.sleep(0.05)
        if len(samples) < count:
            raise RuntimeError(
                f'only received {len(samples)}/{count} TCP samples'
            )
        return samples


def transform_arrays(samples):
    """Return median translation and mean quaternion for transforms."""
    translations = np.asarray([
        [item.translation.x, item.translation.y, item.translation.z]
        for item in samples
    ])
    quaternions = np.asarray([
        [item.rotation.x, item.rotation.y, item.rotation.z, item.rotation.w]
        for item in samples
    ])
    return np.median(translations, axis=0), mean_quaternion(quaternions), translations


def load_pick_parameters(path):
    """Load geometry needed to separate marker, container and TCP offsets."""
    document = yaml.safe_load(Path(path).read_text(encoding='utf-8'))
    return document['/arm2/container_pick_coordinator']['ros__parameters']


def main(args=None):
    """Measure the manually aligned grasp and save a ROS parameter override."""
    parser = argparse.ArgumentParser(
        description=(
            'Save the current physical TCP pose as the exact container grasp. '
            'This command never moves the robot.'
        )
    )
    parser.add_argument('--base-frame', default='base_link')
    parser.add_argument('--marker-frame', default='arm2/container_marker')
    parser.add_argument('--tcp-frame', default='TCP')
    parser.add_argument('--samples', type=int, default=30)
    parser.add_argument('--timeout', type=float, default=15.0)
    parser.add_argument(
        '--hand-guided', action='store_true',
        help='release torque, wait for manual positioning, then save the pose',
    )
    parser.add_argument(
        '--tcp-above-final-mm', type=float, default=0.0,
        help=(
            'Teach from a visible safe pose this many millimeters directly '
            'above the desired final grasp'
        ),
    )
    parser.add_argument(
        '--base-config', default='config/arm2/arm2_container_pick.yaml'
    )
    parser.add_argument(
        '--output', default='config/arm2/arm2_container_grasp_teach.yaml'
    )
    parsed = parser.parse_args(args=args)

    parameters = load_pick_parameters(parsed.base_config)
    marker_correction = np.asarray(
        parameters['marker_translation_correction_xyz_m'], dtype=float
    )
    container_offset = np.asarray(
        parameters['container_offset_xyz_m'], dtype=float
    )
    extra_depth = float(parameters['grasp_extra_depth_m'])

    rclpy.init()
    node = GraspTeacher(
        parsed.base_frame, parsed.marker_frame, parsed.tcp_frame
    )
    hand_guiding_active = False
    try:
        marker_samples = None
        tcp_samples = None
        if parsed.hand_guided:
            print('\n팔을 반드시 손으로 받치고 주변 장애물을 치우세요.')
            print('먼저 ArUco를 카메라에 보이게 하고 컨테이너를 고정하세요.')
            node.open_gripper()
            print('토크 해제 전에 안정적인 마커 위치를 측정합니다...')
            marker_samples, _ = node.sample(parsed.samples, parsed.timeout)
            print('[HAND GUIDING] ArUco 기준 위치 저장 완료')
            input('준비됐으면 Enter를 누르세요. 서보 토크가 해제됩니다: ')
            # Mark this before the service call: even if its reply is lost or
            # times out, the hardware may already have released its servos.
            hand_guiding_active = True
            node.set_hand_guiding(True)
            print('\n양손으로 팔을 지지하면서 열린 그리퍼를 정확한 파지 위치에 놓으세요.')
            print('컨테이너 중심, 파지 높이, 그리퍼 방향을 모두 맞추세요.')
            input('정렬이 끝나면 팔을 그대로 지지한 채 Enter를 누르세요: ')
            node.set_hand_guiding(False)
            hand_guiding_active = False
            print('토크가 복구됐습니다. 팔에서 손을 천천히 떼고 측정을 기다리세요.')
            settle_deadline = time.monotonic() + 1.5
            while time.monotonic() < settle_deadline:
                rclpy.spin_once(node, timeout_sec=0.1)
            tcp_samples = node.sample_tcp(parsed.samples, parsed.timeout)
        else:
            marker_samples, tcp_samples = node.sample(
                parsed.samples, parsed.timeout
            )
        marker_xyz, marker_q, raw_marker_xyz = transform_arrays(marker_samples)
        tcp_xyz, tcp_q, raw_tcp_xyz = transform_arrays(tcp_samples)
        marker_std = np.max(np.std(raw_marker_xyz, axis=0))
        tcp_std = np.max(np.std(raw_tcp_xyz, axis=0))
        if marker_std > 0.003 or tcp_std > 0.003:
            raise RuntimeError(
                'pose was not stable enough to teach: '
                f'marker_std={marker_std * 1000.0:.2f}mm, '
                f'tcp_std={tcp_std * 1000.0:.2f}mm'
            )

        container_xyz = marker_xyz + marker_correction + container_offset
        taught_final_tcp_xyz = tcp_xyz.copy()
        taught_final_tcp_xyz[2] -= parsed.tcp_above_final_mm / 1000.0
        # Convert an optional safe pose above the grasp into the desired final
        # TCP. The saved nominal target then sits extra_depth above that pose.
        grasp_offset = taught_final_tcp_xyz - container_xyz
        grasp_offset[2] += extra_depth
        marker_rpy = quaternion_to_rpy_degrees(marker_q)
        tcp_rpy = quaternion_to_rpy_degrees(tcp_q)

        output = {
            '/arm2/container_pick_coordinator': {
                'ros__parameters': {
                    'grasp_offset_xyz_m': [
                        round(float(value), 6) for value in grasp_offset
                    ],
                    'grasp_offset_rpy_deg': [
                        round(float(value), 3) for value in tcp_rpy
                    ],
                    'reference_marker_yaw_deg': round(marker_rpy[2], 3),
                    'offsets_configured': True,
                }
            }
        }
        output_path = Path(parsed.output)
        output_path.parent.mkdir(parents=True, exist_ok=True)
        output_path.write_text(
            yaml.safe_dump(output, sort_keys=False), encoding='utf-8'
        )
        print(f'[TEACH COMPLETE] saved: {output_path.resolve()}')
        print(
            '[TEACH RESULT] marker_xyz_m='
            f'{np.round(marker_xyz, 5).tolist()}, '
            f'tcp_xyz_m={np.round(tcp_xyz, 5).tolist()}'
        )
        print(
            '[TEACH RESULT] grasp_offset_xyz_m='
            f'{output["/arm2/container_pick_coordinator"]["ros__parameters"]["grasp_offset_xyz_m"]}, '
            'grasp_offset_rpy_deg='
            f'{output["/arm2/container_pick_coordinator"]["ros__parameters"]["grasp_offset_rpy_deg"]}, '
            f'reference_marker_yaw_deg={marker_rpy[2]:.3f}'
        )
    finally:
        if hand_guiding_active:
            print('\n[SAFETY] 종료 전에 서보 토크를 자동 복구합니다.')
            try:
                node.set_hand_guiding(False)
            except Exception as exc:
                print(f'[CRITICAL] 서보 토크 자동 복구 실패: {exc}')
                print('팔을 계속 지지하고 전원을 안전하게 점검하세요.')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
