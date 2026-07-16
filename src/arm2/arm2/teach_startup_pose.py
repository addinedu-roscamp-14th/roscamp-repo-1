"""Hand-guide and save one shared startup/return joint pose for arm2."""

import argparse
import math
from pathlib import Path
import time

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


class StartupPoseTeacher(Node):
    def __init__(self):
        super().__init__('teach_startup_pose')
        self.latest = None
        self.samples = []
        self.collecting = False
        self.create_subscription(JointState, '/joint_states', self.on_joints, 20)
        self.hand_start = self.create_client(
            Trigger, '/arm2/hand_guiding/start'
        )
        self.hand_finish = self.create_client(
            Trigger, '/arm2/hand_guiding/finish'
        )

    def on_joints(self, message):
        if not all(name in message.name for name in JOINT_NAMES):
            return
        order = [message.name.index(name) for name in JOINT_NAMES]
        values = [math.degrees(message.position[index]) for index in order]
        self.latest = values
        if self.collecting:
            self.samples.append(values)

    def set_hand_guiding(self, enabled):
        client = self.hand_start if enabled else self.hand_finish
        label = 'start' if enabled else 'finish'
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f'hand-guiding {label} service is unavailable')
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        response = future.result()
        if response is None or not response.success:
            detail = response.message if response else 'no response'
            raise RuntimeError(f'hand-guiding {label} failed: {detail}')
        print(f'[HAND GUIDING] {response.message}')

    def wait_for_services(self):
        """Fail before prompting if the serial-owning bridge is not running."""
        missing = []
        if not self.hand_start.wait_for_service(timeout_sec=3.0):
            missing.append('/arm2/hand_guiding/start')
        if not self.hand_finish.wait_for_service(timeout_sec=3.0):
            missing.append('/arm2/hand_guiding/finish')
        if missing:
            raise RuntimeError(
                '필수 서비스가 없습니다: '
                + ', '.join(missing)
                + '. 먼저 arm2_container_pick_moveit.launch.py를 실행하세요.'
            )

    def measure(self, count, timeout):
        self.samples = []
        self.collecting = True
        deadline = time.monotonic() + timeout
        try:
            while len(self.samples) < count and time.monotonic() < deadline:
                rclpy.spin_once(self, timeout_sec=0.1)
        finally:
            self.collecting = False
        if len(self.samples) < count:
            raise RuntimeError(
                f'only received {len(self.samples)}/{count} joint samples'
            )
        return np.median(np.asarray(self.samples), axis=0)


def main(args=None):
    parser = argparse.ArgumentParser(
        description='Hand-guide and save the shared arm2 startup pose.'
    )
    parser.add_argument(
        '--output', default='config/arm2/arm2_startup_pose.yaml'
    )
    parser.add_argument('--samples', type=int, default=20)
    parser.add_argument('--timeout', type=float, default=10.0)
    parsed = parser.parse_args(args=args)

    rclpy.init()
    node = StartupPoseTeacher()
    torque_released = False
    try:
        node.wait_for_services()
        print('\n팔을 양손으로 받치고 주변 장애물을 치우세요.')
        input('준비됐으면 Enter를 누르세요. 서보 토크가 해제됩니다: ')
        torque_released = True
        node.set_hand_guiding(True)
        print('\n팔을 원하는 새 초기 위치와 카메라 시야로 직접 맞추세요.')
        input('정렬이 끝나면 팔을 계속 받친 채 Enter를 누르세요: ')
        node.set_hand_guiding(False)
        torque_released = False
        print('토크가 복구됐습니다. 손을 천천히 떼고 측정을 기다리세요.')
        angles = node.measure(parsed.samples, parsed.timeout)
        rounded = [round(float(value), 3) for value in angles]
        document = {
            '/arm2/jetcobot_trajectory_bridge': {
                'ros__parameters': {'startup_angles_deg': list(rounded)}
            },
            '/arm2/container_pick_coordinator': {
                'ros__parameters': {'return_joint_angles_deg': list(rounded)}
            },
        }
        output = Path(parsed.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(
            yaml.safe_dump(document, sort_keys=False), encoding='utf-8'
        )
        print(f'[STARTUP TEACH COMPLETE] saved: {output.resolve()}')
        print(f'[STARTUP TEACH RESULT] joint_angles_deg={rounded}')
        print('다음 launch 재시작부터 시작 위치와 복귀 위치에 모두 적용됩니다.')
    finally:
        if torque_released:
            print('\n[SAFETY] 종료 전에 서보 토크를 자동 복구합니다.')
            try:
                node.set_hand_guiding(False)
            except Exception as exc:
                print(f'[CRITICAL] 서보 토크 복구 실패: {exc}')
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
