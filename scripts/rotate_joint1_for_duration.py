#!/usr/bin/env python3

import argparse
import json
import socket
import time


JOINT1_MIN_DEG = -165.0
JOINT1_MAX_DEG = 165.0


class ArmClient:
    def __init__(self, host, port):
        self.host = host
        self.port = port

    def request(self, payload):
        with socket.create_connection((self.host, self.port), timeout=3.0) as sock:
            sock.sendall((json.dumps(payload) + '\n').encode('utf-8'))
            response = b''
            while not response.endswith(b'\n'):
                chunk = sock.recv(4096)
                if not chunk:
                    break
                response += chunk

        if not response:
            raise RuntimeError('robot_arm_server returned no response')
        result = json.loads(response.decode('utf-8'))
        if not result.get('ok'):
            raise RuntimeError(result.get('error', f'command failed: {payload}'))
        return result

    def get_angles(self):
        result = self.request({'cmd': 'get_angles'})
        angles = result.get('angles')
        if not isinstance(angles, list) or len(angles) < 6:
            raise RuntimeError(f'invalid joint angles: {angles}')
        return [float(value) for value in angles[:6]]

    def send_joint1(self, angle, speed):
        current = self.get_angles()
        delta = float(angle) - current[0]
        # joint_step uses the robot's single-joint send_angle API.  Do not send
        # an angles array because that could command joints 2 through 6 too.
        self.request({
            'cmd': 'joint_step',
            'joint': 1,
            'delta': delta,
            'speed': int(speed),
            'wait': 0.0,
            'min_angle': JOINT1_MIN_DEG,
            'max_angle': JOINT1_MAX_DEG,
        })


def wait_for_joint1(client, target, timeout=15.0, tolerance=3.0):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        actual = client.get_angles()[0]
        if abs(actual - target) <= tolerance:
            return
        time.sleep(0.15)
    raise TimeoutError(f'joint 1 did not reach {target:.1f} degrees')


def parse_args():
    parser = argparse.ArgumentParser(
        description='Oscillate only joint 1 for a limited duration',
    )
    parser.add_argument('--left', type=float, default=JOINT1_MIN_DEG)
    parser.add_argument('--right', type=float, default=JOINT1_MAX_DEG)
    parser.add_argument('--duration', type=float, default=60.0)
    parser.add_argument('--speed', type=int, default=20)
    parser.add_argument('--host', default='127.0.0.1')
    parser.add_argument('--port', type=int, default=15000)
    return parser.parse_args()


def validate(args):
    if not JOINT1_MIN_DEG <= args.left < args.right <= JOINT1_MAX_DEG:
        raise ValueError(
            'joint 1 range must satisfy '
            f'{JOINT1_MIN_DEG} <= left < right <= {JOINT1_MAX_DEG}'
        )
    if not 1.0 <= args.duration <= 600.0:
        raise ValueError('duration must be between 1 and 600 seconds')
    if not 1 <= args.speed <= 50:
        raise ValueError('speed must be between 1 and 50')


def main():
    args = parse_args()
    validate(args)
    client = ArmClient(args.host, args.port)

    # Preserve the starting position. Only joint 1 will ever be commanded.
    starting_angles = client.get_angles()
    starting_joint1 = starting_angles[0]
    print(f'[CHECK] current angles: {starting_angles}')
    print('[CHECK] joints 2-6 will not be commanded')

    started = time.monotonic()
    target = args.left
    print(
        f'[START] joint 1: {args.left:.1f} <-> {args.right:.1f} deg, '
        f'{args.duration:.1f} sec, speed={args.speed}'
    )

    try:
        while time.monotonic() - started < args.duration:
            print(f'[MOVE] joint 1 -> {target:.1f} deg')
            client.send_joint1(target, args.speed)
            wait_for_joint1(client, target)
            target = args.right if target == args.left else args.left
    except KeyboardInterrupt:
        print('\n[STOP] interrupted by user')
    finally:
        print(f'[RETURN] joint 1 only -> starting angle {starting_joint1:.1f} deg')
        client.send_joint1(starting_joint1, args.speed)
        wait_for_joint1(client, starting_joint1)
        print('[DONE] joint 1 returned; joints 2-6 were not commanded')


if __name__ == '__main__':
    main()
