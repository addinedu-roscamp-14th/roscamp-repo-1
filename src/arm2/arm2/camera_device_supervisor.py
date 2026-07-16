"""Restart a camera process when its stable device path disappears/reappears."""

import argparse
import os
import select
import signal
import subprocess
import sys
import time


def device_is_ready(path):
    """Return true only while the video node and its kernel device both exist."""
    if not os.path.exists(path):
        return False
    resolved = os.path.realpath(path)
    name = os.path.basename(resolved)
    if name.startswith('video'):
        return os.path.exists(f'/sys/class/video4linux/{name}/device')
    return True


def stop_child(child):
    """Stop the current camera process without leaving it orphaned."""
    if child is None or child.poll() is not None:
        return
    child.send_signal(signal.SIGINT)
    try:
        child.wait(timeout=3.0)
    except subprocess.TimeoutExpired:
        child.terminate()
        try:
            child.wait(timeout=2.0)
        except subprocess.TimeoutExpired:
            child.kill()
            child.wait()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--device', required=True)
    parser.add_argument('--poll-sec', type=float, default=0.1)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args()
    command = args.command
    if command and command[0] == '--':
        command = command[1:]
    if not command:
        parser.error('camera command is required after --')

    stopping = False
    child = None
    restart_not_before = 0.0

    def request_stop(_signum, _frame):
        nonlocal stopping
        stopping = True

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    print(
        f'[camera_supervisor] watching {args.device}',
        flush=True,
    )
    while not stopping:
        device_ready = device_is_ready(args.device)
        if not device_ready:
            if child is not None and child.poll() is None:
                print(
                    f'[camera_supervisor] {args.device} disappeared; '
                    'stopping camera',
                    flush=True,
                )
                stop_child(child)
            child = None
            time.sleep(args.poll_sec)
            continue

        if child is not None and child.poll() is None and child.stdout:
            readable, _, _ = select.select([child.stdout], [], [], 0.0)
            if readable:
                line = child.stdout.readline()
                if line:
                    # Preserve the camera's normal ROS logs in the launch
                    # terminal even though they pass through this watchdog.
                    print(line, end='', flush=True)
                    if 'No such device (19)' in line:
                        print(
                            '[camera_supervisor] camera returned ENODEV; '
                            'restarting v4l2 process',
                            flush=True,
                        )
                        stop_child(child)
                        child = None
                        restart_not_before = time.monotonic() + 0.5
                        continue

        if (
            (child is None or child.poll() is not None)
            and time.monotonic() >= restart_not_before
        ):
            if child is not None:
                print(
                    f'[camera_supervisor] camera exited with '
                    f'code {child.returncode}; restarting',
                    flush=True,
                )
            else:
                print(
                    f'[camera_supervisor] {args.device} available; '
                    'starting camera',
                    flush=True,
                )
            environment = os.environ.copy()
            environment['RCUTILS_LOGGING_BUFFERED_STREAM'] = '0'
            child = subprocess.Popen(
                command,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                text=True,
                bufsize=1,
                env=environment,
            )
        time.sleep(args.poll_sec)

    stop_child(child)
    return 0


if __name__ == '__main__':
    sys.exit(main())
