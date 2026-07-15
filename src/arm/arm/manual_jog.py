"""Keyboard TCP pose jog control with inverse-kinematics validation."""

import math
import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ._config import BAUD, PORT, SPEED
from ._joint_limits import JOINT_LIMITS_DEG as JETCOBOT_JOINT_LIMITS_DEG
from ._robot_utils import connect_robot


class ManualJogNode(Node):
    """Jog only the TCP pose while hiding joint-level control."""

    AXIS_NAMES = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')
    JOINT_LIMITS_DEG = JETCOBOT_JOINT_LIMITS_DEG

    def __init__(self):
        """Connect to the robot and prepare validated TCP jogging."""
        super().__init__('manual_jog')

        self.declare_parameter('serial_port', PORT)
        self.declare_parameter('baud_rate', BAUD)
        self.declare_parameter('speed', min(SPEED, 20))
        self.declare_parameter('command_wait_seconds', 1.0)
        self.declare_parameter('max_joint_delta_deg', 15.0)
        self.declare_parameter('min_z_mm', 20.0)
        self.declare_parameter('max_z_mm', 300.0)
        self.declare_parameter('joint_states_topic', '/arm/joint_states')
        self.declare_parameter('joint_publish_rate', 10.0)
        self.declare_parameter('torque_release_seconds', 5.0)

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.speed = int(self.get_parameter('speed').value)
        self.command_wait = float(
            self.get_parameter('command_wait_seconds').value
        )
        self.max_joint_delta = float(
            self.get_parameter('max_joint_delta_deg').value
        )
        self.min_z = float(self.get_parameter('min_z_mm').value)
        self.max_z = float(self.get_parameter('max_z_mm').value)
        joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        joint_publish_rate = float(
            self.get_parameter('joint_publish_rate').value
        )
        self.torque_release_seconds = float(
            self.get_parameter('torque_release_seconds').value
        )
        if joint_publish_rate <= 0.0:
            raise ValueError('joint_publish_rate must be greater than zero')
        if not 1.0 <= self.torque_release_seconds <= 15.0:
            raise ValueError(
                'torque_release_seconds must be between 1 and 15 seconds'
            )
        self.translation_step = 1.0
        self.rotation_step = 1.0
        self.motion_fault = False
        self.serial_read_failures = 0
        self.robot = connect_robot(serial_port, baud_rate)
        self.joint_state_publisher = self.create_publisher(
            JointState, joint_states_topic, 10
        )
        self.create_timer(
            1.0 / joint_publish_rate, self._publish_joint_states
        )
        self._prepare_control()

    def _publish_joint_states(self):
        angles = self._read_angles(log_error=False)
        if angles is None:
            return

        self._publish_joint_angles(angles)

    def _publish_joint_angles(self, angles):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [
            '1_Joint',
            '2_Joint',
            '3_Joint',
            '4_Joint',
            '5_Joint',
            '6_Joint',
        ]
        message.position = [math.radians(value) for value in angles]
        message.velocity = [0.0] * 6
        message.effort = [0.0] * 6
        self.joint_state_publisher.publish(message)

    def _prepare_control(self):
        self.get_logger().warning(
            'Preparing servo torque. Keep clear of the robot and support it.'
        )
        self.robot.set_fresh_mode(1)
        time.sleep(0.3)

        power_state = self.robot.is_power_on()
        if power_state != 1:
            self.robot.power_on()
            time.sleep(0.5)

        if self.robot.is_free_mode() == 1:
            self.robot.set_free_mode(0)
            time.sleep(0.3)

        self.robot.focus_all_servos()
        time.sleep(0.5)
        power_state = self.robot.is_power_on()
        servo_state = self.robot.is_all_servo_enable()
        free_mode = self.robot.is_free_mode()
        self.get_logger().info(
            f'Robot state: power={power_state}, servos={servo_state}, '
            f'free_mode={free_mode}'
        )
        if power_state != 1 or servo_state != 1 or free_mode == 1:
            self.motion_fault = True
            raise RuntimeError(
                'Robot is not ready: power and all servos must be enabled, '
                'and free mode must be disabled'
            )
        self.motion_fault = False

    def _read_angles(self, log_error=True):
        try:
            angles = self.robot.get_angles()
        except Exception as exc:
            self._handle_serial_read_error('joint angles', exc, log_error)
            return None
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            if log_error:
                self.get_logger().error(f'Invalid joint angles: {angles}')
            return None
        self.serial_read_failures = 0
        return [float(value) for value in angles]

    def _handle_serial_read_error(self, operation, exception, log_error=True):
        self.serial_read_failures += 1
        self.motion_fault = True
        if log_error or self.serial_read_failures in (1, 10, 50):
            self.get_logger().error(
                f'Failed to read {operation} from the robot: {exception}. '
                'Check USB connection and ensure only one process owns the '
                'serial port.'
            )

    def _read_servo_state(self, attempts=5):
        for _ in range(attempts):
            try:
                state = self.robot.is_all_servo_enable()
            except Exception as exc:
                self._handle_serial_read_error('servo state', exc)
                state = -1
            if state in (0, 1):
                return int(state)
            time.sleep(0.2)
        return -1

    def _read_coords(self):
        try:
            coords = self.robot.get_coords()
        except Exception as exc:
            self._handle_serial_read_error('TCP coordinates', exc)
            return None
        if not isinstance(coords, (list, tuple)) or len(coords) < 6:
            self.get_logger().error(f'Invalid robot coordinates: {coords}')
            return None
        self.serial_read_failures = 0
        return [float(value) for value in coords[:6]]

    def _print_pose(self):
        coords = self._read_coords()
        angles = self._read_angles()
        if coords is not None:
            self.get_logger().info(
                'TCP [x, y, z, rx, ry, rz] = '
                f'{[round(value, 2) for value in coords]}'
            )
        if angles is not None:
            self.get_logger().info(
                f'IK joints [deg] = {[round(value, 2) for value in angles]}'
            )

    def _release_torque_temporarily(self):
        self.get_logger().warning(
            'Releasing all servo torque. Support the arm continuously; '
            'it can fall under gravity.'
        )
        self.robot.stop()
        restore_required = False
        try:
            restore_required = True
            result = self.robot.release_all_servos()
            if result == -1:
                self.get_logger().warning(
                    'release_all_servos returned no ACK; checking actual '
                    'servo state'
                )
            time.sleep(0.3)
            servo_state = self._read_servo_state()
            if servo_state != 0:
                raise RuntimeError(
                    'servo torque release was not verified: '
                    f'is_all_servo_enable={servo_state}'
                )
            deadline = time.monotonic() + self.torque_release_seconds
            last_countdown = None
            while True:
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    break
                countdown = max(1, math.ceil(remaining))
                if countdown != last_countdown:
                    self.get_logger().warning(
                        f'Torque restores in {countdown} s'
                    )
                    last_countdown = countdown
                angles = self._read_angles(log_error=False)
                if angles is not None:
                    self._publish_joint_angles(angles)
                time.sleep(min(0.1, remaining))
        except Exception as exc:
            self.motion_fault = True
            self.get_logger().error(f'Failed to release torque: {exc}')
        finally:
            if restore_required:
                current_angles = self._read_angles(log_error=False)
                try:
                    result = self.robot.focus_all_servos()
                    if result == -1:
                        self.get_logger().warning(
                            'focus_all_servos returned no ACK; checking actual '
                            'servo state'
                        )
                    time.sleep(0.5)
                    servo_state = self._read_servo_state()
                    if servo_state != 1:
                        raise RuntimeError(
                            'servo torque restoration was not verified: '
                            f'is_all_servo_enable={servo_state}'
                        )
                    if current_angles is not None:
                        self.robot.send_angles(current_angles, self.speed)
                        self._publish_joint_angles(current_angles)
                    self.motion_fault = False
                    self.get_logger().info(
                        'All servo torque restored at the current pose. '
                        'Wait for the arm to settle before taking a sample.'
                    )
                    self._print_pose()
                except Exception as exc:
                    self.motion_fault = True
                    self.get_logger().error(
                        f'CRITICAL: failed to restore servo torque: {exc}'
                    )

    @staticmethod
    def _normalize_angle(angle):
        return (angle + 180.0) % 360.0 - 180.0

    def _solve_ik(self, target_coords, current_angles):
        try:
            solution = self.robot.solve_inv_kinematics(
                target_coords, current_angles
            )
        except AttributeError as exc:
            raise RuntimeError(
                'Installed pymycobot does not provide solve_inv_kinematics'
            ) from exc

        if not isinstance(solution, (list, tuple)) or len(solution) != 6:
            self.get_logger().warning(
                f'IK rejected target {target_coords}: solution={solution}'
            )
            return None

        solution = [float(value) for value in solution]
        for index, (angle, limits) in enumerate(
            zip(solution, self.JOINT_LIMITS_DEG)
        ):
            lower, upper = limits
            if not lower <= angle <= upper:
                self.get_logger().warning(
                    f'IK solution rejected: J{index + 1}={angle:.2f} '
                    f'is outside [{lower:.2f}, {upper:.2f}]'
                )
                return None

        deltas = [
            abs(target - current)
            for target, current in zip(solution, current_angles)
        ]
        if max(deltas) > self.max_joint_delta:
            self.get_logger().warning(
                f'IK branch jump rejected: joint deltas={deltas}, '
                f'limit={self.max_joint_delta:g} deg'
            )
            return None
        return solution

    def _move_tcp_axis(self, index, delta):
        if self.motion_fault:
            self.get_logger().error(
                'Jog is locked after a motion fault. Press E to recheck '
                'the servo state.'
            )
            return

        current_coords = self._read_coords()
        current_angles = self._read_angles()
        if current_coords is None or current_angles is None:
            return

        target_coords = current_coords.copy()
        target_coords[index] += delta
        if index == 2 and not self.min_z <= target_coords[2] <= self.max_z:
            self.get_logger().warning(
                f'Z target {target_coords[2]:.2f} is outside '
                f'[{self.min_z:.2f}, {self.max_z:.2f}]'
            )
            return
        if index >= 3:
            target_coords[index] = self._normalize_angle(
                target_coords[index]
            )

        solution = self._solve_ik(target_coords, current_angles)
        if solution is None:
            return

        self.get_logger().info(
            f'TCP {self.AXIS_NAMES[index]}: {current_coords[index]:.2f} '
            f'-> {target_coords[index]:.2f}; '
            f'IK={[round(value, 2) for value in solution]}'
        )
        self.robot.send_angles(solution, self.speed)
        time.sleep(self.command_wait)

        measured = self._read_coords()
        if measured is None:
            return
        actual_delta = measured[index] - current_coords[index]
        self.get_logger().info(
            f'Measured TCP: {[round(value, 2) for value in measured]}'
        )
        minimum_motion = 0.2
        if abs(actual_delta) < minimum_motion:
            self.robot.stop()
            self.motion_fault = True
            self.get_logger().error(
                'TCP did not move on the commanded axis. Jog locked.'
            )

    def _handle_key(self, key):
        translation_keys = {
            'w': (0, self.translation_step),
            's': (0, -self.translation_step),
            'a': (1, self.translation_step),
            'd': (1, -self.translation_step),
            'r': (2, self.translation_step),
            'f': (2, -self.translation_step),
        }
        rotation_keys = {
            'u': (3, self.rotation_step),
            'o': (3, -self.rotation_step),
            'i': (4, self.rotation_step),
            'k': (4, -self.rotation_step),
            'j': (5, self.rotation_step),
            'l': (5, -self.rotation_step),
        }
        if key in translation_keys:
            self._move_tcp_axis(*translation_keys[key])
        elif key in rotation_keys:
            self._move_tcp_axis(*rotation_keys[key])
        elif key in ('1', '2', '3'):
            step = {'1': 1.0, '2': 3.0, '3': 5.0}[key]
            self.translation_step = step
            self.rotation_step = step
            self.get_logger().info(f'TCP step: {step:g} mm / deg')
        elif key == 'p':
            self._print_pose()
        elif key == 't':
            self._release_torque_temporarily()
        elif key == 'e':
            try:
                self._prepare_control()
                self.get_logger().info('Jog fault cleared')
            except Exception as exc:
                self.motion_fault = True
                self.get_logger().error(
                    f'Failed to prepare robot control: {exc}'
                )
        elif key == ' ':
            self.robot.stop()
            self.motion_fault = True
            self.get_logger().warning(
                'Stop command sent. Press E before any further jog.'
            )

    def run_keyboard(self):
        """Read one-key commands until Q or ESC is pressed."""
        if not sys.stdin.isatty():
            raise RuntimeError(
                'manual_jog must run in an interactive terminal'
            )

        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self.get_logger().info(
                'W/S X | A/D Y | R/F Z | U/O RX | I/K RY | J/L RZ | '
                '1/2/3 step | T torque off 5s | E check | P pose | '
                'SPACE stop | Q quit'
            )
            self._print_pose()
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not readable:
                    continue
                key = sys.stdin.read(1)
                if key in ('q', 'Q', '\x1b'):
                    break
                self._handle_key(key.lower())
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    """Run the interactive IK-validated TCP jog node."""
    rclpy.init(args=args)
    node = None
    try:
        node = ManualJogNode()
        node.run_keyboard()
    except (KeyboardInterrupt, RuntimeError) as exc:
        if node is None:
            print(f'Manual jog startup failed: {exc}')
        else:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
