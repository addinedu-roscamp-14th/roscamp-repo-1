"""Keyboard TCP pose jog control with inverse-kinematics validation."""

import select
import sys
import termios
import time
import tty

import rclpy
from rclpy.node import Node

from ._config import BAUD, PORT, SPEED
from ._robot_utils import connect_robot


class ManualJogNode(Node):
    """Jog only the TCP pose while hiding joint-level control."""

    AXIS_NAMES = ('X', 'Y', 'Z', 'RX', 'RY', 'RZ')
    JOINT_LIMITS_DEG = (
        (-162.0, 162.0),
        (-135.0, 135.0),
        (-150.0, 150.0),
        (-162.0, 162.0),
        (-162.0, 162.0),
        (-162.0, 162.0),
    )

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
        self.translation_step = 1.0
        self.rotation_step = 1.0
        self.motion_fault = False
        self.robot = connect_robot(serial_port, baud_rate)
        self._prepare_control()

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

    def _read_angles(self):
        angles = self.robot.get_angles()
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            self.get_logger().error(f'Invalid joint angles: {angles}')
            return None
        return [float(value) for value in angles]

    def _read_coords(self):
        coords = self.robot.get_coords()
        if not isinstance(coords, (list, tuple)) or len(coords) < 6:
            self.get_logger().error(f'Invalid robot coordinates: {coords}')
            return None
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
                '1/2/3 step | E check | P pose | SPACE stop | Q quit'
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
