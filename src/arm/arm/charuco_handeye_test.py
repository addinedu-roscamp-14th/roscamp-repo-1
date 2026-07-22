"""Interactively move a torque-released arm and inspect one ChArUco TF."""

import math
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import TransformStamped
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from tf2_ros import Buffer, TransformException, TransformListener

from ._config import BAUD, PORT
from ._robot_utils import connect_robot


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


def quaternion_to_rpy_degrees(x, y, z, w):
    """Convert an XYZW quaternion to roll, pitch and yaw in degrees."""
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm < 1e-12:
        raise ValueError('Cannot convert a zero quaternion')
    x, y, z, w = (value / norm for value in (x, y, z, w))

    sin_roll = 2.0 * (w * x + y * z)
    cos_roll = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sin_roll, cos_roll)

    sin_pitch = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, sin_pitch)))

    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(sin_yaw, cos_yaw)
    return tuple(math.degrees(value) for value in (roll, pitch, yaw))


class CharucoHandeyeTest(Node):
    """Own the robot serial port, publish joints and inspect a board TF."""

    def __init__(self):
        super().__init__('charuco_handeye_test')
        self.declare_parameter('serial_port', PORT)
        self.declare_parameter('baud_rate', BAUD)
        self.declare_parameter('joint_states_topic', '/arm/joint_states')
        self.declare_parameter('joint_publish_rate', 10.0)
        self.declare_parameter('enable_speed', 10)
        self.declare_parameter('base_frame', 'arm/base_link')
        self.declare_parameter(
            'target_frame', 'arm/charuco_test_target'
        )

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        joint_publish_rate = float(
            self.get_parameter('joint_publish_rate').value
        )
        self.enable_speed = int(self.get_parameter('enable_speed').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        if joint_publish_rate <= 0.0:
            raise ValueError('joint_publish_rate must be positive')
        if not 1 <= self.enable_speed <= 100:
            raise ValueError('enable_speed must be within 1..100')

        self.robot = connect_robot(serial_port, baud_rate)
        self.torque_released = False
        self.serial_error_count = 0
        self.joint_state_publisher = self.create_publisher(
            JointState, joint_states_topic, 10
        )
        self.create_timer(
            1.0 / joint_publish_rate, self._publish_joint_states
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.get_logger().info(
            f'TF test target: {self.base_frame} -> {self.target_frame}'
        )
        self._report_servo_state()

    def _read_angles(self, log_error=True):
        try:
            angles = self.robot.get_angles()
        except Exception as exc:
            self.serial_error_count += 1
            if log_error or self.serial_error_count in (1, 10, 50):
                self.get_logger().error(
                    f'Failed to read joint angles: {exc}. Ensure this is the '
                    'only process using the robot serial port.'
                )
            return None
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            if log_error:
                self.get_logger().error(f'Invalid joint angles: {angles}')
            return None
        self.serial_error_count = 0
        return [float(value) for value in angles]

    def _publish_joint_states(self):
        if self.torque_released:
            return
        angles = self._read_angles(log_error=False)
        if angles is None:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [math.radians(value) for value in angles]
        message.velocity = [0.0] * 6
        message.effort = [0.0] * 6
        self.joint_state_publisher.publish(message)

    def _servo_state(self):
        try:
            state = self.robot.is_all_servo_enable()
        except Exception as exc:
            self.get_logger().error(f'Failed to read servo state: {exc}')
            return -1
        return int(state) if state in (0, 1) else -1

    def _report_servo_state(self):
        state = self._servo_state()
        label = {0: 'DISABLED', 1: 'ENABLED'}.get(state, 'UNKNOWN')
        self.torque_released = state == 0
        self.get_logger().info(f'Servo torque: {label}')

    def _disable_torque(self):
        if self.torque_released:
            self.get_logger().info('Robot power is already off')
            return
        self.get_logger().warning(
            'POWERING OFF ALL SERVOS. Support the arm continuously; it '
            'can fall under gravity. Press W to restore torque.'
        )
        try:
            self.robot.stop()
            self.robot.power_off()
            time.sleep(0.5)
            power_state = self.robot.is_power_on()
            if power_state != 0:
                raise RuntimeError(
                    f'power off was not verified: power_state={power_state}'
                )
            self.torque_released = True
            self.get_logger().warning(
                'Robot power OFF. Keep supporting the arm.'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to power off the robot: {exc}')

    def _enable_torque(self):
        self.get_logger().warning(
            'Enabling servo torque at the current pose. Keep supporting the '
            'arm until activation is confirmed.'
        )
        try:
            if self.robot.is_power_on() != 1:
                self.robot.power_on()
                time.sleep(0.5)
            angles = self._read_angles()
            if angles is None:
                raise RuntimeError(
                    'cannot read current joint angles after power on'
                )
            self.robot.focus_all_servos()
            time.sleep(0.5)
            state = self._servo_state()
            if state != 1:
                raise RuntimeError(
                    f'torque enable was not verified: servo_state={state}'
                )
            self.robot.send_angles(angles, self.enable_speed)
            self.torque_released = False
            self.get_logger().info(
                'Servo torque ENABLED at the current measured pose'
            )
            return True
        except Exception as exc:
            self.get_logger().error(f'Failed to enable servo torque: {exc}')
            return False

    def _lookup_board_transform(self):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.target_frame,
                Time(),
                timeout=Duration(seconds=1.0),
            )
        except TransformException as exc:
            self.get_logger().error(
                f'TF unavailable: {self.base_frame} -> '
                f'{self.target_frame}: {exc}'
            )
            return
        self._print_transform(transform)

    def _print_transform(self, transform: TransformStamped):
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        rpy = quaternion_to_rpy_degrees(
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        self.get_logger().info(
            f'{self.base_frame} -> {self.target_frame}\n'
            '  Translation [m]: '
            f'[{translation.x:.6f}, {translation.y:.6f}, '
            f'{translation.z:.6f}]\n'
            '  Translation [mm]: '
            f'[{translation.x * 1000.0:.2f}, '
            f'{translation.y * 1000.0:.2f}, '
            f'{translation.z * 1000.0:.2f}]\n'
            '  Quaternion [xyzw]: '
            f'[{rotation.x:.6f}, {rotation.y:.6f}, '
            f'{rotation.z:.6f}, {rotation.w:.6f}]\n'
            '  RPY [deg]: '
            f'[{rpy[0]:.3f}, {rpy[1]:.3f}, {rpy[2]:.3f}]'
        )

    def run_keyboard(self):
        """Read single-key commands until X, Escape or Ctrl-C is pressed."""
        if not sys.stdin.isatty():
            raise RuntimeError(
                'charuco_handeye_test must run in an interactive terminal'
            )
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            self.get_logger().info(
                'Q torque OFF | W torque ON | E print TF once | '
                'X/ESC quit'
            )
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.0)
                readable, _, _ = select.select([sys.stdin], [], [], 0.05)
                if not readable:
                    continue
                key = sys.stdin.read(1).lower()
                if key in ('x', '\x1b'):
                    break
                if key == 'q':
                    self._disable_torque()
                elif key == 'w':
                    self._enable_torque()
                elif key == 'e':
                    self._lookup_board_transform()
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    def restore_torque_for_shutdown(self):
        """Avoid leaving a gravity-loaded arm unpowered on normal exit."""
        if self.torque_released:
            self.get_logger().warning(
                'Node is exiting with torque disabled; attempting restoration'
            )
            self._enable_torque()


def main(args=None):
    """Run the interactive ChArUco Hand-Eye TF test node."""
    rclpy.init(args=args)
    node = None
    try:
        node = CharucoHandeyeTest()
        node.run_keyboard()
    except (KeyboardInterrupt, RuntimeError) as exc:
        if node is None:
            print(f'ChArUco Hand-Eye test startup failed: {exc}')
        else:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.restore_torque_for_shutdown()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
