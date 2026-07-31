"""Execute FollowJointTrajectory goals on a JetCobot through pymycobot."""

import math
import threading
import time

from control_msgs.action import FollowJointTrajectory
from pymycobot.mycobot280 import MyCobot280
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectoryPoint

from .arm2_joint_limits import JOINT_LIMITS_DEG


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


def clamp_measured_joints_for_planning(angles, tolerance_degrees):
    """Clamp only small measured limit overshoots for MoveIt state input."""
    tolerance = float(tolerance_degrees)
    if tolerance < 0.0:
        raise ValueError('joint-state clamp tolerance must be non-negative')
    clamped = []
    for angle, (lower, upper) in zip(angles, JOINT_LIMITS_DEG):
        value = float(angle)
        if lower - tolerance <= value < lower:
            value = lower
        elif upper < value <= upper + tolerance:
            value = upper
        clamped.append(value)
    return clamped


def duration_seconds(duration):
    """Convert a ROS duration message to floating-point seconds."""
    return float(duration.sec) + float(duration.nanosec) / 1e9


def interpolate_positions(points, elapsed):
    """Interpolate a trajectory at elapsed seconds."""
    if elapsed <= duration_seconds(points[0].time_from_start):
        return list(points[0].positions)
    for previous, following in zip(points, points[1:]):
        start = duration_seconds(previous.time_from_start)
        end = duration_seconds(following.time_from_start)
        if elapsed <= end:
            if end <= start:
                return list(following.positions)
            ratio = (elapsed - start) / (end - start)
            return [
                float(a) + ratio * (float(b) - float(a))
                for a, b in zip(previous.positions, following.positions)
            ]
    return list(points[-1].positions)


def joint_errors_degrees(actual, target):
    """Return signed target-minus-actual joint errors in degrees."""
    return [float(goal) - float(measured) for measured, goal in zip(
        actual, target
    )]


def adaptive_joint_command_degrees(
    target,
    actual,
    current_command,
    joint_indices,
    gain,
    max_total_correction,
):
    """Apply bounded measured-error feedback to selected command joints."""
    command = [float(value) for value in current_command]
    for index in joint_indices:
        error = float(target[index]) - float(actual[index])
        proposed = command[index] + gain * error
        lower = float(target[index]) - max_total_correction
        upper = float(target[index]) + max_total_correction
        command[index] = max(lower, min(upper, proposed))
    return command


def cumulative_joint_travel_degrees(start_degrees, points, joint_index):
    """Return cumulative absolute travel for one trajectory joint."""
    previous = float(start_degrees)
    travel = 0.0
    for point in points:
        current = math.degrees(float(point.positions[joint_index]))
        travel += abs(current - previous)
        previous = current
    return travel


def validate_home_angles(angles):
    """Return a validated six-joint home target in degrees."""
    if len(angles) != len(JOINT_NAMES):
        raise ValueError('home_joint_angles_deg must contain six values')
    target = [float(value) for value in angles]
    for index, (value, limits) in enumerate(zip(target, JOINT_LIMITS_DEG)):
        if not math.isfinite(value) or not limits[0] <= value <= limits[1]:
            raise ValueError(
                f'home J{index + 1}={value:g}deg outside '
                f'[{limits[0]:g}, {limits[1]:g}]'
            )
    return target


class JetCobotTrajectoryBridge(Node):
    """Own the robot serial port and expose a trajectory action controller."""

    def __init__(self):
        super().__init__('arm2_jetcobot_trajectory_bridge')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 10)
        self.declare_parameter('command_rate_hz', 10.0)
        self.declare_parameter('joint_state_rate_hz', 10.0)
        self.declare_parameter('max_start_error_deg', 15.0)
        self.declare_parameter('max_j6_trajectory_travel_deg', 150.0)
        self.declare_parameter('goal_tolerance_deg', 2.5)
        self.declare_parameter('goal_timeout_sec', 15.0)
        self.declare_parameter('goal_correction_speed', 50)
        self.declare_parameter('goal_correction_period_sec', 1.0)
        self.declare_parameter('adaptive_goal_correction_enabled', True)
        self.declare_parameter(
            'adaptive_goal_correction_joints', ['4_Joint']
        )
        self.declare_parameter(
            'adaptive_goal_correction_tolerance_deg', 0.5
        )
        self.declare_parameter('adaptive_goal_correction_gain', 1.0)
        self.declare_parameter(
            'adaptive_goal_correction_max_total_deg', 3.0
        )
        self.declare_parameter(
            'adaptive_goal_correction_max_attempts', 4
        )
        self.declare_parameter(
            'home_joint_angles_deg',
            [87.01, 55.54, -83.58, -36.47, 4.57, -52.11],
        )
        self.declare_parameter('go_home_on_startup', True)
        self.declare_parameter('go_home_on_shutdown', True)
        self.declare_parameter('home_speed', 50)
        self.declare_parameter('home_tolerance_deg', 3.5)
        self.declare_parameter('home_timeout_sec', 15.0)
        self.declare_parameter(
            'a1_joint_angles_deg',
            [3.77, -4.65, -7.55, -71.01, 0.7, -51.15],
        )
        self.declare_parameter('a1_speed', 30)
        self.declare_parameter(
            'a2_joint_angles_deg',
            [-40.07, -6.24, -7.55, -62.84, 0.87, -51.15],
        )
        self.declare_parameter('a2_speed', 30)
        self.declare_parameter(
            'a3_joint_angles_deg',
            [-90.0, -6.24, -6.67, -73.91, -0.08, -52.11],
        )
        self.declare_parameter('a3_speed', 30)
        self.declare_parameter('joint1_sweep_speed', 30)
        self.declare_parameter('joint1_sweep_angle_deg', 180.0)
        self.declare_parameter('joint1_sweep_tolerance_deg', 3.5)
        self.declare_parameter('joint1_sweep_duration_sec', 10.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('joint_states_topic', '/arm2/joint_states')
        self.declare_parameter(
            'joint_state_limit_clamp_tolerance_deg', 0.5
        )
        self.declare_parameter(
            'follow_joint_trajectory_action',
            '/arm2/arm_group_controller/follow_joint_trajectory',
        )

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.speed = int(self.get_parameter('speed').value)
        self.command_period = 1.0 / float(
            self.get_parameter('command_rate_hz').value
        )
        self.max_start_error = float(
            self.get_parameter('max_start_error_deg').value
        )
        self.max_j6_trajectory_travel = float(
            self.get_parameter('max_j6_trajectory_travel_deg').value
        )
        self.goal_tolerance = float(
            self.get_parameter('goal_tolerance_deg').value
        )
        self.goal_timeout = float(
            self.get_parameter('goal_timeout_sec').value
        )
        self.goal_correction_speed = int(
            self.get_parameter('goal_correction_speed').value
        )
        self.goal_correction_period = float(
            self.get_parameter('goal_correction_period_sec').value
        )
        self.adaptive_goal_correction_enabled = bool(
            self.get_parameter(
                'adaptive_goal_correction_enabled'
            ).value
        )
        adaptive_joint_names = [
            str(value) for value in self.get_parameter(
                'adaptive_goal_correction_joints'
            ).value
        ]
        unknown_adaptive_joints = (
            set(adaptive_joint_names) - set(JOINT_NAMES)
        )
        if unknown_adaptive_joints:
            raise ValueError(
                'Unknown adaptive correction joints: '
                f'{sorted(unknown_adaptive_joints)}'
            )
        self.adaptive_goal_correction_indices = [
            JOINT_NAMES.index(name) for name in adaptive_joint_names
        ]
        self.adaptive_goal_correction_tolerance = float(
            self.get_parameter(
                'adaptive_goal_correction_tolerance_deg'
            ).value
        )
        self.adaptive_goal_correction_gain = float(
            self.get_parameter(
                'adaptive_goal_correction_gain'
            ).value
        )
        self.adaptive_goal_correction_max_total = float(
            self.get_parameter(
                'adaptive_goal_correction_max_total_deg'
            ).value
        )
        self.adaptive_goal_correction_max_attempts = int(
            self.get_parameter(
                'adaptive_goal_correction_max_attempts'
            ).value
        )
        self.home_angles = validate_home_angles(
            self.get_parameter('home_joint_angles_deg').value
        )
        self.go_home_on_startup = bool(
            self.get_parameter('go_home_on_startup').value
        )
        self.go_home_on_shutdown = bool(
            self.get_parameter('go_home_on_shutdown').value
        )
        self.home_speed = int(self.get_parameter('home_speed').value)
        self.home_tolerance = float(
            self.get_parameter('home_tolerance_deg').value
        )
        self.home_timeout = float(
            self.get_parameter('home_timeout_sec').value
        )
        self.a1_angles = validate_home_angles(
            self.get_parameter('a1_joint_angles_deg').value
        )
        self.a1_speed = int(self.get_parameter('a1_speed').value)
        self.a2_angles = validate_home_angles(
            self.get_parameter('a2_joint_angles_deg').value
        )
        self.a2_speed = int(self.get_parameter('a2_speed').value)
        self.a3_angles = validate_home_angles(
            self.get_parameter('a3_joint_angles_deg').value
        )
        self.a3_speed = int(self.get_parameter('a3_speed').value)
        self.joint1_sweep_speed = int(
            self.get_parameter('joint1_sweep_speed').value
        )
        self.joint1_sweep_angle = float(
            self.get_parameter('joint1_sweep_angle_deg').value
        )
        self.joint1_sweep_tolerance = float(
            self.get_parameter('joint1_sweep_tolerance_deg').value
        )
        self.joint1_sweep_duration = float(
            self.get_parameter('joint1_sweep_duration_sec').value
        )
        self.joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        self.joint_state_limit_clamp_tolerance = float(
            self.get_parameter(
                'joint_state_limit_clamp_tolerance_deg'
            ).value
        )
        if not 0.0 <= self.joint_state_limit_clamp_tolerance <= 2.0:
            raise ValueError(
                'joint_state_limit_clamp_tolerance_deg must be within 0..2'
            )
        self.last_joint_state_clamp_log = 0.0
        self.follow_joint_trajectory_action = str(
            self.get_parameter('follow_joint_trajectory_action').value
        )
        if not 1 <= self.goal_correction_speed <= 100:
            raise ValueError('goal_correction_speed must be within 1..100')
        if self.max_j6_trajectory_travel <= 0.0:
            raise ValueError(
                'max_j6_trajectory_travel_deg must be positive'
            )
        if self.goal_correction_period <= 0.0:
            raise ValueError('goal_correction_period_sec must be positive')
        if self.adaptive_goal_correction_tolerance <= 0.0:
            raise ValueError(
                'adaptive correction tolerance must be positive'
            )
        if not 0.0 < self.adaptive_goal_correction_gain <= 1.0:
            raise ValueError('adaptive correction gain must be within (0, 1]')
        if self.adaptive_goal_correction_max_total <= 0.0:
            raise ValueError(
                'adaptive correction maximum must be positive'
            )
        if self.adaptive_goal_correction_max_attempts < 1:
            raise ValueError(
                'adaptive correction attempts must be positive'
            )
        if not 1 <= self.home_speed <= 100:
            raise ValueError('home_speed must be within 1..100')
        if not 1 <= self.a1_speed <= 100:
            raise ValueError('a1_speed must be within 1..100')
        if not 1 <= self.a2_speed <= 100:
            raise ValueError('a2_speed must be within 1..100')
        if not 1 <= self.a3_speed <= 100:
            raise ValueError('a3_speed must be within 1..100')
        if not 1 <= self.joint1_sweep_speed <= 100:
            raise ValueError('joint1_sweep_speed must be within 1..100')
        joint1_range = JOINT_LIMITS_DEG[0][1] - JOINT_LIMITS_DEG[0][0]
        if not 0.0 < self.joint1_sweep_angle <= joint1_range:
            raise ValueError(
                f'joint1_sweep_angle_deg must be within (0, {joint1_range:g}]'
            )
        if self.home_tolerance <= 0.0 or self.home_timeout <= 0.0:
            raise ValueError('home tolerance and timeout must be positive')
        if (
            self.joint1_sweep_tolerance <= 0.0
            or self.joint1_sweep_duration <= 0.0
        ):
            raise ValueError(
                'joint1 sweep tolerance and duration must be positive'
            )

        self.serial_lock = threading.Lock()
        self.execution_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.sweep_pause_event = threading.Event()
        self.sweep_recommand_event = threading.Event()
        self.sweep_active_event = threading.Event()
        self.last_angles = None
        self.robot = MyCobot280(serial_port, baud_rate)
        time.sleep(1.0)
        self.robot.set_fresh_mode(1)
        if self.robot.is_power_on() != 1:
            self.robot.power_on()
            time.sleep(0.5)
        self.robot.focus_all_servos()
        time.sleep(0.5)
        if self.robot.is_all_servo_enable() != 1:
            raise RuntimeError('JetCobot servos are not enabled')

        callback_group = ReentrantCallbackGroup()
        self.joint_state_publisher = self.create_publisher(
            JointState, self.joint_states_topic, 10
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            self.follow_joint_trajectory_action,
            execute_callback=self.execute_trajectory,
            goal_callback=self.accept_goal,
            cancel_callback=self.cancel_goal,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/gripper/open',
            self.open_gripper,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/gripper/close',
            self.close_gripper,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/stop_robot',
            self.stop_robot,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/return_home',
            self.return_home,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/go_initial_pose',
            self.return_home,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/go_a1_pose',
            self.go_a1_pose,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/go_a2_pose',
            self.go_a2_pose,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/go_a3_pose',
            self.go_a3_pose,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/sweep_joint1',
            self.sweep_joint1,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/scan_joint1',
            self.scan_joint1,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/pause_joint1_sweep',
            self.pause_joint1_sweep,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/resume_joint1_sweep',
            self.resume_joint1_sweep,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/joint1_scan_state',
            self.joint1_scan_state,
            callback_group=callback_group,
        )
        state_rate = float(self.get_parameter('joint_state_rate_hz').value)
        self.create_timer(
            1.0 / state_rate,
            self.publish_joint_states,
            callback_group=callback_group,
        )
        self.get_logger().info(
            'JetCobot trajectory bridge ready: action='
            f'{self.follow_joint_trajectory_action}, '
            f'joint_states={self.joint_states_topic}, '
            f'port={serial_port}, speed={self.speed}'
        )
        if self.go_home_on_startup:
            self.get_logger().info(
                f'Moving to startup home: {self.home_angles}'
            )
            self._move_home()

    def accept_goal(self, goal_request):
        """Reject malformed or concurrent trajectory goals."""
        if self.execution_lock.locked():
            self.get_logger().warning('Rejected concurrent trajectory goal')
            return GoalResponse.REJECT
        trajectory = goal_request.trajectory
        if set(trajectory.joint_names) != set(JOINT_NAMES):
            self.get_logger().error(
                f'Invalid trajectory joints: {trajectory.joint_names}'
            )
            return GoalResponse.REJECT
        if not trajectory.points:
            self.get_logger().error('Rejected empty trajectory')
            return GoalResponse.REJECT
        previous_time = -1.0
        for point in trajectory.points:
            point_time = duration_seconds(point.time_from_start)
            if len(point.positions) != len(JOINT_NAMES):
                return GoalResponse.REJECT
            if point_time < previous_time:
                return GoalResponse.REJECT
            previous_time = point_time
        return GoalResponse.ACCEPT

    def cancel_goal(self, _goal_handle):
        """Accept cancellation and stop the physical arm."""
        self.cancel_event.set()
        self._stop_hardware()
        return CancelResponse.ACCEPT

    def execute_trajectory(self, goal_handle):
        """Stream an accepted MoveIt trajectory to the physical arm."""
        result = FollowJointTrajectory.Result()
        if not self.execution_lock.acquire(blocking=False):
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = 'another trajectory is active'
            goal_handle.abort()
            return result
        try:
            self.cancel_event.clear()
            trajectory = self._ordered_trajectory(goal_handle.request.trajectory)
            current = self._read_angles()
            if current is None:
                return self._abort(goal_handle, result, 'failed to read joints')
            first = [math.degrees(value) for value in trajectory[0].positions]
            start_error = max(abs(a - b) for a, b in zip(first, current))
            if start_error > self.max_start_error:
                return self._abort(
                    goal_handle,
                    result,
                    f'trajectory start error {start_error:.1f}deg exceeds limit',
                )
            for point in trajectory:
                self._validate_joint_limits(point.positions)
            j6_travel = cumulative_joint_travel_degrees(
                current[5], trajectory, 5
            )
            if j6_travel > self.max_j6_trajectory_travel:
                return self._abort(
                    goal_handle,
                    result,
                    f'J6 trajectory travel {j6_travel:.1f}deg exceeds '
                    f'{self.max_j6_trajectory_travel:.1f}deg limit',
                )

            finish = duration_seconds(trajectory[-1].time_from_start)
            max_step = max(
                (
                    max(
                        abs(math.degrees(end - start))
                        for start, end in zip(
                            previous.positions, following.positions
                        )
                    )
                    for previous, following in zip(
                        trajectory, trajectory[1:]
                    )
                ),
                default=0.0,
            )
            self.get_logger().info(
                'Executing physical trajectory: '
                f'points={len(trajectory)}, duration={finish:.2f}s, '
                f'max_point_step={max_step:.2f}deg, speed={self.speed}'
            )

            start = time.monotonic()
            while True:
                if goal_handle.is_cancel_requested or self.cancel_event.is_set():
                    self._stop_hardware()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'trajectory canceled'
                    return result
                elapsed = time.monotonic() - start
                desired = interpolate_positions(trajectory, elapsed)
                self._send_radians(desired)
                actual = self._read_angles()
                if actual is not None:
                    self._publish_feedback(goal_handle, desired, actual, elapsed)
                if elapsed >= finish:
                    break
                time.sleep(self.command_period)

            final_degrees = [
                math.degrees(value) for value in trajectory[-1].positions
            ]
            deadline = time.monotonic() + self.goal_timeout
            last_correction = 0.0
            correction_attempts = 0
            correction_command = list(final_degrees)
            last_actual = None
            last_errors = None
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested or self.cancel_event.is_set():
                    self._stop_hardware()
                    goal_handle.canceled()
                    result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
                    result.error_string = 'trajectory canceled'
                    return result
                actual = self._read_angles()
                if actual is not None:
                    errors = joint_errors_degrees(actual, final_degrees)
                    error = max(abs(value) for value in errors)
                    adaptive_error = max(
                        (
                            abs(errors[index])
                            for index in
                            self.adaptive_goal_correction_indices
                        ),
                        default=0.0,
                    )
                    last_actual = list(actual)
                    last_errors = errors
                    adaptive_reached = (
                        not self.adaptive_goal_correction_enabled
                        or adaptive_error
                        <= self.adaptive_goal_correction_tolerance
                    )
                    if error <= self.goal_tolerance and adaptive_reached:
                        goal_handle.succeed()
                        result.error_code = (
                            FollowJointTrajectory.Result.SUCCESSFUL
                        )
                        result.error_string = 'trajectory completed'
                        return result
                    now = time.monotonic()
                    if now - last_correction >= self.goal_correction_period:
                        if (
                            self.adaptive_goal_correction_enabled
                            and not adaptive_reached
                        ):
                            if (
                                correction_attempts
                                >= self.adaptive_goal_correction_max_attempts
                            ):
                                break
                            correction_command = (
                                adaptive_joint_command_degrees(
                                    final_degrees,
                                    actual,
                                    correction_command,
                                    self.adaptive_goal_correction_indices,
                                    self.adaptive_goal_correction_gain,
                                    self.adaptive_goal_correction_max_total,
                                )
                            )
                            validate_home_angles(correction_command)
                            correction_attempts += 1
                        self.get_logger().info(
                            'Correcting final joint target: '
                            f'max_error={error:.2f}deg, '
                            f'adaptive_error={adaptive_error:.2f}deg, '
                            f'attempt={correction_attempts}/'
                            f'{self.adaptive_goal_correction_max_attempts}, '
                            'errors='
                            f'{[round(value, 2) for value in errors]}, '
                            'command='
                            f'{[round(value, 2) for value in correction_command]}, '
                            f'speed={self.goal_correction_speed}'
                        )
                        self._send_degrees(
                            correction_command,
                            speed=self.goal_correction_speed,
                        )
                        last_correction = now
                time.sleep(0.2)
            detail = 'physical arm missed goal tolerance'
            if last_actual is not None and last_errors is not None:
                detail += (
                    f'; target={[round(value, 2) for value in final_degrees]}, '
                    f'actual={[round(value, 2) for value in last_actual]}, '
                    f'errors={[round(value, 2) for value in last_errors]}'
                )
            return self._abort(
                goal_handle, result, detail
            )
        except Exception as exc:
            self._stop_hardware()
            return self._abort(goal_handle, result, str(exc))
        finally:
            self.execution_lock.release()

    def _ordered_trajectory(self, trajectory):
        order = [trajectory.joint_names.index(name) for name in JOINT_NAMES]
        ordered = []
        for source in trajectory.points:
            point = JointTrajectoryPoint()
            point.positions = [source.positions[index] for index in order]
            point.velocities = (
                [source.velocities[index] for index in order]
                if len(source.velocities) == len(order) else []
            )
            point.time_from_start = source.time_from_start
            ordered.append(point)
        return ordered

    @staticmethod
    def _validate_joint_limits(positions):
        for index, (radians, limits) in enumerate(
            zip(positions, JOINT_LIMITS_DEG)
        ):
            degrees = math.degrees(float(radians))
            if not limits[0] <= degrees <= limits[1]:
                raise RuntimeError(
                    f'J{index + 1} target {degrees:.1f}deg outside limits'
                )

    def _send_radians(self, positions, speed=None):
        degrees = [math.degrees(float(value)) for value in positions]
        self._send_degrees(degrees, speed)

    def _send_degrees(self, degrees, speed=None):
        """Send one six-joint degree command to the physical controller."""
        command_speed = self.speed if speed is None else int(speed)
        with self.serial_lock:
            try:
                self.robot.send_angles(
                    degrees, command_speed, _async=True
                )
            except TypeError:
                self.robot.send_angles(degrees, command_speed)

    def _read_angles(self):
        if not self.serial_lock.acquire(blocking=False):
            return self.last_angles
        try:
            angles = self.robot.get_angles()
        except Exception as exc:
            self.get_logger().warning(f'Joint read failed: {exc}')
            return self.last_angles
        finally:
            self.serial_lock.release()
        if isinstance(angles, (list, tuple)) and len(angles) == 6:
            self.last_angles = [float(value) for value in angles]
        return self.last_angles

    def _publish_feedback(self, goal_handle, desired, actual_degrees, elapsed):
        feedback = FollowJointTrajectory.Feedback()
        feedback.header.stamp = self.get_clock().now().to_msg()
        feedback.joint_names = JOINT_NAMES
        feedback.desired.positions = [float(value) for value in desired]
        feedback.actual.positions = [
            math.radians(value) for value in actual_degrees
        ]
        feedback.error.positions = [
            target - actual
            for target, actual in zip(
                feedback.desired.positions, feedback.actual.positions
            )
        ]
        seconds = int(elapsed)
        feedback.actual.time_from_start.sec = seconds
        feedback.actual.time_from_start.nanosec = int(
            (elapsed - seconds) * 1e9
        )
        goal_handle.publish_feedback(feedback)

    def publish_joint_states(self):
        """Publish measured physical joints for MoveIt's state monitor."""
        angles = self._read_angles()
        if angles is None:
            return
        planning_angles = clamp_measured_joints_for_planning(
            angles,
            self.joint_state_limit_clamp_tolerance,
        )
        if planning_angles != [float(value) for value in angles]:
            now = time.monotonic()
            if now - self.last_joint_state_clamp_log >= 5.0:
                self.get_logger().warning(
                    'Clamping small measured joint-limit overshoot for '
                    f'MoveIt state only: measured={angles}, '
                    f'published={planning_angles}'
                )
                self.last_joint_state_clamp_log = now
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [
            math.radians(value) for value in planning_angles
        ]
        message.velocity = [0.0] * len(JOINT_NAMES)
        message.effort = [0.0] * len(JOINT_NAMES)
        self.joint_state_publisher.publish(message)

    def open_gripper(self, _request, response):
        return self._set_gripper(True, response)

    def close_gripper(self, _request, response):
        return self._set_gripper(False, response)

    def _set_gripper(self, open_gripper, response):
        parameter = (
            'gripper_open_value'
            if open_gripper else 'gripper_closed_value'
        )
        value = int(self.get_parameter(parameter).value)
        speed = int(self.get_parameter('gripper_speed').value)
        try:
            with self.serial_lock:
                self.robot.set_gripper_value(value, speed)
            response.success = True
            response.message = f'gripper command sent: value={value}'
        except Exception as exc:
            response.success = False
            response.message = f'gripper command failed: {exc}'
        return response

    def stop_robot(self, _request, response):
        self.cancel_event.set()
        self.sweep_pause_event.clear()
        self.sweep_recommand_event.clear()
        try:
            self._stop_hardware()
            response.success = True
            response.message = 'robot stop sent'
        except Exception as exc:
            response.success = False
            response.message = f'robot stop failed: {exc}'
        return response

    def pause_joint1_sweep(self, _request, response):
        """Pause an active J1 sweep without ending its time budget."""
        self.sweep_pause_event.set()
        response.success = True
        response.message = 'J1 sweep pause requested'
        return response

    def resume_joint1_sweep(self, _request, response):
        """Allow a paused J1 sweep to continue."""
        self.sweep_pause_event.clear()
        self.sweep_recommand_event.set()
        response.success = True
        response.message = 'J1 sweep resumed; target will be resent'
        return response

    def joint1_scan_state(self, _request, response):
        """Report whether the bounded J1 scan owns the hardware lock."""
        response.success = self.sweep_active_event.is_set()
        response.message = (
            'J1 scan active' if response.success else 'J1 scan inactive'
        )
        return response

    def return_home(self, _request, response):
        """Move the physical arm to the configured joint-space home."""
        try:
            self._move_home()
            response.success = True
            response.message = f'home reached: {self.home_angles}'
        except Exception as exc:
            response.success = False
            response.message = f'home move failed: {exc}'
        return response

    def go_a1_pose(self, _request, response):
        """Move the physical arm to the configured A-1 joint pose."""
        try:
            self._run_exclusive_targets(
                (self.a1_angles,),
                self.a1_speed,
                self.home_tolerance,
                self.home_timeout,
            )
            response.success = True
            response.message = f'A-1 pose reached: {self.a1_angles}'
        except Exception as exc:
            response.success = False
            response.message = f'A-1 pose move failed: {exc}'
        return response

    def go_a2_pose(self, _request, response):
        """Move the physical arm to the configured A-2 joint pose."""
        try:
            self._run_exclusive_targets(
                (self.a2_angles,),
                self.a2_speed,
                self.home_tolerance,
                self.home_timeout,
            )
            response.success = True
            response.message = f'A-2 pose reached: {self.a2_angles}'
        except Exception as exc:
            response.success = False
            response.message = f'A-2 pose move failed: {exc}'
        return response

    def go_a3_pose(self, _request, response):
        """Move the physical arm to the configured A-3 joint pose."""
        try:
            self._run_exclusive_targets(
                (self.a3_angles,),
                self.a3_speed,
                self.home_tolerance,
                self.home_timeout,
            )
            response.success = True
            response.message = f'A-3 pose reached: {self.a3_angles}'
        except Exception as exc:
            response.success = False
            response.message = f'A-3 pose move failed: {exc}'
        return response

    def sweep_joint1(self, _request, response):
        """Sweep J1 for a bounded duration, then return the robot home."""
        sweep_error = None
        try:
            self._sweep_joint1_for_duration()
        except Exception as exc:
            sweep_error = exc
            self.get_logger().error(f'J1 sweep failed: {exc}')
        try:
            self.get_logger().info('J1 sweep finished; returning home')
            self._move_home()
            response.success = sweep_error is None
            response.message = (
                'J1 sweep completed and home reached'
                if sweep_error is None
                else f'J1 sweep failed, but home reached: {sweep_error}'
            )
        except Exception as exc:
            response.success = False
            response.message = (
                f'J1 sweep/home failed: sweep={sweep_error}, home={exc}'
            )
        return response

    def scan_joint1(self, _request, response):
        """Run one bounded J1 scan without an automatic home movement."""
        try:
            self._sweep_joint1_for_duration()
            response.success = True
            response.message = 'J1 scan pass completed'
        except Exception as exc:
            response.success = False
            response.message = f'J1 scan failed: {exc}'
        return response

    def _sweep_joint1_for_duration(self):
        """Visit J1 endpoints until both are reached or time expires."""
        self.cancel_event.set()
        self._stop_hardware()
        if not self.execution_lock.acquire(timeout=2.0):
            raise RuntimeError('trajectory execution did not stop')
        try:
            self.sweep_active_event.set()
            self.cancel_event.clear()
            self.sweep_pause_event.clear()
            self.sweep_recommand_event.clear()
            current = self._read_angles()
            if current is None:
                raise RuntimeError('failed to read current joint angles')
            targets = []
            half_angle = self.joint1_sweep_angle / 2.0
            for endpoint in (half_angle, -half_angle):
                target = list(current)
                target[0] = endpoint
                targets.append(target)
            deadline = time.monotonic() + self.joint1_sweep_duration
            self.get_logger().warning(
                'J1 bounded sweep starting: '
                f'{targets[0][0]:g}deg -> {targets[1][0]:g}deg, '
                f'speed={self.joint1_sweep_speed}, '
                f'duration={self.joint1_sweep_duration:g}s'
            )
            for target in targets:
                command_required = True
                pause_started = None
                while time.monotonic() < deadline:
                    if self.cancel_event.is_set():
                        return
                    if self.sweep_pause_event.is_set():
                        if pause_started is None:
                            self._stop_hardware()
                            pause_started = time.monotonic()
                        time.sleep(0.05)
                        continue
                    if pause_started is not None:
                        deadline += time.monotonic() - pause_started
                        pause_started = None
                        command_required = True
                    if self.sweep_recommand_event.is_set():
                        self.sweep_recommand_event.clear()
                        command_required = True
                    if command_required:
                        with self.serial_lock:
                            try:
                                self.robot.send_angles(
                                    target,
                                    self.joint1_sweep_speed,
                                    _async=True,
                                )
                            except TypeError:
                                self.robot.send_angles(
                                    target, self.joint1_sweep_speed
                                )
                        command_required = False
                    actual = self._read_angles()
                    if (
                        actual is not None
                        and abs(actual[0] - target[0])
                        <= self.joint1_sweep_tolerance
                    ):
                        break
                    time.sleep(0.1)
                else:
                    break
            self._stop_hardware()
        finally:
            self.sweep_active_event.clear()
            self.execution_lock.release()

    def _move_home(self):
        """Stop active execution, command home, and wait for convergence."""
        self._run_exclusive_targets(
            (self.home_angles,),
            self.home_speed,
            self.home_tolerance,
            self.home_timeout,
        )

    def _run_exclusive_targets(
        self, targets, speed, tolerance, timeout
    ):
        """Stop trajectory execution and visit joint targets exclusively."""
        self.cancel_event.set()
        self._stop_hardware()
        if not self.execution_lock.acquire(timeout=2.0):
            raise RuntimeError('trajectory execution did not stop')
        try:
            for target in targets:
                validate_home_angles(target)
                with self.serial_lock:
                    try:
                        self.robot.send_angles(
                            target, speed, _async=True
                        )
                    except TypeError:
                        self.robot.send_angles(target, speed)
                deadline = time.monotonic() + timeout
                last_angles = None
                while time.monotonic() < deadline:
                    last_angles = self._read_angles()
                    if last_angles is not None:
                        error = max(
                            abs(value)
                            for value in joint_errors_degrees(
                                last_angles, target
                            )
                        )
                        if error <= tolerance:
                            if rclpy.ok():
                                self.get_logger().info(
                                    'Joint target reached: target='
                                    f'{[round(value, 2) for value in target]}, '
                                    'actual='
                                    f'{[round(value, 2) for value in last_angles]}'
                                )
                            break
                    time.sleep(0.2)
                else:
                    raise RuntimeError(
                        'joint target timeout; target='
                        f'{target}, actual={last_angles}'
                    )
        finally:
            self.execution_lock.release()

    def _stop_hardware(self):
        with self.serial_lock:
            self.robot.stop()

    @staticmethod
    def _abort(goal_handle, result, message):
        goal_handle.abort()
        result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
        result.error_string = message
        return result

    def destroy_node(self):
        try:
            if self.go_home_on_shutdown:
                if rclpy.ok():
                    self.get_logger().info('Moving to shutdown home')
                self._move_home()
            else:
                self._stop_hardware()
        except (Exception, KeyboardInterrupt) as exc:
            if rclpy.ok():
                self.get_logger().error(f'Shutdown home failed: {exc}')
            try:
                self._stop_hardware()
            except (Exception, KeyboardInterrupt):
                pass
        self.action_server.destroy()
        return super().destroy_node()


def main(args=None):
    """Run the JetCobot trajectory bridge."""
    rclpy.init(args=args)
    node = JetCobotTrajectoryBridge()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
