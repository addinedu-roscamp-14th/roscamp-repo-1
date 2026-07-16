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

from ._joint_limits import JOINT_LIMITS_DEG


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


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


class JetCobotTrajectoryBridge(Node):
    """Own the robot serial port and expose a trajectory action controller."""

    def __init__(self):
        super().__init__('jetcobot_trajectory_bridge')
        self.declare_parameter('serial_port', '/dev/jetcobot')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 10)
        # Smaller streamed joint increments improve tracking at high speed.
        self.declare_parameter('command_rate_hz', 20.0)
        self.declare_parameter('joint_state_rate_hz', 10.0)
        self.declare_parameter('max_start_error_deg', 15.0)
        self.declare_parameter('goal_tolerance_deg', 3.0)
        # The physical servos repeatedly settle around 1.2--1.7 degrees due
        # to backlash/resolution.  A 1-degree cap causes false CONTROL_FAILED
        # timeouts even though the arm has reached its practical endpoint.
        self.declare_parameter('max_effective_goal_tolerance_deg', 2.0)
        self.declare_parameter('goal_timeout_sec', 15.0)
        self.declare_parameter('goal_correction_speed', 50)
        self.declare_parameter('max_effective_goal_correction_speed', 100)
        self.declare_parameter('goal_correction_period_sec', 1.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 100)
        # Opening can run while MoveIt approaches the container.  The close
        # command remains blocking so the lift never starts before grasping.
        self.declare_parameter('fast_gripper_open', True)
        self.declare_parameter('startup_move_enabled', False)
        self.declare_parameter(
            'startup_angles_deg', [0.0, 45.0, -85.0, -25.0, 0.0, 45.0]
        )
        self.declare_parameter('startup_speed', 100)
        self.declare_parameter('startup_tolerance_deg', 3.0)
        self.declare_parameter('startup_timeout_sec', 20.0)

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.speed = int(self.get_parameter('speed').value)
        self.command_period = 1.0 / float(
            self.get_parameter('command_rate_hz').value
        )
        self.max_start_error = float(
            self.get_parameter('max_start_error_deg').value
        )
        requested_goal_tolerance = float(
            self.get_parameter('goal_tolerance_deg').value
        )
        self.goal_tolerance = min(
            requested_goal_tolerance,
            float(self.get_parameter('max_effective_goal_tolerance_deg').value),
        )
        self.goal_timeout = float(
            self.get_parameter('goal_timeout_sec').value
        )
        requested_correction_speed = int(
            self.get_parameter('goal_correction_speed').value
        )
        self.goal_correction_speed = min(
            requested_correction_speed,
            int(self.get_parameter('max_effective_goal_correction_speed').value),
        )
        self.goal_correction_period = float(
            self.get_parameter('goal_correction_period_sec').value
        )
        if self.goal_tolerance <= 0.0:
            raise ValueError('effective goal tolerance must be positive')
        if not 1 <= self.goal_correction_speed <= 100:
            raise ValueError('goal_correction_speed must be within 1..100')
        if self.goal_correction_period <= 0.0:
            raise ValueError('goal_correction_period_sec must be positive')
        if (
            self.goal_tolerance < requested_goal_tolerance
            or self.goal_correction_speed < requested_correction_speed
        ):
            self.get_logger().info(
                'Precision caps applied: '
                f'tolerance={self.goal_tolerance:.2f}deg, '
                f'correction_speed={self.goal_correction_speed}'
            )

        self.serial_lock = threading.Lock()
        self.execution_lock = threading.Lock()
        self.cancel_event = threading.Event()
        self.hand_guiding = False
        self.last_angles = None
        self.robot = MyCobot280(serial_port, baud_rate)
        time.sleep(1.0)
        self.robot.set_fresh_mode(1)
        self._ensure_servos_enabled()

        if bool(self.get_parameter('startup_move_enabled').value):
            self._move_to_startup_position()

        callback_group = ReentrantCallbackGroup()
        self.joint_state_publisher = self.create_publisher(
            JointState, '/joint_states', 10
        )
        self.action_server = ActionServer(
            self,
            FollowJointTrajectory,
            '/arm_group_controller/follow_joint_trajectory',
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
            '/arm2/hand_guiding/start',
            self.start_hand_guiding,
            callback_group=callback_group,
        )
        self.create_service(
            Trigger,
            '/arm2/hand_guiding/finish',
            self.finish_hand_guiding,
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
            '/arm_group_controller/follow_joint_trajectory, '
            f'port={serial_port}, speed={self.speed}'
        )

    def _move_to_startup_position(self):
        """Move to the historically taught camera-ready joint position."""
        target = [
            float(value)
            for value in self.get_parameter('startup_angles_deg').value
        ]
        if len(target) != len(JOINT_NAMES):
            raise ValueError('startup_angles_deg must contain six angles')
        self._validate_joint_limits([math.radians(value) for value in target])

        speed = int(self.get_parameter('startup_speed').value)
        tolerance = float(
            self.get_parameter('startup_tolerance_deg').value
        )
        timeout = float(self.get_parameter('startup_timeout_sec').value)
        if not 1 <= speed <= 100:
            raise ValueError('startup_speed must be within 1..100')
        if tolerance <= 0.0 or timeout <= 0.0:
            raise ValueError(
                'startup tolerance and timeout must be positive'
            )

        initial = self._read_angles()
        self.get_logger().info(
            f'Moving to startup joint position: {target}, speed={speed}, '
            f'actual_before={initial}'
        )
        with self.serial_lock:
            reply = self.robot.send_angles(target, speed, _async=False)
        self.get_logger().info(f'Startup command reply: {reply}')

        deadline = time.monotonic() + timeout
        last_angles = None
        recovery_sent = False
        while time.monotonic() < deadline:
            last_angles = self._read_angles()
            if last_angles is not None:
                error = max(
                    abs(goal - actual)
                    for goal, actual in zip(target, last_angles)
                )
                if error <= tolerance:
                    self.get_logger().info(
                        'Startup joint position reached: '
                        f'max_error={error:.2f}deg'
                    )
                    return
                if (
                    not recovery_sent
                    and initial is not None
                    and time.monotonic() > deadline - timeout + 1.5
                    and max(abs(a - b) for a, b in zip(last_angles, initial))
                    < 0.3
                ):
                    self.get_logger().warning(
                        'No physical startup motion detected; forcing all '
                        'six servos on and resending the target'
                    )
                    self._ensure_servos_enabled()
                    with self.serial_lock:
                        self.robot.send_angles(target, speed, _async=False)
                    recovery_sent = True
            time.sleep(0.2)

        self._stop_hardware()
        raise RuntimeError(
            'failed to reach startup joint position within timeout; '
            f'target={target}, actual={last_angles}'
        )

    def _ensure_servos_enabled(self):
        """Force power, free mode and every individual servo to motion mode."""
        # This runs before every trajectory segment.  Query the aggregate
        # state first: focusing and querying six individual servos over the
        # serial bus adds several seconds even when everything is healthy.
        with self.serial_lock:
            aggregate = self.robot.is_all_servo_enable()
        if aggregate == 1:
            self.get_logger().debug('Servo enable fast check: all=1')
            return

        self.get_logger().warning(
            f'Servo fast check returned all={aggregate}; running recovery'
        )
        with self.serial_lock:
            if self.robot.is_power_on() != 1:
                self.robot.power_on()
                time.sleep(0.5)
            try:
                if self.robot.is_free_mode() == 1:
                    self.robot.set_free_mode(0)
                    time.sleep(0.3)
            except AttributeError:
                pass
            self.robot.focus_all_servos()
            for index in range(1, 7):
                self.robot.focus_servo(index)
        time.sleep(0.8)
        with self.serial_lock:
            aggregate = self.robot.is_all_servo_enable()
            individual = [
                self.robot.is_servo_enable(index) for index in range(1, 7)
            ]
        self.get_logger().info(
            f'Servo enable check: all={aggregate}, joints={individual}'
        )
        if aggregate != 1 and not all(value == 1 for value in individual):
            raise RuntimeError(
                'JetCobot servos are not enabled: '
                f'all={aggregate}, joints={individual}'
            )

    def accept_goal(self, goal_request):
        """Reject malformed or concurrent trajectory goals."""
        if self.hand_guiding:
            self.get_logger().warning(
                'Rejected trajectory while hand guiding is active'
            )
            return GoalResponse.REJECT
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
            self._ensure_servos_enabled()
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
                    last_actual = list(actual)
                    last_errors = errors
                    if error <= self.goal_tolerance:
                        goal_handle.succeed()
                        result.error_code = (
                            FollowJointTrajectory.Result.SUCCESSFUL
                        )
                        result.error_string = 'trajectory completed'
                        return result
                    now = time.monotonic()
                    if now - last_correction >= self.goal_correction_period:
                        self.get_logger().info(
                            'Correcting final joint target: '
                            f'max_error={error:.2f}deg, '
                            f'errors={[round(value, 2) for value in errors]}, '
                            f'speed={self.goal_correction_speed}'
                        )
                        self._send_radians(
                            trajectory[-1].positions,
                            speed=self.goal_correction_speed,
                            wait=True,
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

    def _send_radians(self, positions, speed=None, wait=False):
        degrees = [math.degrees(float(value)) for value in positions]
        command_speed = self.speed if speed is None else int(speed)
        with self.serial_lock:
            try:
                self.robot.send_angles(
                    degrees, command_speed, _async=not wait
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
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = JOINT_NAMES
        message.position = [math.radians(value) for value in angles]
        message.velocity = [0.0] * len(JOINT_NAMES)
        message.effort = [0.0] * len(JOINT_NAMES)
        self.joint_state_publisher.publish(message)

    def open_gripper(self, _request, response):
        return self._set_gripper(True, response)

    def close_gripper(self, _request, response):
        return self._set_gripper(False, response)

    def _set_gripper(self, open_gripper, response):
        parameter = 'gripper_open_value' if open_gripper else 'gripper_closed_value'
        value = int(self.get_parameter(parameter).value)
        speed = int(self.get_parameter('gripper_speed').value)
        try:
            with self.serial_lock:
                if (
                    open_gripper
                    and bool(self.get_parameter('fast_gripper_open').value)
                ):
                    self.robot.set_gripper_state(0, speed)
                else:
                    self.robot.set_gripper_value(value, speed)
            response.success = True
            mode = 'non-blocking open' if open_gripper else 'verified close'
            response.message = (
                f'gripper command sent: value={value}, mode={mode}'
            )
        except Exception as exc:
            response.success = False
            response.message = f'gripper command failed: {exc}'
        return response

    def stop_robot(self, _request, response):
        self.cancel_event.set()
        try:
            self._stop_hardware()
            response.success = True
            response.message = 'robot stop sent'
        except Exception as exc:
            response.success = False
            response.message = f'robot stop failed: {exc}'
        return response

    def start_hand_guiding(self, _request, response):
        """Release servo torque so the supported arm can be positioned by hand."""
        if self.execution_lock.locked():
            response.success = False
            response.message = 'trajectory is active; stop it before hand guiding'
            return response
        if self.hand_guiding:
            response.success = True
            response.message = 'hand guiding is already active'
            return response
        self.hand_guiding = True
        try:
            with self.serial_lock:
                self.robot.stop()
                # The default damping mode is the command verified to work on
                # this Atom firmware. Do not wait for its stale state query.
                self.robot.release_all_servos()
            response.success = True
            response.message = (
                'servo torque release command sent; '
                'support the arm continuously'
            )
            self.get_logger().warning(response.message)
        except Exception as exc:
            try:
                with self.serial_lock:
                    self.robot.focus_all_servos()
            except Exception:
                pass
            self.hand_guiding = False
            response.success = False
            response.message = f'failed to release servo torque: {exc}'
        return response

    def finish_hand_guiding(self, _request, response):
        """Restore torque and hold the arm at its manually taught position."""
        try:
            with self.serial_lock:
                angles = self.robot.get_angles()
                if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                    raise RuntimeError(f'invalid current angles: {angles}')
                angles = [float(value) for value in angles]
                self.robot.focus_all_servos()
                time.sleep(0.5)
                self.robot.send_angles(angles, 20)
                self.last_angles = angles
            self.hand_guiding = False
            response.success = True
            response.message = (
                'servo torque restored; holding the manually taught pose'
            )
            self.get_logger().info(response.message)
        except Exception as exc:
            response.success = False
            response.message = f'failed to restore servo torque: {exc}'
            self.get_logger().error(response.message)
        return response

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
            self._stop_hardware()
        except Exception:
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
