"""Run a deliberately simple ArUco container pick/place sequence."""

from collections import deque
import copy
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
from pymycobot.mycobot280 import MyCobot280
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

from .sequence import (
    Heights,
    MarkerPose,
    observation_roles,
    pick_steps,
    place_steps,
    pose_within_tolerance,
    remaining_observation_roles,
    wrap_degrees,
)

try:
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import (
        Constraints,
        JointConstraint,
        MoveItErrorCodes,
        OrientationConstraint,
        PositionConstraint,
    )
    from moveit_msgs.srv import GetCartesianPath
except ImportError:
    ExecuteTrajectory = None
    GetCartesianPath = None
    MoveGroup = None


def quaternion_from_rpy_degrees(roll, pitch, yaw):
    """Return a normalized XYZW quaternion."""
    roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def yaw_from_quaternion(quaternion):
    """Extract base-frame yaw in degrees from XYZW."""
    x, y, z, w = quaternion
    sin_yaw = 2.0 * (w * z + x * y)
    cos_yaw = 1.0 - 2.0 * (y * y + z * z)
    return wrap_degrees(math.degrees(math.atan2(sin_yaw, cos_yaw)))


class SimplePickPlace(Node):
    """Freeze ArUco poses once, then execute fixed-attitude Z motions."""

    def __init__(self):
        super().__init__('simple_pick_place')
        self._declare_parameters()
        self.backend = str(self.parameter('motion_backend')).lower()
        if self.backend not in ('direct', 'moveit'):
            raise ValueError('motion_backend must be direct or moveit')

        self.base_frame = str(self.parameter('base_frame'))
        self.pick_frame = str(self.parameter('pick_marker_frame'))
        self.place_frame = str(self.parameter('place_marker_frame'))
        self.minimum_samples = int(self.parameter('minimum_stable_samples'))
        self.translation_std = float(
            self.parameter('max_translation_std_m')
        )
        self.yaw_spread = float(self.parameter('max_yaw_spread_deg'))
        self.marker_age = float(self.parameter('max_marker_age_sec'))
        self.first_observation_timeout = float(
            self.parameter('first_observation_timeout_sec')
        )
        self.second_observation_timeout = float(
            self.parameter('second_observation_timeout_sec')
        )
        self.position_tolerance = float(
            self.parameter('position_tolerance_m')
        )
        self.angle_tolerance = float(
            self.parameter('angle_tolerance_deg')
        )
        self.heights = Heights(
            float(self.parameter('approach_z_m')),
            float(self.parameter('pick_z_offset_m')),
            float(self.parameter('pick_lift_z_m')),
            float(self.parameter('place_z_offset_m')),
            float(self.parameter('retreat_z_m')),
        )
        self.marker_yaw_offset = float(
            self.parameter('marker_yaw_offset_deg')
        )
        self.first_observation_joints = self._joint_pose_parameter(
            'first_observation_joint_angles_deg'
        )
        self.second_observation_joints = self._joint_pose_parameter(
            'second_observation_joint_angles_deg'
        )
        self.observation_settle = float(
            self.parameter('observation_settle_sec')
        )
        self.observation_joint_tolerance = float(
            self.parameter('observation_joint_tolerance_deg')
        )
        self._validate_parameters()

        self.callback_group = ReentrantCallbackGroup()
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        history_length = max(30, self.minimum_samples * 5)
        self.histories = {
            self.pick_frame: deque(maxlen=history_length),
            self.place_frame: deque(maxlen=history_length),
        }
        self.history_lock = threading.Lock()
        self.motion_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motion_thread = None
        self.last_stamps = {}
        self.frozen_markers = {}
        self.detection_enabled = False
        self.detection_started_ns = None
        self.detection_control = self.create_publisher(
            Bool,
            '/arm/simple_pick_place/detection_enabled',
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.status = self.create_publisher(
            String, '/arm/simple_pick_place/status', 10
        )
        self.create_timer(
            0.05, self.collect_marker_samples,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger, '/arm/simple_pick', self.start_pick,
            callback_group=self.callback_group,
        )
        self.set_detection_enabled(False)
        self.create_service(
            Trigger, '/arm/simple_place', self.start_place,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger, '/arm/simple_pick_and_place',
            self.start_pick_and_place,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger, '/arm/simple_stop', self.stop,
            callback_group=self.callback_group,
        )

        self.robot = None
        self.move_group = None
        self.cartesian = None
        self.execute_trajectory = None
        self.gripper_open = None
        self.gripper_close = None
        if self.backend == 'direct':
            self._start_direct_backend()
        else:
            self._start_moveit_backend()
        self.publish_status(
            f'ready: backend={self.backend}, base={self.base_frame}, '
            f'command_frame={self.parameter("command_frame")}, '
            f'marker_yaw_offset={self.marker_yaw_offset:+.1f} deg, '
            f'pose_tolerance=±{self.position_tolerance * 1000.0:.1f} mm/'
            f'±{self.angle_tolerance:.1f} deg'
        )

    def _declare_parameters(self):
        self.declare_parameter('motion_backend', 'direct')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('command_frame', 'arm/controller_coords')
        self.declare_parameter('pick_marker_frame', 'arm/pick_marker')
        self.declare_parameter('place_marker_frame', 'arm/place_marker')
        self.declare_parameter('approach_z_m', 0.20)
        self.declare_parameter('pick_z_offset_m', 0.0)
        self.declare_parameter('pick_lift_z_m', 0.18)
        self.declare_parameter('place_z_offset_m', 0.012)
        self.declare_parameter('retreat_z_m', 0.20)
        self.declare_parameter('marker_yaw_offset_deg', 45.0)
        self.declare_parameter(
            'first_observation_joint_angles_deg',
            [1.0, 57.0, -28.0, -85.0, 16.0, -45.0],
        )
        self.declare_parameter(
            'second_observation_joint_angles_deg',
            [-91.0, 64.0, -71.0, -50.0, 13.0, -50.0],
        )
        self.declare_parameter('observation_settle_sec', 1.0)
        self.declare_parameter('observation_joint_tolerance_deg', 3.0)
        self.declare_parameter('observation_speed', 15)
        self.declare_parameter('observation_correction_attempts', 2)
        self.declare_parameter('minimum_stable_samples', 7)
        self.declare_parameter('max_translation_std_m', 0.003)
        self.declare_parameter('max_yaw_spread_deg', 3.0)
        self.declare_parameter('max_marker_age_sec', 1.0)
        self.declare_parameter('first_observation_timeout_sec', 5.0)
        self.declare_parameter('second_observation_timeout_sec', 15.0)
        self.declare_parameter('position_tolerance_m', 0.005)
        self.declare_parameter('angle_tolerance_deg', 3.0)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 20)
        self.declare_parameter('motion_timeout_sec', 20.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('gripper_wait_sec', 1.0)
        self.declare_parameter('joint_state_rate_hz', 10.0)
        self.declare_parameter('moveit_group', 'arm_group')
        self.declare_parameter('moveit_ee_link', '6_Link')
        self.declare_parameter('moveit_planning_time_sec', 8.0)
        self.declare_parameter('moveit_planning_attempts', 10)
        self.declare_parameter('moveit_velocity_scale', 0.25)
        self.declare_parameter('moveit_acceleration_scale', 0.20)
        self.declare_parameter('cartesian_max_step_m', 0.001)
        self.declare_parameter('cartesian_min_fraction', 0.995)

    def parameter(self, name):
        return self.get_parameter(name).value

    def _joint_pose_parameter(self, name):
        values = np.asarray(self.parameter(name), dtype=np.float64)
        if values.shape != (6,) or not np.all(np.isfinite(values)):
            raise ValueError(f'{name} must contain six finite angles')
        return tuple(float(value) for value in values)

    def _validate_parameters(self):
        if self.minimum_samples < 3:
            raise ValueError('minimum_stable_samples must be at least 3')
        if not math.isfinite(self.marker_yaw_offset):
            raise ValueError('marker_yaw_offset_deg must be finite')
        positive = {
            'max_translation_std_m': self.translation_std,
            'max_yaw_spread_deg': self.yaw_spread,
            'max_marker_age_sec': self.marker_age,
            'first_observation_timeout_sec':
                self.first_observation_timeout,
            'second_observation_timeout_sec':
                self.second_observation_timeout,
            'position_tolerance_m': self.position_tolerance,
            'angle_tolerance_deg': self.angle_tolerance,
            'observation_settle_sec': self.observation_settle,
            'observation_joint_tolerance_deg':
                self.observation_joint_tolerance,
        }
        for name, value in positive.items():
            if value <= 0.0:
                raise ValueError(f'{name} must be positive')
        if self.position_tolerance > 0.020:
            raise ValueError('position_tolerance_m may not exceed 0.020')
        if self.position_tolerance > 0.005:
            self.get_logger().warning(
                'Cartesian position tolerance is relaxed for testing: '
                f'±{self.position_tolerance * 1000.0:.1f} mm'
            )
        if self.angle_tolerance > 10.0:
            raise ValueError('angle_tolerance_deg may not exceed 10.0')
        if self.angle_tolerance > 3.0:
            self.get_logger().warning(
                'Cartesian angle tolerance is relaxed for testing: '
                f'±{self.angle_tolerance:.1f} deg'
            )
        observation_speed = int(self.parameter('observation_speed'))
        if not 1 <= observation_speed <= 100:
            raise ValueError('observation_speed must be within 1..100')
        if int(self.parameter('observation_correction_attempts')) < 0:
            raise ValueError(
                'observation_correction_attempts must be non-negative'
            )

    def _start_direct_backend(self):
        self.robot = MyCobot280(
            str(self.parameter('serial_port')),
            int(self.parameter('baud_rate')),
        )
        time.sleep(1.0)
        self.robot.set_fresh_mode(1)
        if self.robot.is_power_on() != 1:
            self.robot.power_on()
            time.sleep(0.5)
        self.joint_state_publisher = self.create_publisher(
            JointState, '/arm/joint_states', 10
        )
        rate = float(self.parameter('joint_state_rate_hz'))
        if rate <= 0.0:
            raise ValueError('joint_state_rate_hz must be positive')
        self.create_timer(
            1.0 / rate, self.publish_direct_joint_state,
            callback_group=self.callback_group,
        )

    def _start_moveit_backend(self):
        if MoveGroup is None:
            raise RuntimeError('moveit_msgs is unavailable')
        self.move_group = ActionClient(
            self, MoveGroup, '/move_action',
            callback_group=self.callback_group,
        )
        self.cartesian = self.create_client(
            GetCartesianPath, '/compute_cartesian_path',
            callback_group=self.callback_group,
        )
        self.execute_trajectory = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory',
            callback_group=self.callback_group,
        )
        self.gripper_open = self.create_client(
            Trigger, '/arm/gripper/open',
            callback_group=self.callback_group,
        )
        self.gripper_close = self.create_client(
            Trigger, '/arm/gripper/close',
            callback_group=self.callback_group,
        )

    def collect_marker_samples(self):
        """Collect transforms only during a stationary observation window."""
        if not self.detection_enabled:
            return
        now_ns = self.get_clock().now().nanoseconds
        for frame in self.histories:
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame, frame, Time()
                )
            except TransformException:
                continue
            stamp_ns = (
                transform.header.stamp.sec * 1_000_000_000
                + transform.header.stamp.nanosec
            )
            if (
                self.detection_started_ns is None
                or stamp_ns < self.detection_started_ns
            ):
                continue
            if stamp_ns == self.last_stamps.get(frame):
                continue
            age = (now_ns - stamp_ns) / 1e9
            if age < 0.0 or age > self.marker_age:
                continue
            self.last_stamps[frame] = stamp_ns
            translation = np.array([
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
            ], dtype=np.float64)
            rotation = transform.transform.rotation
            yaw = yaw_from_quaternion(
                (rotation.x, rotation.y, rotation.z, rotation.w)
            )
            with self.history_lock:
                self.histories[frame].append(
                    (stamp_ns, translation, yaw)
                )

    def publish_direct_joint_state(self):
        """Publish the same serial owner's joint state for robot TF."""
        if not self.serial_lock.acquire(blocking=False):
            return
        try:
            angles = self.robot.get_angles()
        except Exception:
            return
        finally:
            self.serial_lock.release()
        if not isinstance(angles, (list, tuple)) or len(angles) != 6:
            return
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = [f'{index}_Joint' for index in range(1, 7)]
        message.position = [
            math.radians(float(value)) for value in angles
        ]
        self.joint_state_publisher.publish(message)

    def stable_marker(self, frame):
        with self.history_lock:
            samples = list(self.histories[frame])[-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None
        newest_age = (
            self.get_clock().now().nanoseconds - samples[-1][0]
        ) / 1e9
        if newest_age < 0.0 or newest_age > self.marker_age:
            return None
        xyz = np.asarray([sample[1] for sample in samples])
        if float(np.max(np.std(xyz, axis=0))) > self.translation_std:
            return None
        yaws = np.radians([sample[2] for sample in samples])
        mean_yaw = math.atan2(
            float(np.mean(np.sin(yaws))),
            float(np.mean(np.cos(yaws))),
        )
        spread = max(
            abs(wrap_degrees(math.degrees(value - mean_yaw)))
            for value in yaws
        )
        if spread > self.yaw_spread:
            return None
        mean = np.mean(xyz, axis=0)
        return MarkerPose(
            float(mean[0]), float(mean[1]), float(mean[2]),
            wrap_degrees(math.degrees(mean_yaw)),
        )

    def discover_available(self, roles, timeout):
        """Freeze every requested marker that becomes stable before timeout."""
        frames = {
            'pick': self.pick_frame,
            'place': self.place_frame,
        }
        with self.history_lock:
            for role in roles:
                self.histories[frames[role]].clear()
        deadline = time.monotonic() + timeout
        found = {}
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
                raise RuntimeError('operation stopped')
            for role in roles:
                if role in found:
                    continue
                marker = self.stable_marker(frames[role])
                if marker is None:
                    continue
                found[role] = marker
                self.frozen_markers[role] = marker
                self.publish_status(
                    f'marker frozen: {role}=('
                    f'{marker.x_m:.4f}, {marker.y_m:.4f}, '
                    f'{marker.z_m:.4f}, yaw={marker.yaw_deg:.2f})'
                )
            if len(found) == len(roles):
                break
        return found

    def observe_pose(self, label, angles, roles):
        """Move to one observation pose and return any markers found there."""
        self.set_detection_enabled(False)
        self.publish_status(
            f'observation: moving to {label} pose; wanted={list(roles)}'
        )
        self.move_to_observation_joints(angles)
        self.publish_status(
            f'observation: {label} pose reached; settling'
        )
        if self.stop_event.wait(self.observation_settle):
            raise RuntimeError('operation stopped')
        if not roles:
            self.publish_status(
                f'observation: {label} pose reached; no marker remains'
            )
            return {}
        self.set_detection_enabled(True)
        self.publish_status(
            f'observation: searching {list(roles)} at {label} pose'
        )
        timeout = (
            self.first_observation_timeout
            if label == 'first'
            else self.second_observation_timeout
        )
        try:
            found = self.discover_available(roles, timeout)
        finally:
            self.set_detection_enabled(False)
        missing = [role for role in roles if role not in found]
        if missing:
            suffix = (
                'continuing to the second observation pose'
                if label == 'first'
                else 'observation sequence has no further pose'
            )
            self.publish_status(
                f'observation: {label} pose did not find {missing}; {suffix}'
            )
        return found

    def move_to_observation_joints(self, angles):
        """Move all six joints using the selected motion backend."""
        if self.backend == 'direct':
            timeout = int(math.ceil(
                float(self.parameter('motion_timeout_sec'))
            ))
            attempts = (
                int(self.parameter('observation_correction_attempts')) + 1
            )
            for attempt in range(1, attempts + 1):
                with self.serial_lock:
                    result = self.robot.sync_send_angles(
                        list(angles),
                        int(self.parameter('observation_speed')),
                        timeout=timeout,
                    )
                    measured = self.robot.get_angles()
                if result is False:
                    raise RuntimeError(
                        'direct observation joint move timed out'
                    )
                errors = self._observation_joint_errors(measured, angles)
                if max(errors) <= self.observation_joint_tolerance:
                    return
                if attempt < attempts:
                    self.publish_status(
                        'observation joint residual '
                        f'{max(errors):.2f} deg exceeds tolerance; '
                        f'correction {attempt}/{attempts - 1}'
                    )
            self._verify_observation_joints(measured, angles)
            return
        self._move_moveit_joints(angles)

    @staticmethod
    def _observation_joint_errors(measured, target):
        if not isinstance(measured, (list, tuple)) or len(measured) != 6:
            raise RuntimeError(
                f'failed to read observation joints: {measured}'
            )
        return [
            abs(wrap_degrees(float(actual) - float(goal)))
            for actual, goal in zip(measured, target)
        ]

    def _verify_observation_joints(self, measured, target):
        errors = self._observation_joint_errors(measured, target)
        if max(errors) > self.observation_joint_tolerance:
            raise RuntimeError(
                'observation joint verification failed: '
                f'max_error={max(errors):.2f} deg, errors={errors}'
            )

    def _move_moveit_joints(self, angles):
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action is unavailable')
        constraints = Constraints()
        tolerance = math.radians(self.observation_joint_tolerance)
        for index, angle in enumerate(angles, start=1):
            joint = JointConstraint()
            joint.joint_name = f'{index}_Joint'
            joint.position = math.radians(angle)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        goal = MoveGroup.Goal()
        goal.request.group_name = str(self.parameter('moveit_group'))
        goal.request.num_planning_attempts = int(
            self.parameter('moveit_planning_attempts')
        )
        goal.request.allowed_planning_time = float(
            self.parameter('moveit_planning_time_sec')
        )
        goal.request.max_velocity_scaling_factor = float(
            self.parameter('moveit_velocity_scale')
        )
        goal.request.max_acceleration_scaling_factor = float(
            self.parameter('moveit_acceleration_scale')
        )
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self._goal_handle(
            self.move_group, goal,
            float(self.parameter('moveit_planning_time_sec')) + 5.0,
        )
        wrapped = self._future(
            handle.get_result_async(),
            float(self.parameter('motion_timeout_sec')) + 60.0,
        )
        error = wrapped.result.error_code
        if error.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'MoveIt observation move failed: code={error.val}, '
                f'message={error.message}'
            )

    def start_pick(self, _request, response):
        return self._accept_operation('pick', response)

    def start_place(self, _request, response):
        return self._accept_operation('place', response)

    def start_pick_and_place(self, _request, response):
        return self._accept_operation('pick_and_place', response)

    def _accept_operation(self, operation, response):
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'another manipulation is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.run_operation, args=(operation,), daemon=True
        )
        self.motion_thread.start()
        response.success = True
        response.message = f'{operation} accepted'
        return response

    def stop(self, _request, response):
        self.stop_event.set()
        self.set_detection_enabled(False)
        if self.backend == 'direct' and self.robot is not None:
            try:
                with self.serial_lock:
                    self.robot.stop()
            except Exception:
                pass
        response.success = True
        response.message = 'stop requested'
        return response

    def run_operation(self, operation):
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self.frozen_markers = {}
            markers = {}
            required = observation_roles(operation)
            markers.update(self.observe_pose(
                'first',
                self.first_observation_joints,
                required,
            ))
            remaining = remaining_observation_roles(required, markers)
            # Always visit the second pose. It may be the only view in which
            # every target marker is visible.
            markers.update(self.observe_pose(
                'second',
                self.second_observation_joints,
                remaining,
            ))
            missing = remaining_observation_roles(required, markers)
            if missing:
                raise RuntimeError(
                    'required ArUco markers were not found after both '
                    f'observation poses: {missing}'
                )
            self.publish_status(
                'all required ArUco observations frozen; '
                'starting manipulation'
            )
            if operation in ('pick', 'pick_and_place'):
                self.execute_steps(pick_steps(
                    markers['pick'],
                    self.heights,
                    self.marker_yaw_offset,
                ))
                self.publish_status('pick complete')
            if operation in ('place', 'pick_and_place'):
                self.execute_steps(place_steps(
                    markers['place'],
                    self.heights,
                    self.marker_yaw_offset,
                ))
                self.publish_status('place complete')
            self.publish_status(f'{operation} complete')
        except Exception as exc:
            self.stop_event.set()
            self.set_detection_enabled(False)
            self.publish_status(f'{operation} FAILED: {exc}')
        finally:
            self.set_detection_enabled(False)
            self.motion_lock.release()

    def execute_steps(self, steps):
        previous_pose = None
        for step in steps:
            if self.stop_event.is_set():
                raise RuntimeError('operation stopped')
            if step.action == 'gripper_open':
                self.command_gripper(True)
            elif step.action == 'gripper_close':
                self.command_gripper(False)
            elif step.action == 'move':
                vertical = (
                    previous_pose is not None
                    and step.pose[:2] == previous_pose[:2]
                    and step.pose[3:] == previous_pose[3:]
                )
                self.move(step.pose, vertical=vertical)
                previous_pose = step.pose
            else:
                raise RuntimeError(f'unknown sequence action: {step.action}')

    def move(self, pose_m_deg, vertical=False):
        target_frame = (
            str(self.parameter('command_frame'))
            if self.backend == 'direct'
            else str(self.parameter('moveit_ee_link'))
        )
        self.publish_status(
            f'{target_frame} '
            + ('vertical Z' if vertical else 'pose')
            + ' move -> '
            + str([round(value, 4) for value in pose_m_deg])
        )
        if self.backend == 'direct':
            self._move_direct(pose_m_deg)
        elif vertical:
            self._move_cartesian(pose_m_deg)
        else:
            self._move_moveit(pose_m_deg)
        self._verify_pose(pose_m_deg)

    def _move_direct(self, pose):
        coords = [
            pose[0] * 1000.0,
            pose[1] * 1000.0,
            pose[2] * 1000.0,
            *pose[3:],
        ]
        with self.serial_lock:
            result = self.robot.sync_send_coords(
                coords,
                int(self.parameter('speed')),
                mode=0,
                timeout=int(math.ceil(
                    float(self.parameter('motion_timeout_sec'))
                )),
            )
        if result is False:
            raise RuntimeError(f'JetCobot rejected or timed out: {coords}')

    def _pose_stamped(self, pose):
        message = PoseStamped()
        message.header.frame_id = self.base_frame
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.position.x = pose[0]
        message.pose.position.y = pose[1]
        message.pose.position.z = pose[2]
        qx, qy, qz, qw = quaternion_from_rpy_degrees(*pose[3:])
        message.pose.orientation.x = qx
        message.pose.orientation.y = qy
        message.pose.orientation.z = qz
        message.pose.orientation.w = qw
        return message

    def _constraints(self, target):
        link = str(self.parameter('moveit_ee_link'))
        position = PositionConstraint()
        position.header = target.header
        position.link_name = link
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [self.position_tolerance]
        position.constraint_region.primitives = [primitive]
        position.constraint_region.primitive_poses = [
            copy.deepcopy(target.pose)
        ]
        position.constraint_region.primitive_poses[0].orientation.x = 0.0
        position.constraint_region.primitive_poses[0].orientation.y = 0.0
        position.constraint_region.primitive_poses[0].orientation.z = 0.0
        position.constraint_region.primitive_poses[0].orientation.w = 1.0
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header = target.header
        orientation.link_name = link
        orientation.orientation = target.pose.orientation
        tolerance = math.radians(self.angle_tolerance)
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.weight = 1.0
        result = Constraints()
        result.position_constraints = [position]
        result.orientation_constraints = [orientation]
        return result

    def _move_moveit(self, pose):
        if not self.move_group.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action is unavailable')
        target = self._pose_stamped(pose)
        goal = MoveGroup.Goal()
        goal.request.group_name = str(self.parameter('moveit_group'))
        goal.request.num_planning_attempts = int(
            self.parameter('moveit_planning_attempts')
        )
        goal.request.allowed_planning_time = float(
            self.parameter('moveit_planning_time_sec')
        )
        goal.request.max_velocity_scaling_factor = float(
            self.parameter('moveit_velocity_scale')
        )
        goal.request.max_acceleration_scaling_factor = float(
            self.parameter('moveit_acceleration_scale')
        )
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [self._constraints(target)]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        handle = self._goal_handle(
            self.move_group, goal,
            float(self.parameter('moveit_planning_time_sec')) + 5.0,
        )
        wrapped = self._future(
            handle.get_result_async(),
            float(self.parameter('motion_timeout_sec')) + 60.0,
        )
        error = wrapped.result.error_code
        if error.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'MoveIt pose failed: code={error.val}, '
                f'message={error.message}'
            )

    def _move_cartesian(self, pose):
        if not self.cartesian.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt Cartesian service is unavailable')
        target = self._pose_stamped(pose)
        request = GetCartesianPath.Request()
        request.header = target.header
        request.header.stamp = Time().to_msg()
        request.start_state.is_diff = True
        request.group_name = str(self.parameter('moveit_group'))
        request.link_name = str(self.parameter('moveit_ee_link'))
        request.waypoints = [target.pose]
        request.max_step = float(self.parameter('cartesian_max_step_m'))
        request.jump_threshold = 0.0
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = float(
            self.parameter('moveit_velocity_scale')
        )
        request.max_acceleration_scaling_factor = float(
            self.parameter('moveit_acceleration_scale')
        )
        response = self._future(
            self.cartesian.call_async(request),
            float(self.parameter('moveit_planning_time_sec')) + 5.0,
        )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                f'Cartesian planning failed: {response.error_code.val}'
            )
        minimum = float(self.parameter('cartesian_min_fraction'))
        if response.fraction < minimum:
            raise RuntimeError(
                f'Cartesian path fraction {response.fraction:.4f} '
                f'is below {minimum:.4f}'
            )
        if not self.execute_trajectory.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt trajectory executor is unavailable')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        handle = self._goal_handle(self.execute_trajectory, goal, 5.0)
        wrapped = self._future(
            handle.get_result_async(),
            float(self.parameter('motion_timeout_sec')) + 30.0,
        )
        if wrapped.result.error_code.val != MoveItErrorCodes.SUCCESS:
            raise RuntimeError(
                'Cartesian execution failed: '
                f'{wrapped.result.error_code.val}'
            )

    def _verify_pose(self, target):
        if self.backend == 'direct':
            verified_frame = str(self.parameter('command_frame'))
            with self.serial_lock:
                measured = self.robot.get_coords()
            if not isinstance(measured, (list, tuple)) or len(measured) != 6:
                raise RuntimeError(f'failed to read robot pose: {measured}')
            actual = [
                measured[0] / 1000.0,
                measured[1] / 1000.0,
                measured[2] / 1000.0,
                *measured[3:],
            ]
        else:
            link = str(self.parameter('moveit_ee_link'))
            verified_frame = link
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame, link, Time()
                )
            except TransformException as exc:
                raise RuntimeError(
                    f'cannot verify {self.base_frame}->{link}: {exc}'
                ) from exc
            rotation = transform.transform.rotation
            # The commanded roll/pitch are fixed; use TF yaw plus a quaternion
            # comparison through the same RPY convention for diagnostics.
            q = (rotation.x, rotation.y, rotation.z, rotation.w)
            actual = [
                transform.transform.translation.x,
                transform.transform.translation.y,
                transform.transform.translation.z,
                *self._rpy_from_quaternion(q),
            ]
        if not pose_within_tolerance(
            actual, target, self.position_tolerance, self.angle_tolerance
        ):
            raise RuntimeError(
                f'{verified_frame} pose verification failed (limit: ±'
                f'{self.position_tolerance * 1000.0:.1f} mm, ±'
                f'{self.angle_tolerance:.1f} deg): target={target}, '
                f'actual={actual}'
            )

    @staticmethod
    def _rpy_from_quaternion(q):
        x, y, z, w = q
        sin_roll = 2.0 * (w * x + y * z)
        cos_roll = 1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(sin_roll, cos_roll)
        sin_pitch = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
        pitch = math.asin(sin_pitch)
        yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z),
        )
        return tuple(map(math.degrees, (roll, pitch, yaw)))

    def command_gripper(self, open_gripper):
        if self.backend == 'direct':
            value = int(self.parameter(
                'gripper_open_value'
                if open_gripper else 'gripper_closed_value'
            ))
            with self.serial_lock:
                self.robot.set_gripper_value(
                    value, int(self.parameter('gripper_speed'))
                )
        else:
            client = self.gripper_open if open_gripper else self.gripper_close
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('gripper service is unavailable')
            result = self._future(client.call_async(Trigger.Request()), 5.0)
            if result is None or not result.success:
                raise RuntimeError(
                    'gripper command failed: '
                    + ('no response' if result is None else result.message)
                )
        if self.stop_event.wait(float(self.parameter('gripper_wait_sec'))):
            raise RuntimeError('operation stopped')

    def _goal_handle(self, client, goal, timeout):
        handle = self._future(client.send_goal_async(goal), timeout)
        if handle is None or not handle.accepted:
            raise RuntimeError('motion goal was rejected')
        return handle

    def _future(self, future, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return future.result()
            if self.stop_event.wait(0.02):
                raise RuntimeError('operation stopped')
        raise RuntimeError('ROS request timed out')

    def publish_status(self, text):
        message = String()
        message.data = text
        self.status.publish(message)
        self.get_logger().info(text)

    def set_detection_enabled(self, enabled):
        """Publish and apply the detector/sample collection gate."""
        self.detection_enabled = bool(enabled)
        if self.detection_enabled:
            self.detection_started_ns = (
                self.get_clock().now().nanoseconds
            )
        else:
            self.detection_started_ns = None
            with self.history_lock:
                for history in self.histories.values():
                    history.clear()
        message = Bool()
        message.data = self.detection_enabled
        self.detection_control.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    executor = MultiThreadedExecutor(num_threads=4)
    try:
        node = SimplePickPlace()
        executor.add_node(node)
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
