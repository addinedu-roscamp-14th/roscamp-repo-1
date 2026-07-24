"""Execute one guarded, sequential-observation ArUco pick/place sequence."""

from collections import deque
import copy
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from ._joint_limits import JOINT_LIMITS_DEG
from .container_pick_coordinator import (
    apply_radial_xy_offset,
    apply_vertical_pick_offsets,
    CartesianPlanningError,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    ContainerPickCoordinator,
    joint_trajectory_metrics,
    lift_distance_candidates,
    normalize_quaternion,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
    wrap_degrees,
)

try:
    from moveit_msgs.action import MoveGroup
    from moveit_msgs.msg import Constraints, JointConstraint, MoveItErrorCodes
except ImportError:
    Constraints = None
    JointConstraint = None
    MoveGroup = None
    MoveItErrorCodes = None


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


def validated_joint_angles_degrees(values):
    """Return six finite joint angles after enforcing robot limits."""
    angles = np.asarray(values, dtype=np.float64)
    if angles.shape != (6,):
        raise ValueError('observation joint pose must contain 6 angles')
    if not np.all(np.isfinite(angles)):
        raise ValueError('observation joint pose contains a non-finite angle')
    for index, (angle, limits) in enumerate(
        zip(angles, JOINT_LIMITS_DEG)
    ):
        if not limits[0] <= float(angle) <= limits[1]:
            raise ValueError(
                f'observation J{index + 1}={angle:.2f}deg outside '
                f'limits [{limits[0]:.2f}, {limits[1]:.2f}]'
            )
    return angles


def _radical_inverse(index, base):
    """Return one deterministic low-discrepancy value in [0, 1)."""
    result = 0.0
    scale = 1.0 / float(base)
    while index:
        result += (index % base) * scale
        index //= base
        scale /= float(base)
    return result


def alternative_ik_seeds(current_positions, attempts):
    """Generate deterministic, joint-limit-safe seeds for alternate IK."""
    current = np.asarray(current_positions, dtype=np.float64)
    if current.shape != (6,):
        raise ValueError('current joint state must contain six positions')
    count = int(attempts)
    if count < 1:
        raise ValueError('IK branch attempts must be positive')
    seeds = [current.copy()]
    bases = (2, 3, 5, 7, 11, 13)
    limits = np.radians(np.asarray(JOINT_LIMITS_DEG, dtype=np.float64))
    lower = limits[:, 0]
    upper = limits[:, 1]
    margin = np.minimum(np.radians(3.0), (upper - lower) * 0.05)
    lower = lower + margin
    upper = upper - margin
    for sample_index in range(1, count):
        unit = np.asarray([
            _radical_inverse(sample_index, base) for base in bases
        ])
        seeds.append(lower + unit * (upper - lower))
    return seeds


class ContainerPickPlaceCoordinator(ContainerPickCoordinator):
    """Observe two marker IDs from separate arm poses, then pick and place."""

    def _declare_parameters(self):
        super()._declare_parameters()
        self.declare_parameter('place_marker_frame', 'arm/place_marker')
        self.declare_parameter('place_orientation_mode', 'marker_yaw')
        self.declare_parameter('place_offset_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('place_offset_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('place_reference_marker_yaw_deg', 0.0)
        self.declare_parameter('place_pregrasp_lift_m', 0.08)
        self.declare_parameter('place_extra_depth_m', 0.0)
        self.declare_parameter('lift_after_place_m', 0.08)
        self.declare_parameter('minimum_lift_after_place_m', 0.05)
        self.declare_parameter(
            'place_keep_current_orientation_on_approach', True
        )
        self.declare_parameter(
            'first_observation_pose_configured', False
        )
        self.declare_parameter(
            'first_observation_joint_angles_deg',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter(
            'second_observation_pose_configured', False
        )
        self.declare_parameter(
            'second_observation_joint_angles_deg',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('observation_joint_tolerance_deg', 1.0)
        self.declare_parameter('startup_joint_state_timeout_sec', 20.0)
        self.declare_parameter('place_ik_branch_attempts', 24)
        self.declare_parameter('place_ik_timeout_sec', 0.2)
        self.declare_parameter('place_branch_min_fraction', 0.999)

    def __init__(self):
        super().__init__()
        self.place_marker_frame = str(
            self.get_parameter('place_marker_frame').value
        )
        self.place_orientation_mode = str(
            self.get_parameter('place_orientation_mode').value
        ).lower()
        self.place_offset = self._vector_parameter(
            'place_offset_xyz_m', 3
        )
        self.place_rpy = self._vector_parameter(
            'place_offset_rpy_deg', 3
        )
        self.place_rotation = quaternion_from_rpy_degrees(*self.place_rpy)
        self.place_reference_yaw = float(
            self.get_parameter('place_reference_marker_yaw_deg').value
        )
        self.place_pregrasp_lift = float(
            self.get_parameter('place_pregrasp_lift_m').value
        )
        self.place_extra_depth = float(
            self.get_parameter('place_extra_depth_m').value
        )
        self.lift_after_place = float(
            self.get_parameter('lift_after_place_m').value
        )
        self.minimum_lift_after_place = float(
            self.get_parameter('minimum_lift_after_place_m').value
        )
        self.place_keep_current_orientation = bool(
            self.get_parameter(
                'place_keep_current_orientation_on_approach'
            ).value
        )
        self.first_observation_pose_configured = bool(
            self.get_parameter(
                'first_observation_pose_configured'
            ).value
        )
        self.first_observation_joint_angles = (
            validated_joint_angles_degrees(
                self.get_parameter(
                    'first_observation_joint_angles_deg'
                ).value
            )
        )
        self.second_observation_pose_configured = bool(
            self.get_parameter(
                'second_observation_pose_configured'
            ).value
        )
        self.second_observation_joint_angles = (
            validated_joint_angles_degrees(
                self.get_parameter(
                    'second_observation_joint_angles_deg'
                ).value
            )
        )
        self.observation_joint_tolerance = float(
            self.get_parameter('observation_joint_tolerance_deg').value
        )
        if self.observation_joint_tolerance <= 0.0:
            raise ValueError(
                'observation_joint_tolerance_deg must be positive'
            )
        self.startup_joint_state_timeout = float(
            self.get_parameter('startup_joint_state_timeout_sec').value
        )
        if self.startup_joint_state_timeout <= 0.0:
            raise ValueError(
                'startup_joint_state_timeout_sec must be positive'
            )
        self.place_ik_branch_attempts = int(
            self.get_parameter('place_ik_branch_attempts').value
        )
        self.place_ik_timeout = float(
            self.get_parameter('place_ik_timeout_sec').value
        )
        self.place_branch_min_fraction = float(
            self.get_parameter('place_branch_min_fraction').value
        )
        if self.place_ik_branch_attempts < 1:
            raise ValueError('place_ik_branch_attempts must be positive')
        if self.place_ik_timeout <= 0.0:
            raise ValueError('place_ik_timeout_sec must be positive')
        if not 0.99 <= self.place_branch_min_fraction <= 1.0:
            raise ValueError(
                'place_branch_min_fraction must be within [0.99, 1.0]'
            )
        if self.place_orientation_mode not in (
            'fixed', 'marker_yaw', 'marker_full'
        ):
            raise ValueError(
                'place_orientation_mode must be fixed, marker_yaw, or '
                'marker_full'
            )
        if self.place_extra_depth < 0.0:
            raise ValueError('place_extra_depth_m must be non-negative')
        lift_distance_candidates(
            self.lift_after_place,
            self.minimum_lift_after_place,
            self.lift_search_step,
        )

        # The inherited marker_frame/history belong to configured pick ID.
        self.place_history = deque(
            maxlen=max(100, self.minimum_samples * 3)
        )
        self.place_history_lock = threading.Lock()
        self.last_place_transform_stamp = None
        self.last_place_tracking_error = ''
        self.last_place_tracking_error_time = 0.0
        self.place_pose_publisher = self.create_publisher(
            PoseStamped, '/arm/container_pick_place/place_pose', 10
        )
        self.place_pregrasp_publisher = self.create_publisher(
            PoseStamped, '/arm/container_pick_place/place_pregrasp_pose', 10
        )
        self.create_service(
            Trigger, '/arm/pick_and_place', self.start_pick_and_place
        )
        self.create_service(
            Trigger,
            '/arm/preview_pick_and_place',
            self.preview_pick_and_place,
        )
        self.create_timer(0.1, self.update_place_tracking)
        self.joint_state_ready = threading.Event()
        self.latest_joint_positions = None
        self.latest_joint_lock = threading.Lock()
        self.create_subscription(
            JointState, '/joint_states', self.on_joint_state, 10
        )
        self.publish_status(
            'P&P ready: pick='
            f'{self.marker_frame}, place={self.place_marker_frame}'
        )

    def on_joint_state(self, message):
        """Unlock startup after one complete six-joint hardware state."""
        positions_by_name = dict(zip(message.name, message.position))
        if all(name in positions_by_name for name in JOINT_NAMES):
            with self.latest_joint_lock:
                self.latest_joint_positions = np.asarray([
                    positions_by_name[name] for name in JOINT_NAMES
                ], dtype=np.float64)
            self.joint_state_ready.set()

    def update_place_tracking(self):
        """Collect base-frame samples for the place marker."""
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.place_marker_frame, rclpy.time.Time()
            )
        except TransformException as exc:
            error = str(exc)
            now = time.monotonic()
            if (
                error != self.last_place_tracking_error
                or now - self.last_place_tracking_error_time >= 5.0
            ):
                self.get_logger().warning(
                    'Cannot collect place marker sample: '
                    f'{self.base_frame} -> {self.place_marker_frame}: {error}'
                )
                self.last_place_tracking_error = error
                self.last_place_tracking_error_time = now
            return
        stamp = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec
        )
        if stamp == self.last_place_transform_stamp:
            return
        self.last_place_transform_stamp = stamp
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ])
        rotation = normalize_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        with self.place_history_lock:
            self.place_history.append((stamp, translation, rotation))

    def stable_place_pose(self):
        """Return the filtered place marker using the pick stability limits."""
        with self.place_history_lock:
            samples = list(self.place_history)[-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None, (
                f'need {self.minimum_samples - len(samples)} more samples'
            )
        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - samples[-1][0]) / 1e9
        if age < 0.0 or age > self.max_marker_age:
            return None, f'marker age {age:.2f}s exceeds limit'
        translations = np.array([sample[1] for sample in samples])
        translation = np.mean(translations, axis=0)
        translation_std = np.std(translations, axis=0)
        if float(np.max(translation_std)) > self.max_translation_std:
            return None, (
                'translation unstable: std_mm='
                f'{np.round(translation_std * 1000.0, 2).tolist()}'
            )
        if self.place_orientation_mode == 'marker_yaw':
            yaws = np.array([
                math.radians(quaternion_to_rpy_degrees(sample[2])[2])
                for sample in samples
            ])
            mean_yaw = math.atan2(
                float(np.mean(np.sin(yaws))),
                float(np.mean(np.cos(yaws))),
            )
            yaw_spread = max(
                abs(wrap_degrees(math.degrees(yaw - mean_yaw)))
                for yaw in yaws
            )
            if yaw_spread > self.max_yaw_spread:
                return None, f'yaw spread {yaw_spread:.2f}deg exceeds limit'
            rotation = quaternion_from_rpy_degrees(
                0.0, 0.0, math.degrees(mean_yaw)
            )
            return (translation, rotation), 'stable'
        reference = samples[-1][2]
        aligned = [
            value if np.dot(reference, value) >= 0.0 else -value
            for _, _, value in samples
        ]
        rotation = normalize_quaternion(np.mean(aligned, axis=0))
        spread = max(
            math.degrees(2.0 * math.acos(min(
                1.0, abs(float(np.dot(rotation, value)))
            )))
            for value in aligned
        )
        if spread > self.max_rotation_spread:
            return None, f'rotation spread {spread:.2f}deg exceeds limit'
        return (translation, rotation), 'stable'

    def calculate_place_targets(self, validate_workspace=True):
        marker_pose, reason = self.stable_place_pose()
        if marker_pose is None:
            return None, reason
        marker_translation, marker_rotation = marker_pose
        mode = self.place_orientation_mode
        if mode == 'marker_full':
            translation, rotation = compose_pose(
                marker_translation,
                marker_rotation,
                self.place_offset,
                self.place_rotation,
            )
        elif mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            translation, rotation, yaw_delta = compose_yaw_follow_pose(
                marker_translation,
                self.place_offset,
                self.place_rpy,
                marker_yaw,
                self.place_reference_yaw,
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                return None, (
                    f'place yaw delta {yaw_delta:.2f}deg exceeds limit'
                )
        else:
            translation, rotation = compose_fixed_base_pose(
                marker_translation,
                self.place_offset,
                self.place_rotation,
            )
        translation = apply_radial_xy_offset(
            translation, self.target_radial_offset
        )
        place, preplace = apply_vertical_pick_offsets(
            translation,
            self.place_pregrasp_lift,
            self.place_extra_depth,
        )
        if validate_workspace:
            if not self.in_workspace(place):
                return None, f'place target outside workspace: {place}'
            if not self.in_workspace(preplace):
                return None, f'place pregrasp outside workspace: {preplace}'
        stamp = self.get_clock().now().to_msg()
        place_pose = self.make_pose(place, rotation, stamp)
        preplace_pose = self.make_pose(preplace, rotation, stamp)
        self.place_pose_publisher.publish(place_pose)
        self.place_pregrasp_publisher.publish(preplace_pose)
        return (place_pose, preplace_pose), 'targets valid'

    def clear_observation_history(self, role):
        """Discard samples without making a cached TF count as a new sample."""
        if role == 'pick':
            with self.history_lock:
                self.history.clear()
            return
        if role == 'place':
            with self.place_history_lock:
                self.place_history.clear()
            return
        raise ValueError(f'unknown marker role: {role}')

    def calculate_role_targets(self, role, validate_workspace=True):
        """Calculate targets for one configured marker ID."""
        if role == 'pick':
            return self.calculate_targets(
                validate_workspace=validate_workspace
            )
        if role == 'place':
            return self.calculate_place_targets(
                validate_workspace=validate_workspace
            )
        raise ValueError(f'unknown marker role: {role}')

    @staticmethod
    def other_role(role):
        """Return the role assigned to the other configured marker ID."""
        if role == 'pick':
            return 'place'
        if role == 'place':
            return 'pick'
        raise ValueError(f'unknown marker role: {role}')

    def wait_for_first_target(self):
        """Lock exactly one of the two configured markers at the first view."""
        with self.history_lock:
            self.history.clear()
        with self.place_history_lock:
            self.place_history.clear()
        deadline = time.monotonic() + self.stabilization_timeout
        reasons = ('no pick samples', 'no place samples')
        last_report = ''
        last_report_time = 0.0
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                return None, 'pick and place stopped'
            # Observation and motion safety are separate phases. A marker can
            # be locked even when its eventual grasp/place offset needs
            # correction; workspace validation happens after both views.
            pick_targets, pick_reason = self.calculate_targets(
                validate_workspace=False
            )
            place_targets, place_reason = self.calculate_place_targets(
                validate_workspace=False
            )
            pick_ready = pick_targets is not None
            place_ready = place_targets is not None
            if pick_ready:
                return ('pick', pick_targets), 'first marker locked'
            if place_ready:
                return ('place', place_targets), 'first marker locked'
            reasons = (pick_reason, place_reason)
            report = f'pick=({pick_reason}), place=({place_reason})'
            now = time.monotonic()
            if report != last_report or now - last_report_time >= 1.0:
                self.publish_status(f'P&P FIRST VIEW: {report}')
                last_report = report
                last_report_time = now
        return None, (
            'first marker did not stabilize: '
            f'pick=({reasons[0]}), place=({reasons[1]})'
        )

    def wait_for_role_target(self, role):
        """Lock a fresh observation of the requested marker role."""
        self.clear_observation_history(role)
        deadline = time.monotonic() + self.stabilization_timeout
        reason = 'no samples'
        last_report = ''
        last_report_time = 0.0
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                return None, 'pick and place stopped'
            targets, reason = self.calculate_role_targets(
                role, validate_workspace=False
            )
            if targets is not None:
                return targets, f'{role} marker locked'
            now = time.monotonic()
            if reason != last_report or now - last_report_time >= 1.0:
                self.publish_status(
                    f'P&P SECOND VIEW: waiting for {role}: {reason}'
                )
                last_report = reason
                last_report_time = now
        return None, f'second {role} marker did not stabilize: {reason}'

    def move_to_observation_joint_pose(self, angles_degrees):
        """Plan and execute a collision-checked MoveIt joint goal."""
        if self.motion_backend != 'moveit':
            raise RuntimeError(
                'observation joint poses require motion_backend=moveit'
            )
        if MoveGroup is None or JointConstraint is None:
            raise RuntimeError('moveit_msgs is unavailable')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveIt /move_action server is unavailable')

        angles = validated_joint_angles_degrees(angles_degrees)
        constraints = Constraints()
        tolerance = math.radians(self.observation_joint_tolerance)
        for name, angle in zip(JOINT_NAMES, angles):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = math.radians(float(angle))
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)

        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.moveit_group
        request.num_planning_attempts = self.moveit_planning_attempts
        request.allowed_planning_time = self.moveit_planning_time
        request.max_velocity_scaling_factor = self.moveit_velocity_scale
        request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scale
        )
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_future = self.move_group_client.send_goal_async(goal)
        self._wait_future(goal_future, self.moveit_planning_time + 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected observation joint goal')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                self.moveit_planning_time + self.motion_timeout + 60.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError(
                    'MoveIt returned no observation joint result'
                )
            error = wrapped_result.result.error_code
            if error.val != MoveItErrorCodes.SUCCESS:
                detail = error.message or 'no detail'
                raise RuntimeError(
                    'MoveIt observation joint goal failed: '
                    f'code={error.val}, message={detail}'
                )
            if self.stop_event.wait(self.moveit_state_settle):
                raise RuntimeError('pick and place stopped')
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def start_pick_and_place(self, _request, response):
        if not self.execute_motion:
            response.success = False
            response.message = 'Start the MoveIt pick/place launch'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full motion or offsets are not enabled'
            return response
        if self.motion_backend != 'moveit':
            response.success = False
            response.message = 'Observation joint poses require MoveIt'
            return response
        if not self.first_observation_pose_configured:
            response.success = False
            response.message = 'First observation joint pose is not configured'
            return response
        if not self.second_observation_pose_configured:
            response.success = False
            response.message = (
                'Second observation joint pose is not configured'
            )
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_after_stabilization,
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            'Pick and place accepted; starting sequential marker observations'
        )
        return response

    def preview_pick_and_place(self, _request, response):
        pick_targets, pick_reason = self.calculate_targets(
            validate_workspace=False
        )
        place_targets, place_reason = self.calculate_place_targets(
            validate_workspace=False
        )
        visible = []
        if pick_targets is not None:
            pick, pick_pre = pick_targets
            visible.append(
                'pick_pre='
                f'{self._pose_xyz(pick_pre)}, pick={self._pose_xyz(pick)}'
            )
        if place_targets is not None:
            place, place_pre = place_targets
            visible.append(
                'place_pre='
                f'{self._pose_xyz(place_pre)}, place={self._pose_xyz(place)}'
            )
        if not visible:
            response.success = False
            response.message = (
                f'pick=({pick_reason}), place=({place_reason})'
            )
            return response
        response.success = True
        response.message = (
            'PREVIEW CURRENT VIEW ONLY: ' + '; '.join(visible)
        )
        return response

    @staticmethod
    def _pose_xyz(pose):
        return [round(value, 4) for value in (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )]

    def execute_after_stabilization(self):
        if not self.motion_lock.acquire(blocking=False):
            self.publish_status('P&P FAILED: another robot motion is active')
            return
        try:
            self.publish_status(
                'P&P: waiting for complete /joint_states'
            )
            if not self.joint_state_ready.wait(
                self.startup_joint_state_timeout
            ):
                raise RuntimeError(
                    'timed out waiting for complete /joint_states'
                )
            # Let MoveIt's current-state monitor consume the same hardware
            # stream before sending the first planning/execution request.
            if self.stop_event.wait(0.5):
                raise RuntimeError('pick and place stopped')

            self.publish_status(
                'P&P: moving to first observation joint pose'
            )
            self.move_to_observation_joint_pose(
                self.first_observation_joint_angles
            )

            # The first target can be either configured ID. Its base-frame
            # coordinates remain valid after the eye-in-hand camera moves.
            self.publish_status('P&P FIRST VIEW: waiting for one marker')
            first, reason = self.wait_for_first_target()
            if first is None:
                raise RuntimeError(reason)
            first_role, first_targets = first
            second_role = self.other_role(first_role)
            self.publish_status(
                f'P&P FIRST VIEW: {first_role} marker locked'
            )

            self.publish_status(
                'P&P: moving to second observation joint pose'
            )
            self.move_to_observation_joint_pose(
                self.second_observation_joint_angles
            )
            second_targets, reason = self.wait_for_role_target(second_role)
            if second_targets is None:
                raise RuntimeError(reason)

            targets_by_role = {
                first_role: first_targets,
                second_role: second_targets,
            }
            self.publish_status(
                'P&P: sequential pick/place targets locked'
            )
            self.validate_locked_motion_targets(
                targets_by_role['pick'],
                targets_by_role['place'],
            )
            self._execute_pick_and_place_locked(
                targets_by_role['pick'],
                targets_by_role['place'],
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'P&P FAILED: {exc}')
        finally:
            self.motion_lock.release()

    def validate_locked_motion_targets(self, pick_targets, place_targets):
        """Validate all saved motion targets after both observations."""
        named_targets = (
            ('pick', pick_targets[0]),
            ('pick pregrasp', pick_targets[1]),
            ('place', place_targets[0]),
            ('place pregrasp', place_targets[1]),
        )
        for name, pose in named_targets:
            translation = np.array([
                pose.pose.position.x,
                pose.pose.position.y,
                pose.pose.position.z,
            ])
            if not self.in_workspace(translation):
                raise RuntimeError(
                    f'{name} target outside workspace: {translation}'
                )

    def execute_pick_and_place(self, pick_targets, place_targets):
        """Execute already locked targets (kept for direct callers/tests)."""
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self._execute_pick_and_place_locked(
                pick_targets, place_targets
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'P&P FAILED: {exc}')
        finally:
            self.motion_lock.release()

    def _execute_pick_and_place_locked(self, pick_targets, place_targets):
        """Run robot motions while the caller owns ``motion_lock``."""
        grasp, pregrasp = pick_targets
        place, preplace = place_targets
        self.publish_status('P&P PICK: opening gripper')
        self.command_gripper(open_gripper=True)
        self.publish_status('P&P PICK: moving above target')
        self.move_to_pose(
            pregrasp,
            keep_current_orientation=self.pregrasp_test_keep_orientation,
        )
        if self.pregrasp_test_keep_orientation:
            self.publish_status('P&P PICK: aligning vertical')
            self.move_to_pose(pregrasp)
        self.publish_status('P&P PICK: descending')
        try:
            self.move_cartesian_to_pose(grasp)
        except CartesianPlanningError as initial_error:
            grasp = self.move_with_yaw_fallbacks(
                grasp, pregrasp, initial_error
            )
        self.publish_status('P&P PICK: closing gripper')
        self.command_gripper(open_gripper=False)
        self._move_adaptive_lift(
            grasp,
            self.lift_after_pick,
            self.minimum_lift_after_pick,
            'P&P PICK',
        )

        self.publish_status('P&P PLACE: moving above target')
        self.move_to_pose(
            preplace,
            keep_current_orientation=self.place_keep_current_orientation,
        )
        if self.place_keep_current_orientation:
            self.publish_status('P&P PLACE: aligning vertical')
            self.move_to_pose(preplace)
        self.publish_status('P&P PLACE: descending')
        try:
            self.move_cartesian_to_pose(place)
        except CartesianPlanningError as initial_error:
            self.move_place_with_alternate_ik(
                place, preplace, initial_error
            )
        self.publish_status('P&P PLACE: opening gripper')
        self.command_gripper(open_gripper=True)
        self._move_adaptive_lift(
            place,
            self.lift_after_place,
            self.minimum_lift_after_place,
            'P&P PLACE',
        )
        self.publish_status('P&P: completed')

    def move_place_with_alternate_ik(
        self, place, preplace, initial_error
    ):
        """Find and execute a collision-free IK branch for exact placement."""
        if self.motion_backend != 'moveit':
            raise initial_error
        with self.latest_joint_lock:
            current = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
        if current is None:
            raise RuntimeError(
                'Cannot search alternate place IK without joint state'
            ) from initial_error

        self.publish_status(
            'P&P PLACE: collision detected; searching exact-pose '
            'alternate IK branches'
        )
        candidates = []
        seen_preplaces = []
        failures = []
        seeds = alternative_ik_seeds(
            current, self.place_ik_branch_attempts
        )
        for branch_index, seed in enumerate(seeds, start=1):
            preplace_joints = self.solve_collision_free_ik(
                preplace, seed, self.place_ik_timeout
            )
            if preplace_joints is None:
                failures.append(f'branch {branch_index}: preplace IK')
                continue
            if any(
                np.max(np.abs(preplace_joints - known))
                < math.radians(2.0)
                for known in seen_preplaces
            ):
                continue
            seen_preplaces.append(preplace_joints)

            place_joints = self.solve_collision_free_ik(
                place, preplace_joints, self.place_ik_timeout
            )
            if place_joints is None:
                failures.append(f'branch {branch_index}: place IK')
                continue
            response = self.plan_cartesian_from_joint_state(
                place, preplace_joints
            )
            if response is None:
                failures.append(
                    f'branch {branch_index}: no Cartesian response'
                )
                continue
            if response.error_code.val != MoveItErrorCodes.SUCCESS:
                failures.append(
                    f'branch {branch_index}: Cartesian error '
                    f'{response.error_code.val}'
                )
                continue
            if response.fraction < self.place_branch_min_fraction:
                failures.append(
                    f'branch {branch_index}: Cartesian '
                    f'{response.fraction:.3f}'
                )
                continue
            if not response.solution.joint_trajectory.points:
                failures.append(
                    f'branch {branch_index}: empty Cartesian path'
                )
                continue
            endpoint_travel = float(np.sum(np.abs(
                preplace_joints - current
            )))
            cartesian_travel = joint_trajectory_metrics(
                response.solution.joint_trajectory.points
            )[1]
            candidates.append({
                'index': branch_index,
                'preplace_joints': preplace_joints,
                'endpoint_travel': endpoint_travel,
                'cartesian_travel': cartesian_travel,
            })
            self.publish_status(
                f'P&P PLACE: branch {branch_index} is safe '
                f'(Cartesian={response.fraction:.3f}, '
                f'joint travel={math.degrees(endpoint_travel):.1f}deg)'
            )

        if not candidates:
            detail = '; '.join(failures[-8:])
            raise RuntimeError(
                'No collision-free exact-pose place IK branch found; '
                f'initial path: {initial_error}; attempts: {detail}'
            )

        candidates.sort(key=lambda candidate: (
            candidate['endpoint_travel'],
            candidate['cartesian_travel'],
        ))
        descent_failures = []
        for candidate in candidates:
            branch_index = candidate['index']
            self.publish_status(
                f'P&P PLACE: moving to alternate IK branch {branch_index}'
            )
            trajectory, _ = self._plan_moveit_joint_goal(
                JOINT_NAMES, candidate['preplace_joints']
            )
            self._execute_moveit_trajectory(trajectory)
            try:
                self.move_cartesian_to_pose(place)
                self.publish_status(
                    'P&P PLACE: exact-depth descent succeeded with '
                    f'alternate IK branch {branch_index}'
                )
                return
            except CartesianPlanningError as exc:
                descent_failures.append(
                    f'branch {branch_index}: {exc}'
                )
                self.publish_status(
                    f'P&P PLACE: branch {branch_index} changed after '
                    'execution; trying another safe branch'
                )
        raise RuntimeError(
            'Alternate place IK branches failed final live validation: '
            + '; '.join(descent_failures)
        )

    def _move_adaptive_lift(self, pose, requested, minimum, label):
        failures = []
        for distance in lift_distance_candidates(
            requested, minimum, self.lift_search_step
        ):
            target = copy.deepcopy(pose)
            target.pose.position.z += distance
            try:
                self.move_cartesian_to_pose(target)
                self.publish_status(
                    f'{label}: lifted {distance * 100.0:.1f}cm'
                )
                return
            except CartesianPlanningError as exc:
                failures.append(f'{distance:.3f}m: {exc}')
                self.get_logger().warning(
                    f'{label}: lift {distance * 100.0:.1f}cm '
                    'not feasible; trying shorter'
                )
        raise RuntimeError('No safe lift path: ' + '; '.join(failures))


def main(args=None):
    rclpy.init(args=args)
    node = ContainerPickPlaceCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
