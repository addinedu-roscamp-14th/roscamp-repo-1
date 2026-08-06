"""Execute a predeclared container-stack relocation plan."""

from collections import defaultdict, deque
import copy
from dataclasses import dataclass
import json
import math
import threading
import time

from arm.container_pick_coordinator import (
    CartesianPlanningError,
    apply_radial_xy_offset,
    apply_vertical_pick_offsets,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    normalize_quaternion,
    quaternion_to_rpy_degrees,
    rotate_vector,
)
from arm.container_pick_place_coordinator import (
    ContainerPickPlaceCoordinator,
)
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger
from tf2_ros import TransformException


@dataclass(frozen=True)
class StackMove:
    """One fully determined container transfer."""

    pick_id: int
    place_target_id: int
    destination: str


def parse_stack(value, name):
    """Parse a JSON stack ordered as base, bottom, ..., top."""
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError as exc:
            raise ValueError(f'{name} must be a JSON integer list') from exc
    if (
        not isinstance(value, list)
        or not value
        or any(isinstance(item, bool) or not isinstance(item, int)
               for item in value)
    ):
        raise ValueError(f'{name} must be a non-empty integer list')
    if len(set(value)) != len(value):
        raise ValueError(f'{name} must not contain duplicate marker IDs')
    if any(item < 0 for item in value):
        raise ValueError(f'{name} marker IDs must be non-negative')
    return list(value)


def calculate_stack_plan(
    source_stack,
    empty_stack,
    target_container_id,
    final_place_marker_id,
):
    """Calculate every move without relying on later stack inference."""
    source = parse_stack(source_stack, 'source_stack')
    empty = parse_stack(empty_stack, 'empty_stack')
    target = int(target_container_id)
    final_place = int(final_place_marker_id)
    if final_place < 0:
        raise ValueError('final place marker ID must be non-negative')
    if target == source[0]:
        raise ValueError('source stack base marker cannot be picked')
    if target not in source[1:]:
        raise ValueError(
            f'target container ID {target} is not in source_stack'
        )
    if set(source) & set(empty):
        raise ValueError('source_stack and empty_stack IDs must be disjoint')
    if final_place in set(source) | set(empty):
        raise ValueError(
            'final place marker ID must be outside both declared stacks'
        )

    moves = []
    while source[-1] != target:
        blocker = source.pop()
        moves.append(StackMove(blocker, empty[-1], 'empty'))
        empty.append(blocker)
    source.pop()
    moves.append(StackMove(target, final_place, 'final_place'))
    return moves


def apply_completed_move(source_state, empty_state, move):
    """Update declared states after one physical move has completed."""
    if not source_state or source_state[-1] != move.pick_id:
        raise RuntimeError(
            f'source state top is not completed pick ID {move.pick_id}'
        )
    source_state.pop()
    if move.destination == 'empty':
        if not empty_state or empty_state[-1] != move.place_target_id:
            raise RuntimeError(
                'empty state top does not match completed place target '
                f'ID {move.place_target_id}'
            )
        empty_state.append(move.pick_id)


def marker_normal_angle_deg(quaternion):
    """Return the unsigned marker-normal angle from base Z."""
    normal = rotate_vector(quaternion, [0.0, 0.0, 1.0])
    cosine = abs(float(normal[2])) / max(float(np.linalg.norm(normal)), 1e-12)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


class StackPickPlaceCoordinator(ContainerPickPlaceCoordinator):
    """Execute a declared source/empty stack transfer plan."""

    def _declare_parameters(self):
        super()._declare_parameters()
        self.declare_parameter('source_stack', '[1, 2, 3, 4]')
        self.declare_parameter('empty_stack', '[5]')
        self.declare_parameter('target_container_id', 2)
        self.declare_parameter('final_place_marker_id', 6)
        self.declare_parameter(
            'stack_marker_frame_prefix', 'arm/stack_marker_'
        )
        self.declare_parameter('top_face_max_angle_deg', 30.0)
        self.declare_parameter('observation_settle_sec', 1.0)
        self.declare_parameter('observation_capture_sec', 2.0)
        self.declare_parameter('pending_detection_timeout_sec', 2.0)
        self.declare_parameter('tf_failure_log_period_sec', 1.0)

    def __init__(self):
        super().__init__()
        self.source_stack = parse_stack(
            self.get_parameter('source_stack').value, 'source_stack'
        )
        self.empty_stack = parse_stack(
            self.get_parameter('empty_stack').value, 'empty_stack'
        )
        self.target_container_id = int(
            self.get_parameter('target_container_id').value
        )
        self.final_place_marker_id = int(
            self.get_parameter('final_place_marker_id').value
        )
        self.stack_plan = calculate_stack_plan(
            self.source_stack,
            self.empty_stack,
            self.target_container_id,
            self.final_place_marker_id,
        )
        self.stack_frame_prefix = str(
            self.get_parameter('stack_marker_frame_prefix').value
        )
        self.top_face_max = float(
            self.get_parameter('top_face_max_angle_deg').value
        )
        self.observation_settle = float(
            self.get_parameter('observation_settle_sec').value
        )
        self.observation_capture = float(
            self.get_parameter('observation_capture_sec').value
        )
        self.pending_detection_timeout = float(
            self.get_parameter('pending_detection_timeout_sec').value
        )
        self.tf_failure_log_period = float(
            self.get_parameter('tf_failure_log_period_sec').value
        )
        if not 0.0 <= self.top_face_max < 90.0:
            raise ValueError('top_face_max_angle_deg must be in [0, 90)')
        if self.observation_settle < 0.0:
            raise ValueError('observation_settle_sec must be non-negative')
        if self.observation_capture <= 0.0:
            raise ValueError('observation_capture_sec must be positive')
        if self.pending_detection_timeout <= 0.0:
            raise ValueError(
                'pending_detection_timeout_sec must be positive'
            )

        self.capture_lock = threading.Lock()
        self.capture_enabled = False
        self.pending_detections = defaultdict(deque)
        self.marker_histories = defaultdict(
            lambda: deque(maxlen=max(100, self.minimum_samples * 3))
        )
        self.tf_failure_reports = {}
        self.capture_frames = 0
        self.capture_pose_detections = 0
        self.create_subscription(
            String,
            '/arm/gripper_camera/stack_detections',
            self.on_stack_detections,
            10,
        )
        control_qos = QoSProfile(depth=1)
        control_qos.reliability = ReliabilityPolicy.RELIABLE
        control_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.detection_control = self.create_publisher(
            Bool, '/arm/stack_detection_enabled', control_qos
        )
        self.create_timer(0.05, self.update_stack_tracking)
        self.create_service(
            Trigger,
            '/arm/pick_place_relocation2',
            self.start_stack_relocation,
        )
        self.set_detection_enabled(False)
        self.publish_status(
            'STACK RELOCATION ready: '
            f'source={self.source_stack}, empty={self.empty_stack}, '
            f'target={self.target_container_id}, '
            f'final_place={self.final_place_marker_id}, '
            f'plan={self.format_plan(self.stack_plan)}'
        )

    # Disable the inherited two-role marker tracking timers.
    def update_tracking(self):
        """Ignore the inherited fixed pick-marker stream."""

    def update_place_tracking(self):
        """Ignore the inherited fixed place-marker stream."""

    @staticmethod
    def format_plan(plan):
        """Format a plan for status output."""
        return ' -> '.join(
            f'{move.pick_id}=>{move.place_target_id}({move.destination})'
            for move in plan
        )

    def set_detection_enabled(self, enabled):
        """Enable ArUco work only during a stationary capture window."""
        message = Bool()
        message.data = bool(enabled)
        self.detection_control.publish(message)

    def on_stack_detections(self, message):
        """Queue detections only while the robot is stationary."""
        try:
            payload = json.loads(message.data)
            stamp_ns = int(payload['stamp_ns'])
            detections = payload['detections']
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'invalid stack detections: {exc}')
            return
        receipt = time.monotonic()
        with self.capture_lock:
            if not self.capture_enabled:
                return
            self.capture_frames += 1
            self.capture_pose_detections += len(detections)
            for detection in detections:
                try:
                    marker_id = int(detection['id'])
                    marker_frame = str(detection['frame'])
                    area = float(detection['area_px'])
                except (KeyError, TypeError, ValueError):
                    continue
                queue = self.pending_detections[marker_frame]
                if queue and queue[-1]['stamp_ns'] == stamp_ns:
                    continue
                queue.append({
                    'marker_id': marker_id,
                    'frame': marker_frame,
                    'stamp_ns': stamp_ns,
                    'area_px': area,
                    'receipt': receipt,
                })

    def update_stack_tracking(self):
        """Convert queued detections using their exact-time TF."""
        with self.capture_lock:
            tracking_keys = list(self.pending_detections)
        for tracking_key in tracking_keys:
            for _ in range(10):
                with self.capture_lock:
                    queue = self.pending_detections[tracking_key]
                    detection = copy.deepcopy(queue[0]) if queue else None
                if detection is None:
                    break
                age = time.monotonic() - detection['receipt']
                if age > self.pending_detection_timeout:
                    self.pop_detection(
                        tracking_key, detection['stamp_ns']
                    )
                    self.report_tf_failure(
                        tracking_key,
                        'timeout',
                        f'discarded after {age:.2f}s',
                    )
                    continue
                try:
                    transform = self.buffer.lookup_transform(
                        self.base_frame,
                        detection['frame'],
                        Time(nanoseconds=detection['stamp_ns']),
                    )
                except TransformException as exc:
                    self.report_tf_failure(
                        tracking_key, 'waiting', str(exc)
                    )
                    break
                sample = (
                    detection['stamp_ns'],
                    np.array([
                        transform.transform.translation.x,
                        transform.transform.translation.y,
                        transform.transform.translation.z,
                    ]),
                    normalize_quaternion([
                        transform.transform.rotation.x,
                        transform.transform.rotation.y,
                        transform.transform.rotation.z,
                        transform.transform.rotation.w,
                    ]),
                    detection['area_px'],
                    detection['receipt'],
                    detection['marker_id'],
                    detection['frame'],
                )
                self.pop_detection(tracking_key, detection['stamp_ns'])
                with self.capture_lock:
                    self.marker_histories[tracking_key].append(sample)
                    self.tf_failure_reports.pop(
                        (tracking_key, 'waiting'), None
                    )

    def pop_detection(self, tracking_key, stamp_ns):
        """Remove a processed queue head."""
        with self.capture_lock:
            queue = self.pending_detections[tracking_key]
            if queue and queue[0]['stamp_ns'] == stamp_ns:
                queue.popleft()

    def report_tf_failure(self, tracking_key, kind, detail):
        """Log TF failures at a bounded rate."""
        key = (tracking_key, kind)
        now = time.monotonic()
        with self.capture_lock:
            previous = self.tf_failure_reports.get(key, 0.0)
            should_log = now - previous >= self.tf_failure_log_period
            if should_log:
                self.tf_failure_reports[key] = now
        if should_log:
            self.get_logger().warning(
                f'STACK TF {kind}: marker={tracking_key}: {detail}'
            )

    def stable_observation(self, tracking_key, cutoff):
        """Return a stable top-face marker observation."""
        with self.capture_lock:
            samples = [
                sample for sample in self.marker_histories[tracking_key]
                if sample[4] > cutoff
            ][-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None
        translations = np.asarray([sample[1] for sample in samples])
        if float(np.max(np.std(translations, axis=0))) > (
            self.max_translation_std
        ):
            return None
        reference = samples[-1][2]
        rotations = [
            rotation if np.dot(reference, rotation) >= 0.0 else -rotation
            for _, _, rotation, _, _, _, _ in samples
        ]
        rotation = normalize_quaternion(np.mean(rotations, axis=0))
        spread = max(
            2.0 * math.degrees(math.acos(min(
                1.0, abs(float(np.dot(rotation, candidate)))
            )))
            for candidate in rotations
        )
        if spread > self.max_rotation_spread:
            return None
        angle = marker_normal_angle_deg(rotation)
        if angle > self.top_face_max:
            return None
        return {
            'id': samples[-1][5],
            'frame': samples[-1][6],
            'translation': np.mean(translations, axis=0),
            'rotation': rotation,
            'area_px': float(np.mean([sample[3] for sample in samples])),
            'normal_angle_deg': angle,
        }

    def capture_scene(self):
        """Capture stable top markers after motion and settling."""
        self.set_detection_enabled(False)
        self.publish_status(
            'STACK OBSERVE: motion complete; settling '
            f'{self.observation_settle:.2f}s'
        )
        if self.stop_event.wait(self.observation_settle):
            raise RuntimeError('stack relocation stopped')
        with self.capture_lock:
            self.pending_detections.clear()
            self.marker_histories.clear()
            self.tf_failure_reports.clear()
            self.capture_frames = 0
            self.capture_pose_detections = 0
            cutoff = time.monotonic()
            self.capture_enabled = True
        self.set_detection_enabled(True)
        captured = {}
        deadline = time.monotonic() + self.observation_capture
        try:
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.1):
                    raise RuntimeError('stack relocation stopped')
                with self.capture_lock:
                    tracking_keys = list(self.marker_histories)
                for tracking_key in tracking_keys:
                    observation = self.stable_observation(
                        tracking_key, cutoff
                    )
                    if observation is not None:
                        captured[tracking_key] = observation
        finally:
            self.set_detection_enabled(False)
            with self.capture_lock:
                self.capture_enabled = False
                frames = self.capture_frames
                detections = self.capture_pose_detections
                samples = {
                    key: len(value)
                    for key, value in self.marker_histories.items()
                }
        top_summary = [
            (item['id'], round(item['area_px']))
            for item in captured.values()
        ]
        self.publish_status(
            'STACK OBSERVE SUMMARY: '
            f'frames={frames}, detections={detections}, '
            f'samples={samples}, '
            f'top={top_summary}'
        )
        return list(captured.values())

    @staticmethod
    def merge_views(*views):
        """Merge observations, keeping the largest view of each marker ID."""
        merged = {}
        for observation in (
            item for view in views for item in view
        ):
            marker_id = observation['id']
            if (
                marker_id not in merged
                or observation['area_px'] > merged[marker_id]['area_px']
            ):
                merged[marker_id] = observation
        return list(merged.values())

    def observe_move_targets(self, move):
        """Observe only the two IDs required by a precomputed move."""
        self.set_detection_enabled(False)
        self.publish_status(
            f'STACK STEP: observing pick={move.pick_id}, '
            f'place_target={move.place_target_id}'
        )
        self.move_to_observation_joint_pose(
            self.first_observation_joint_angles
        )
        first = self.capture_scene()
        self.move_to_observation_joint_pose(
            self.second_observation_joint_angles
        )
        second = self.capture_scene()
        observations = self.merge_views(first, second)
        by_id = {item['id']: item for item in observations}
        missing = [
            marker_id
            for marker_id in (move.pick_id, move.place_target_id)
            if marker_id not in by_id
        ]
        if missing:
            visible = sorted(by_id)
            raise RuntimeError(
                f'planned top marker IDs not stabilized: {missing}; '
                f'visible top IDs={visible}'
            )
        return by_id[move.pick_id], by_id[move.place_target_id]

    def build_pick_targets(self, observation):
        """Build grasp/pregrasp targets from one top marker."""
        marker_translation = observation['translation']
        marker_rotation = observation['rotation']
        mode = self.grasp_orientation_mode
        if self.use_marker_rotation and mode == 'fixed':
            mode = 'marker_full'
        if mode == 'marker_full':
            translation, rotation = compose_pose(
                marker_translation,
                marker_rotation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        elif mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            translation, rotation, yaw_delta = compose_yaw_follow_pose(
                marker_translation,
                self.grasp_offset,
                self.grasp_rpy,
                marker_yaw,
                self.reference_marker_yaw,
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                raise RuntimeError(
                    f'pick yaw delta {yaw_delta:.2f}deg exceeds limit'
                )
        else:
            translation, rotation = compose_fixed_base_pose(
                marker_translation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        translation = apply_radial_xy_offset(
            translation, self.target_radial_offset
        )
        grasp_xyz, pregrasp_xyz = apply_vertical_pick_offsets(
            translation, self.pregrasp_lift, self.grasp_extra_depth
        )
        stamp = self.get_clock().now().to_msg()
        return (
            self.make_pose(grasp_xyz, rotation, stamp),
            self.make_pose(pregrasp_xyz, rotation, stamp),
        )

    def build_place_targets(self, observation):
        """Build place/preplace targets from the current stack top."""
        marker_translation = observation['translation']
        marker_rotation = observation['rotation']
        if self.place_orientation_mode == 'marker_full':
            translation, rotation = compose_pose(
                marker_translation,
                marker_rotation,
                self.place_offset,
                self.place_rotation,
            )
        elif self.place_orientation_mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            translation, rotation, yaw_delta = compose_yaw_follow_pose(
                marker_translation,
                self.place_offset,
                self.place_rpy,
                marker_yaw,
                self.place_reference_yaw,
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                raise RuntimeError(
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
        place_xyz, preplace_xyz = apply_vertical_pick_offsets(
            translation,
            self.place_pregrasp_lift,
            self.place_extra_depth,
        )
        stamp = self.get_clock().now().to_msg()
        return (
            self.make_pose(place_xyz, rotation, stamp),
            self.make_pose(preplace_xyz, rotation, stamp),
        )

    def align_to_pick(self, pick_targets):
        """Approach by position first, then apply the grasp orientation."""
        _grasp, pregrasp = pick_targets
        self.publish_status('STACK PICK: moving above planned marker')
        self.move_to_pose(pregrasp, keep_current_orientation=True)
        self.publish_status('STACK PICK: applying grasp orientation')
        self.move_to_pose(pregrasp)

    def execute_from_aligned_pregrasp(self, pick_targets, place_targets):
        """Execute without repeating the already completed pick approach."""
        grasp, pregrasp = pick_targets
        place, preplace = place_targets
        self.publish_status('STACK PICK: descending')
        try:
            self.move_cartesian_to_pose(grasp)
        except CartesianPlanningError as initial_error:
            grasp = self.move_with_yaw_fallbacks(
                grasp, pregrasp, initial_error
            )
        self.publish_status('STACK PICK: closing gripper')
        self.command_gripper(open_gripper=False)
        self._move_adaptive_lift(
            grasp,
            self.lift_after_pick,
            self.minimum_lift_after_pick,
            'STACK PICK',
        )

        self.publish_status('STACK PLACE: moving above planned target')
        self.move_to_pose(
            preplace,
            keep_current_orientation=self.place_keep_current_orientation,
        )
        if self.place_keep_current_orientation:
            self.publish_status('STACK PLACE: aligning orientation')
            self.move_to_pose(preplace)
        self.publish_status('STACK PLACE: descending')
        try:
            self.move_cartesian_to_pose(place)
        except CartesianPlanningError as initial_error:
            self.move_place_with_alternate_ik(
                place, preplace, initial_error
            )
        self.publish_status('STACK PLACE: opening gripper')
        self.command_gripper(open_gripper=True)
        self._move_adaptive_lift(
            place,
            self.lift_after_place,
            self.minimum_lift_after_place,
            'STACK PLACE',
        )

    def start_stack_relocation(self, request, response):
        """Start the precomputed stack relocation thread."""
        result = super().start_pick_and_place(request, response)
        if result.success:
            result.message = (
                'Stack relocation accepted: '
                + self.format_plan(self.stack_plan)
            )
        return result

    def execute_after_stabilization(self):
        """Observe and execute every move in the precomputed plan."""
        if not self.motion_lock.acquire(blocking=False):
            self.publish_status(
                'STACK RELOCATION FAILED: another motion is active'
            )
            return
        source_state = list(self.source_stack)
        empty_state = list(self.empty_stack)
        completed = False
        try:
            if not self.joint_state_ready.wait(
                self.startup_joint_state_timeout
            ):
                raise RuntimeError(
                    'timed out waiting for complete /joint_states'
                )
            self.publish_status(
                'STACK PLAN LOCKED: ' + self.format_plan(self.stack_plan)
            )
            for index, move in enumerate(self.stack_plan, start=1):
                self.publish_status(
                    f'STACK MOVE {index}/{len(self.stack_plan)}: '
                    f'pick={move.pick_id}, '
                    f'place_target={move.place_target_id}, '
                    f'destination={move.destination}'
                )
                pick_observation, place_observation = (
                    self.observe_move_targets(move)
                )
                pick_targets = self.build_pick_targets(pick_observation)
                place_targets = self.build_place_targets(place_observation)
                self.validate_locked_motion_targets(
                    pick_targets, place_targets
                )
                self.publish_status('STACK PICK: opening gripper')
                self.command_gripper(open_gripper=True)
                self.align_to_pick(pick_targets)
                self.execute_from_aligned_pregrasp(
                    pick_targets, place_targets
                )
                apply_completed_move(source_state, empty_state, move)
                self.publish_status(
                    f'STACK MOVE {index}/{len(self.stack_plan)} completed'
                )
            completed = True
            self.publish_status('STACK RELOCATION completed')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'STACK RELOCATION FAILED: {exc}')
        finally:
            self.set_detection_enabled(False)
            state_label = 'FINAL' if completed else 'AT STOP'
            self.publish_status(
                f'STACK STATE {state_label}: '
                f'Stack_A={source_state}, Empty_Stack={empty_state}'
            )
            self.motion_lock.release()


def main(args=None):
    """Run the stack relocation coordinator."""
    rclpy.init(args=args)
    node = None
    try:
        node = StackPickPlaceCoordinator()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
