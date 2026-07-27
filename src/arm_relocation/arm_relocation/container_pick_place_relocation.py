"""Relocate blocking containers until the commanded side marker is reached."""

from collections import defaultdict, deque
import copy
import json
import math
import threading
import time

from arm.container_pick_coordinator import (
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
from rclpy.time import Time
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import TransformException


def marker_normal_angle_from_vertical_deg(quaternion):
    """Return the unsigned angle between marker +Z and base vertical."""
    normal = rotate_vector(quaternion, [0.0, 0.0, 1.0])
    cosine = abs(float(normal[2])) / max(float(np.linalg.norm(normal)), 1e-12)
    return math.degrees(math.acos(max(-1.0, min(1.0, cosine))))


def classify_marker_face(quaternion, top_max_deg, side_min_deg):
    """Classify a marker as top, side, or tilted from its base-frame normal."""
    if not 0.0 <= top_max_deg < side_min_deg <= 90.0:
        raise ValueError(
            'face thresholds must satisfy 0 <= top < side <= 90'
        )
    angle = marker_normal_angle_from_vertical_deg(quaternion)
    if angle <= top_max_deg:
        return 'top', angle
    if angle >= side_min_deg:
        return 'side', angle
    return 'tilted', angle


def select_topmost_stacked_marker(
    observations,
    side_pick_observation,
    xy_tolerance_m,
    min_height_m=0.0,
    excluded_ids=(),
):
    """Select the highest top marker in the side pick marker's XY stack."""
    if xy_tolerance_m <= 0.0:
        raise ValueError('stack XY tolerance must be positive')
    if min_height_m < 0.0:
        raise ValueError('minimum stack height must be non-negative')
    pick_xyz = np.asarray(side_pick_observation['translation'])
    excluded = set(excluded_ids)
    candidates = []
    for observation in observations:
        marker_xyz = np.asarray(observation['translation'])
        xy_distance = float(np.linalg.norm(marker_xyz[:2] - pick_xyz[:2]))
        height = float(marker_xyz[2] - pick_xyz[2])
        if (
            observation['face'] == 'top'
            and observation['id'] not in excluded
            and xy_distance <= xy_tolerance_m
            and height >= min_height_m
        ):
            candidate = copy.deepcopy(observation)
            candidate['stack_xy_distance_m'] = xy_distance
            candidate['height_above_pick_m'] = height
            candidates.append(candidate)
    if not candidates:
        return None
    return max(
        candidates,
        key=lambda observation: (
            observation['translation'][2],
            observation['area_px'],
        ),
    )


def relocation_destination(observed_id, commanded_id):
    """Choose the final slot for a newly inspected container."""
    return 'place' if observed_id == commanded_id else 'empty'


class ContainerPickPlaceRelocation(ContainerPickPlaceCoordinator):
    """Add three-role observation and blocker relocation to existing motion."""

    def _declare_parameters(self):
        super()._declare_parameters()
        self.declare_parameter('pick_marker_id', 1)
        self.declare_parameter('place_marker_id', 2)
        self.declare_parameter('empty_marker_id', 3)
        self.declare_parameter(
            'marker_frame_prefix', 'arm/relocation_marker_'
        )
        self.declare_parameter('top_face_max_angle_deg', 30.0)
        self.declare_parameter('side_face_min_angle_deg', 60.0)
        self.declare_parameter('require_pick_side', True)
        self.declare_parameter('require_destination_top', True)
        self.declare_parameter('observation_capture_sec', 3.0)
        self.declare_parameter('stack_xy_tolerance_m', 0.08)
        self.declare_parameter('stack_min_height_above_pick_m', 0.0)
        self.declare_parameter('max_relocation_cycles', 3)
        self.declare_parameter('empty_stack_step_m', 0.0)

    def __init__(self):
        super().__init__()
        self.pick_marker_id = int(
            self.get_parameter('pick_marker_id').value
        )
        self.place_marker_id = int(
            self.get_parameter('place_marker_id').value
        )
        self.empty_marker_id = int(
            self.get_parameter('empty_marker_id').value
        )
        role_ids = (
            self.pick_marker_id,
            self.place_marker_id,
            self.empty_marker_id,
        )
        if len(set(role_ids)) != 3:
            raise ValueError('pick/place/empty marker IDs must be different')
        self.frame_prefix = str(
            self.get_parameter('marker_frame_prefix').value
        )
        self.top_face_max = float(
            self.get_parameter('top_face_max_angle_deg').value
        )
        self.side_face_min = float(
            self.get_parameter('side_face_min_angle_deg').value
        )
        classify_marker_face(
            [0.0, 0.0, 0.0, 1.0],
            self.top_face_max,
            self.side_face_min,
        )
        self.require_pick_side = bool(
            self.get_parameter('require_pick_side').value
        )
        self.require_destination_top = bool(
            self.get_parameter('require_destination_top').value
        )
        self.observation_capture = float(
            self.get_parameter('observation_capture_sec').value
        )
        self.stack_xy_tolerance = float(
            self.get_parameter('stack_xy_tolerance_m').value
        )
        self.stack_min_height = float(
            self.get_parameter('stack_min_height_above_pick_m').value
        )
        self.max_relocation_cycles = int(
            self.get_parameter('max_relocation_cycles').value
        )
        self.empty_stack_step = float(
            self.get_parameter('empty_stack_step_m').value
        )
        if self.observation_capture <= 0.0:
            raise ValueError('observation_capture_sec must be positive')
        if self.stack_xy_tolerance <= 0.0:
            raise ValueError('stack_xy_tolerance_m must be positive')
        if self.stack_min_height < 0.0:
            raise ValueError(
                'stack_min_height_above_pick_m must be non-negative'
            )
        if self.max_relocation_cycles < 1:
            raise ValueError('max_relocation_cycles must be positive')
        if self.empty_stack_step < 0.0:
            raise ValueError('empty_stack_step_m must be non-negative')

        self.detection_lock = threading.Lock()
        self.latest_detections = {}
        self.marker_histories = defaultdict(
            lambda: deque(maxlen=max(100, self.minimum_samples * 3))
        )
        self.last_recorded_stamps = {}
        self.create_subscription(
            String,
            '/arm/gripper_camera/relocation_detections',
            self.on_detections,
            10,
        )
        self.create_timer(0.05, self.update_relocation_tracking)
        self.create_service(
            Trigger,
            '/arm/pick_place_relocation',
            self.start_relocation,
        )
        self.publish_status(
            'RELOCATION ready: '
            f'pick={self.pick_marker_id}, place={self.place_marker_id}, '
            f'empty={self.empty_marker_id}'
        )

    # The inherited two-frame timers are not used by this three-role node.
    def update_tracking(self):
        pass

    def update_place_tracking(self):
        pass

    def on_detections(self, message):
        """Cache one image's valid IDs and areas."""
        try:
            payload = json.loads(message.data)
            stamp_ns = int(payload['stamp_ns'])
            detections = payload['detections']
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'invalid detection metadata: {exc}')
            return
        receipt = time.monotonic()
        with self.detection_lock:
            for detection in detections:
                try:
                    marker_id = int(detection['id'])
                    area = float(detection['area_px'])
                except (KeyError, TypeError, ValueError):
                    continue
                self.latest_detections[marker_id] = {
                    'stamp_ns': stamp_ns,
                    'area_px': area,
                    'receipt': receipt,
                }

    def update_relocation_tracking(self):
        """Transform fresh detections into base-frame marker histories."""
        with self.detection_lock:
            detections = copy.deepcopy(self.latest_detections)
        for marker_id, detection in detections.items():
            stamp_ns = detection['stamp_ns']
            if self.last_recorded_stamps.get(marker_id) == stamp_ns:
                continue
            try:
                transform = self.buffer.lookup_transform(
                    self.base_frame,
                    f'{self.frame_prefix}{marker_id}',
                    Time(),
                )
            except TransformException:
                continue
            transform_stamp = (
                transform.header.stamp.sec * 1_000_000_000
                + transform.header.stamp.nanosec
            )
            if transform_stamp != stamp_ns:
                continue
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
            sample = (
                stamp_ns,
                translation,
                rotation,
                detection['area_px'],
                detection['receipt'],
            )
            with self.detection_lock:
                self.marker_histories[marker_id].append(sample)
                self.last_recorded_stamps[marker_id] = stamp_ns

    def clear_relocation_histories(self):
        with self.detection_lock:
            self.marker_histories.clear()
            self.last_recorded_stamps.clear()

    def stable_observation(self, marker_id, after_receipt=0.0):
        """Return a filtered marker pose, face class, and mean image area."""
        with self.detection_lock:
            samples = [
                sample for sample in self.marker_histories[marker_id]
                if sample[4] > after_receipt
            ][-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None
        translations = np.asarray([sample[1] for sample in samples])
        if float(np.max(np.std(translations, axis=0))) > (
            self.max_translation_std
        ):
            return None
        reference = samples[-1][2]
        aligned = [
            rotation if np.dot(reference, rotation) >= 0.0 else -rotation
            for _, _, rotation, _, _ in samples
        ]
        rotation = normalize_quaternion(np.mean(aligned, axis=0))
        spread = max(
            math.degrees(2.0 * math.acos(min(
                1.0, abs(float(np.dot(rotation, value)))
            )))
            for value in aligned
        )
        if spread > self.max_rotation_spread:
            return None
        face, angle = classify_marker_face(
            rotation, self.top_face_max, self.side_face_min
        )
        return {
            'id': marker_id,
            'translation': np.mean(translations, axis=0),
            'rotation': rotation,
            'area_px': float(np.mean([sample[3] for sample in samples])),
            'face': face,
            'normal_angle_deg': angle,
        }

    def capture_scene(self):
        """Capture every stable marker exposed from the current view."""
        self.clear_relocation_histories()
        cutoff = time.monotonic()
        deadline = time.monotonic() + self.observation_capture
        captured = {}
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.1):
                raise RuntimeError('relocation stopped')
            with self.detection_lock:
                marker_ids = list(self.latest_detections)
            for marker_id in marker_ids:
                observation = self.stable_observation(
                    marker_id, after_receipt=cutoff
                )
                if observation is not None:
                    captured[marker_id] = observation
        return list(captured.values())

    @staticmethod
    def merge_observations(*views):
        """Merge views while retaining different faces of the same ID."""
        merged = []
        for observation in (
            item for view in views for item in view
        ):
            duplicate_index = None
            for index, known in enumerate(merged):
                if (
                    known['id'] == observation['id']
                    and known['face'] == observation['face']
                    and np.linalg.norm(
                        known['translation'] - observation['translation']
                    ) <= 0.03
                ):
                    duplicate_index = index
                    break
            if duplicate_index is None:
                merged.append(observation)
            elif (
                observation['area_px']
                > merged[duplicate_index]['area_px']
            ):
                merged[duplicate_index] = observation
        return merged

    @staticmethod
    def find_role_observation(observations, marker_id, required_face):
        """Find one role ID with the required face classification."""
        candidates = [
            observation for observation in observations
            if (
                observation['id'] == marker_id
                and (
                    required_face is None
                    or observation['face'] == required_face
                )
            )
        ]
        if not candidates:
            return None
        return max(candidates, key=lambda item: item['area_px'])

    def observe_scene(self, require_destinations):
        """Visit both observation poses and classify every visible marker."""
        self.publish_status('RELOCATION: moving to first observation pose')
        self.move_to_observation_joint_pose(
            self.first_observation_joint_angles
        )
        first_view = self.capture_scene()
        self.publish_status(
            'RELOCATION FIRST VIEW: '
            f'{[(item["id"], item["face"]) for item in first_view]}'
        )

        self.publish_status('RELOCATION: moving to second observation pose')
        self.move_to_observation_joint_pose(
            self.second_observation_joint_angles
        )
        second_view = self.capture_scene()
        self.publish_status(
            'RELOCATION SECOND VIEW: '
            f'{[(item["id"], item["face"]) for item in second_view]}'
        )
        observations = self.merge_observations(first_view, second_view)

        pick_face = 'side' if self.require_pick_side else None
        pick = self.find_role_observation(
            observations, self.pick_marker_id, pick_face
        )
        if pick is None:
            raise RuntimeError(
                f'could not stabilize {pick_face or "any"} pick marker '
                f'ID {self.pick_marker_id}'
            )
        roles = {'pick': pick}
        if require_destinations:
            destination_face = (
                'top' if self.require_destination_top else None
            )
            for role, marker_id in (
                ('place', self.place_marker_id),
                ('empty', self.empty_marker_id),
            ):
                marker = self.find_role_observation(
                    observations, marker_id, destination_face
                )
                if marker is None:
                    raise RuntimeError(
                        f'could not stabilize '
                        f'{destination_face or "any"} {role} marker '
                        f'ID {marker_id}'
                    )
                roles[role] = marker
        return observations, roles

    def build_pick_targets(self, observation):
        """Apply container_pick_place.yaml grasp offsets to a top marker."""
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
                    f'marker yaw delta {yaw_delta:.2f}deg exceeds limit'
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

    def build_place_targets(self, observation, stack_height=0.0):
        """Apply the existing place calibration to one destination marker."""
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
        translation[2] += stack_height
        place_xyz, preplace_xyz = apply_vertical_pick_offsets(
            translation, self.place_pregrasp_lift, self.place_extra_depth
        )
        stamp = self.get_clock().now().to_msg()
        return (
            self.make_pose(place_xyz, rotation, stamp),
            self.make_pose(preplace_xyz, rotation, stamp),
        )

    def align_to_top_marker(self, pick_targets):
        """Move to the selected top marker, then apply the grasp RPY."""
        _grasp, pregrasp = pick_targets
        self.publish_status(
            'RELOCATION TOP: moving to selected marker coordinates'
        )
        self.move_to_pose(pregrasp, keep_current_orientation=True)
        self.publish_status(
            'RELOCATION TOP: applying grasp_offset_rpy_deg'
        )
        self.move_to_pose(pregrasp)

    def start_relocation(self, request, response):
        return super().start_pick_and_place(request, response)

    def execute_after_stabilization(self):
        """Run observation, inspection, and repeated blocker relocation."""
        if not self.motion_lock.acquire(blocking=False):
            self.publish_status(
                'RELOCATION FAILED: another robot motion is active'
            )
            return
        try:
            if not self.joint_state_ready.wait(
                self.startup_joint_state_timeout
            ):
                raise RuntimeError(
                    'timed out waiting for complete /joint_states'
                )
            observations, roles = self.observe_scene(
                require_destinations=True
            )
            place_targets = self.build_place_targets(
                roles['place']
            )
            empty_observation = roles['empty']

            for cycle in range(self.max_relocation_cycles):
                if cycle > 0:
                    observations, roles = self.observe_scene(
                        require_destinations=False
                    )
                self.publish_status(
                    f'RELOCATION CYCLE {cycle + 1}: selecting stack top'
                )
                selected = select_topmost_stacked_marker(
                    observations,
                    roles['pick'],
                    self.stack_xy_tolerance,
                    self.stack_min_height,
                    excluded_ids=(
                        self.place_marker_id,
                        self.empty_marker_id,
                    ),
                )
                if selected is None:
                    raise RuntimeError(
                        'no top marker found above side pick marker; '
                        f'xy tolerance={self.stack_xy_tolerance:.3f}m, '
                        f'min height={self.stack_min_height:.3f}m'
                    )
                destination = relocation_destination(
                    selected['id'], self.pick_marker_id
                )
                self.publish_status(
                    f'RELOCATION: topmost marker={selected["id"]} '
                    f'z={selected["translation"][2]:.3f}m, '
                    f'xy from pick={selected["stack_xy_distance_m"]:.3f}m '
                    f'-> {destination}'
                )
                pick_targets = self.build_pick_targets(selected)
                self.align_to_top_marker(pick_targets)
                if destination == 'place':
                    selected_place_targets = place_targets
                else:
                    selected_place_targets = self.build_place_targets(
                        empty_observation,
                        stack_height=cycle * self.empty_stack_step,
                    )
                self.validate_locked_motion_targets(
                    pick_targets, selected_place_targets
                )
                self._execute_pick_and_place_locked(
                    pick_targets, selected_place_targets
                )
                if destination == 'place':
                    self.publish_status(
                        'RELOCATION: commanded container placed; complete'
                    )
                    return
            raise RuntimeError(
                'commanded marker was not reached within '
                f'{self.max_relocation_cycles} cycles'
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'RELOCATION FAILED: {exc}')
        finally:
            self.motion_lock.release()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ContainerPickPlaceRelocation()
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
