"""Execute one guarded ArUco pick-and-place sequence."""

from collections import deque
import copy
import math
import threading
import time

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from std_srvs.srv import Trigger
from tf2_ros import TransformException

from .container_pick_coordinator import (
    CartesianPlanningError,
    ContainerPickCoordinator,
    apply_vertical_pick_offsets,
    compose_fixed_base_pose,
    compose_pose,
    compose_yaw_follow_pose,
    lift_distance_candidates,
    normalize_quaternion,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
    wrap_degrees,
)


class ContainerPickPlaceCoordinator(ContainerPickCoordinator):
    """Lock two marker poses, pick ID 1, then place at ID 0."""

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

        # The inherited marker_frame/history are the pick marker (ID 1).
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
        self.publish_status(
            'P&P ready: pick='
            f'{self.marker_frame}, place={self.place_marker_frame}'
        )

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

    def wait_for_both_targets(self):
        """Require fresh, stable observations of ID 1 and ID 0."""
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
            pick_targets, pick_reason = self.calculate_targets()
            place_targets, place_reason = self.calculate_place_targets()
            if pick_targets is not None and place_targets is not None:
                return (pick_targets, place_targets), 'targets locked'
            reasons = (pick_reason, place_reason)
            report = f'pick=({pick_reason}), place=({place_reason})'
            now = time.monotonic()
            if report != last_report or now - last_report_time >= 1.0:
                self.publish_status(f'P&P waiting: {report}')
                last_report = report
                last_report_time = now
        return None, (
            'markers did not stabilize: '
            f'pick=({reasons[0]}), place=({reasons[1]})'
        )

    def start_pick_and_place(self, _request, response):
        if not self.execute_motion:
            response.success = False
            response.message = 'Start the MoveIt pick/place launch'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full motion or offsets are not enabled'
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
        response.message = 'Pick and place accepted; locking both markers'
        return response

    def preview_pick_and_place(self, _request, response):
        pick_targets, pick_reason = self.calculate_targets(
            validate_workspace=False
        )
        place_targets, place_reason = self.calculate_place_targets(
            validate_workspace=False
        )
        if pick_targets is None or place_targets is None:
            response.success = False
            response.message = (
                f'pick=({pick_reason}), place=({place_reason})'
            )
            return response
        pick, pick_pre = pick_targets
        place, place_pre = place_targets
        response.success = True
        response.message = (
            'PREVIEW ONLY: pick_pre='
            f'{self._pose_xyz(pick_pre)}, pick={self._pose_xyz(pick)}, '
            f'place_pre={self._pose_xyz(place_pre)}, '
            f'place={self._pose_xyz(place)}'
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
        self.publish_status('P&P: waiting for fresh pick/place markers')
        locked, reason = self.wait_for_both_targets()
        if locked is None:
            self.publish_status(f'P&P FAILED: {reason}')
            return
        pick_targets, place_targets = locked
        self.publish_status('P&P: both base-frame targets locked')
        self.execute_pick_and_place(pick_targets, place_targets)

    def execute_pick_and_place(self, pick_targets, place_targets):
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
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
            self.move_cartesian_to_pose(place)
            self.publish_status('P&P PLACE: opening gripper')
            self.command_gripper(open_gripper=True)
            self._move_adaptive_lift(
                place,
                self.lift_after_place,
                self.minimum_lift_after_place,
                'P&P PLACE',
            )
            self.publish_status('P&P: completed')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'P&P FAILED: {exc}')
        finally:
            self.motion_lock.release()

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
