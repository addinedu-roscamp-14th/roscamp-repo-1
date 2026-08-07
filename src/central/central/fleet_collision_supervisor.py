#!/usr/bin/env python3

"""Top-down vision collision supervisor for the two-vehicle fleet."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import math
from pathlib import Path
import threading
import time

from nav_msgs.msg import Odometry, Path as NavPath
import numpy as np
from porter_interfaces.msg import VehicleState
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import String
from std_srvs.srv import SetBool
import yaml


VEHICLE_IDS = ('agv1', 'agv2')


@dataclass
class VisionTrack:
    """Latest map position and filtered velocity for one detected vehicle."""

    position: np.ndarray | None = None
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(2))
    received_at: float | None = None
    confidence: float = 0.0


@dataclass
class VehicleMotion:
    """Non-vision state used to predict motion along the Nav2 path."""

    busy: bool = False
    emergency: bool = False
    locked_zone: str = ''
    odom_speed: float = 0.0
    plan: list[np.ndarray] = field(default_factory=list)
    fleet_position: np.ndarray | None = None
    fleet_position_received_at: float | None = None


def pixel_to_map(homography, pixel):
    """Transform one top-down image pixel into the shared map frame."""
    source = np.asarray([float(pixel[0]), float(pixel[1]), 1.0])
    projected = np.asarray(homography, dtype=float) @ source
    if abs(float(projected[2])) < 1e-9:
        raise ValueError('camera-map homography produced a zero scale')
    return projected[:2] / projected[2]


def advance_along_path(start, path, distance):
    """Advance from a current position along the nearest part of a path."""
    current = np.asarray(start, dtype=float)
    if distance <= 0.0 or not path:
        return current.copy()

    points = [np.asarray(point, dtype=float) for point in path]
    if len(points) == 1:
        segment = points[0] - current
        length = float(np.linalg.norm(segment))
        if length < 1e-9 or distance >= length:
            return points[0].copy()
        return current + segment * (float(distance) / length)

    nearest_index = 0
    nearest_projection = points[0]
    nearest_distance = math.inf
    for index, (left, right) in enumerate(zip(points, points[1:])):
        segment = right - left
        squared_length = float(np.dot(segment, segment))
        fraction = (
            0.0
            if squared_length < 1e-12
            else float(np.dot(current - left, segment) / squared_length)
        )
        projection = left + segment * min(1.0, max(0.0, fraction))
        separation = float(np.linalg.norm(projection - current))
        if separation < nearest_distance:
            nearest_index = index
            nearest_projection = projection
            nearest_distance = separation

    current = nearest_projection
    remaining = float(distance)
    for target in points[nearest_index + 1:]:
        segment = target - current
        length = float(np.linalg.norm(segment))
        if length < 1e-9:
            continue
        if remaining <= length:
            return current + segment * (remaining / length)
        remaining -= length
        current = target
    return current.copy()


def predict_positions(start, velocity, path, speed, times, held=False):
    """Predict synchronized map positions using a plan or observed velocity."""
    start = np.asarray(start, dtype=float)
    velocity = np.asarray(velocity, dtype=float)
    if held:
        return [start.copy() for _ in times]
    if path and speed > 0.0:
        return [advance_along_path(start, path, speed * value) for value in times]
    return [start + velocity * value for value in times]


def closest_predicted_approach(first, second, times):
    """Return minimum synchronous separation and its prediction time."""
    if not first or len(first) != len(second) or len(first) != len(times):
        raise ValueError('prediction arrays must be non-empty and equal length')
    distances = [
        float(np.linalg.norm(np.asarray(left) - np.asarray(right)))
        for left, right in zip(first, second)
    ]
    index = int(np.argmin(distances))
    return distances[index], float(times[index])


def fresh_position(position, age, maximum_age):
    """Return the position as a 2-vector when it is present, fresh and finite."""
    if position is None or age is None:
        return None
    if not math.isfinite(float(age)) or float(age) > float(maximum_age):
        return None
    value = np.asarray(position, dtype=float)
    if value.shape != (2,) or not np.all(np.isfinite(value)):
        return None
    return value


def select_fresh_position(
    vision_position,
    vision_age,
    fleet_position,
    fleet_age,
    max_vision_age,
    max_fleet_age,
):
    """Prefer physical vision and fall back to a fresh AMCL fleet pose.

    Vision stays first even when it disagrees with the fleet pose: it is a
    direct observation, while AMCL can sit at its default initial_pose without
    ever saying so. _warn_on_identity_mismatch reports the disagreement.
    """
    vision = fresh_position(vision_position, vision_age, max_vision_age)
    fleet = fresh_position(fleet_position, fleet_age, max_fleet_age)
    if vision is not None:
        return vision.copy(), 'vision'
    if fleet is not None:
        return fleet.copy(), 'fleet'
    return None, 'missing'


class FleetCollisionSupervisor(Node):
    """Hold one AGV when top-down vision predicts an imminent collision."""

    def __init__(self):
        super().__init__('fleet_collision_supervisor')
        self.declare_parameter(
            'calibration_yaml',
            'config/central/camera_map_calibration.yaml',
        )
        self.declare_parameter(
            'detection_topic', '/central/yolo/detections'
        )
        self.declare_parameter(
            'status_topic', '/central/fleet/collision_status'
        )
        self.declare_parameter('minimum_detection_confidence', 0.60)
        self.declare_parameter('max_detection_age_sec', 1.5)
        self.declare_parameter('max_fleet_pose_age_sec', 1.5)
        self.declare_parameter('prediction_horizon_sec', 3.0)
        self.declare_parameter('prediction_step_sec', 0.1)
        self.declare_parameter('minimum_separation_m', 0.22)
        self.declare_parameter('release_separation_m', 0.30)
        self.declare_parameter('release_stable_sec', 0.8)
        self.declare_parameter('minimum_hold_sec', 0.5)
        self.declare_parameter('max_hold_sec', 10.0)
        self.declare_parameter('identity_mismatch_m', 0.45)
        self.declare_parameter('hold_transition_timeout_sec', 2.0)
        self.declare_parameter('nominal_plan_speed_mps', 0.12)
        self.declare_parameter('max_prediction_speed_mps', 0.35)
        self.declare_parameter('velocity_filter_alpha', 0.35)
        self.declare_parameter('max_tracking_speed_mps', 0.7)
        self.declare_parameter('motion_threshold_mps', 0.015)
        self.declare_parameter('preferred_priority_vehicle', 'agv1')
        self.declare_parameter('update_rate_hz', 10.0)
        # agv1 carries the blue cargo box and agv2 the yellow one, matching the
        # YOLO classes and pinky.urdf.xacro. 'car_bule' is the misspelled class
        # the model actually emits for blue. Swapping these makes the
        # supervisor measure one robot's detection against the other's fleet
        # pose, which reads as a near-zero separation and latches a hold.
        self.declare_parameter('agv1_labels', ['car_bule', 'car_blue'])
        self.declare_parameter('agv2_labels', ['car_yellow'])

        self.homography = self._load_homography(
            str(self.get_parameter('calibration_yaml').value)
        )
        self.minimum_confidence = float(
            self.get_parameter('minimum_detection_confidence').value
        )
        self.max_detection_age = float(
            self.get_parameter('max_detection_age_sec').value
        )
        self.max_fleet_pose_age = float(
            self.get_parameter('max_fleet_pose_age_sec').value
        )
        self.horizon = float(
            self.get_parameter('prediction_horizon_sec').value
        )
        self.step = float(self.get_parameter('prediction_step_sec').value)
        self.minimum_separation = float(
            self.get_parameter('minimum_separation_m').value
        )
        self.release_separation = float(
            self.get_parameter('release_separation_m').value
        )
        self.release_stable_sec = float(
            self.get_parameter('release_stable_sec').value
        )
        self.minimum_hold_sec = float(
            self.get_parameter('minimum_hold_sec').value
        )
        self.max_hold_sec = float(self.get_parameter('max_hold_sec').value)
        self.identity_mismatch_m = float(
            self.get_parameter('identity_mismatch_m').value
        )
        self.hold_transition_timeout_sec = float(
            self.get_parameter('hold_transition_timeout_sec').value
        )
        self.nominal_plan_speed = float(
            self.get_parameter('nominal_plan_speed_mps').value
        )
        self.max_prediction_speed = float(
            self.get_parameter('max_prediction_speed_mps').value
        )
        self.velocity_alpha = float(
            self.get_parameter('velocity_filter_alpha').value
        )
        self.max_tracking_speed = float(
            self.get_parameter('max_tracking_speed_mps').value
        )
        self.motion_threshold = float(
            self.get_parameter('motion_threshold_mps').value
        )
        self.preferred_priority = str(
            self.get_parameter('preferred_priority_vehicle').value
        )
        rate = float(self.get_parameter('update_rate_hz').value)
        self._validate_parameters(rate)

        self.labels = {}
        for vehicle_id in VEHICLE_IDS:
            for label in self.get_parameter(f'{vehicle_id}_labels').value:
                self.labels[str(label)] = vehicle_id

        self._lock = threading.RLock()
        self.callback_group = ReentrantCallbackGroup()
        self.tracks = {vehicle_id: VisionTrack() for vehicle_id in VEHICLE_IDS}
        self.motion = {
            vehicle_id: VehicleMotion() for vehicle_id in VEHICLE_IDS
        }
        self.hold_clients = {
            vehicle_id: self.create_client(
                SetBool,
                f'/{vehicle_id}/safety_hold',
                callback_group=self.callback_group,
            )
            for vehicle_id in VEHICLE_IDS
        }
        # Older vehicle images expose only emergency_stop. Keep that service
        # as a compatibility transport until every AGV has safety_hold.
        self.emergency_hold_clients = {
            vehicle_id: self.create_client(
                SetBool,
                f'/{vehicle_id}/emergency_stop',
                callback_group=self.callback_group,
            )
            for vehicle_id in VEHICLE_IDS
        }
        self.enabled = True
        self.held_vehicle = ''
        self._hold_transport = ''
        self._hold_transition = False
        self._hold_transition_started_at = None
        self._hold_started_at = None
        self._release_safe_since = None
        self._last_error = ''
        self._last_minimum_distance = None
        self._last_ttc = None
        self._position_sources = {
            vehicle_id: 'missing' for vehicle_id in VEHICLE_IDS
        }
        self._active_positions = {
            vehicle_id: None for vehicle_id in VEHICLE_IDS
        }
        self._tracking_error = ''

        qos = self._telemetry_qos()
        self.create_subscription(
            String,
            str(self.get_parameter('detection_topic').value),
            self._on_detections,
            qos,
            callback_group=self.callback_group,
        )
        for vehicle_id in VEHICLE_IDS:
            self.create_subscription(
                VehicleState,
                f'/central/fleet/{vehicle_id}/state',
                lambda message, vid=vehicle_id: self._on_vehicle_state(
                    vid, message
                ),
                qos,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                Odometry,
                f'/{vehicle_id}/odom',
                lambda message, vid=vehicle_id: self._on_odom(vid, message),
                qos,
                callback_group=self.callback_group,
            )
            self.create_subscription(
                NavPath,
                f'/{vehicle_id}/plan',
                lambda message, vid=vehicle_id: self._on_plan(vid, message),
                qos,
                callback_group=self.callback_group,
            )

        self.status_publisher = self.create_publisher(
            String,
            str(self.get_parameter('status_topic').value),
            10,
        )
        self.create_service(
            SetBool,
            '/central/fleet/collision_supervisor/enabled',
            self._set_enabled,
            callback_group=self.callback_group,
        )
        self.create_timer(1.0 / rate, self._evaluate)
        self.get_logger().info(
            'YOLO collision supervisor ready: '
            f'stop={self.minimum_separation:.2f}m, '
            f'release={self.release_separation:.2f}m, '
            + (
                'measuring the separation right now (no prediction)'
                if self.horizon <= 0.0
                else f'closest approach predicted over {self.horizon:.1f}s'
            )
        )

    @staticmethod
    def _telemetry_qos():
        return QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

    def _validate_parameters(self, rate):
        positive = {
            'max_detection_age_sec': self.max_detection_age,
            'max_fleet_pose_age_sec': self.max_fleet_pose_age,
            'prediction_step_sec': self.step,
            'minimum_separation_m': self.minimum_separation,
            'release_separation_m': self.release_separation,
            'update_rate_hz': rate,
            'hold_transition_timeout_sec': self.hold_transition_timeout_sec,
            'max_hold_sec': self.max_hold_sec,
            'identity_mismatch_m': self.identity_mismatch_m,
        }
        invalid = [name for name, value in positive.items() if value <= 0.0]
        if invalid:
            raise ValueError(f'parameters must be positive: {invalid}')
        # 0.0 is meaningful: hold on the separation measured right now instead
        # of the closest approach predicted along both planned paths. The
        # predictive form stops vehicles while they are still far apart, which
        # in a workspace this size leaves them unable to pass each other.
        if self.horizon < 0.0:
            raise ValueError('prediction_horizon_sec must be non-negative')
        if self.release_separation <= self.minimum_separation:
            raise ValueError(
                'release_separation_m must exceed minimum_separation_m'
            )
        if self.max_hold_sec <= self.minimum_hold_sec:
            raise ValueError('max_hold_sec must exceed minimum_hold_sec')
        if self.preferred_priority not in VEHICLE_IDS:
            raise ValueError('preferred_priority_vehicle must be agv1 or agv2')
        if not 0.0 < self.velocity_alpha <= 1.0:
            raise ValueError('velocity_filter_alpha must be in (0, 1]')

    @staticmethod
    def _load_homography(configured_path):
        path = Path(configured_path).expanduser().resolve()
        if not path.exists():
            raise FileNotFoundError(
                f'camera-map calibration not found: {path}'
            )
        with path.open('r', encoding='utf-8') as stream:
            calibration = yaml.safe_load(stream) or {}
        try:
            matrix = np.asarray(
                calibration['homography']['camera_pixel_to_map_xy'],
                dtype=float,
            )
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(
                'calibration does not contain a valid camera-to-map homography'
            ) from exc
        if matrix.shape != (3, 3):
            raise ValueError('camera-to-map homography must be 3x3')
        return matrix

    def _on_detections(self, message):
        try:
            summary = json.loads(message.data)
            detections = summary.get('detections', [])
        except (json.JSONDecodeError, AttributeError) as exc:
            self._last_error = f'invalid YOLO JSON: {exc}'
            return

        candidates = {}
        for detection in detections:
            if not isinstance(detection, dict):
                continue
            vehicle_id = self.labels.get(str(detection.get('label', '')))
            confidence = float(detection.get('confidence') or 0.0)
            bbox = detection.get('bbox_xyxy')
            if (
                vehicle_id is None
                or confidence < self.minimum_confidence
                or not isinstance(bbox, list)
                or len(bbox) != 4
            ):
                continue
            current = candidates.get(vehicle_id)
            if current is None or confidence > current[0]:
                center = (
                    (float(bbox[0]) + float(bbox[2])) * 0.5,
                    (float(bbox[1]) + float(bbox[3])) * 0.5,
                )
                candidates[vehicle_id] = (confidence, center)

        now = time.monotonic()
        with self._lock:
            for vehicle_id, (confidence, center) in candidates.items():
                self._update_track(
                    self.tracks[vehicle_id],
                    pixel_to_map(self.homography, center),
                    confidence,
                    now,
                )
            if candidates:
                self._last_error = ''

    def _update_track(self, track, position, confidence, now):
        position = np.asarray(position, dtype=float)
        if track.position is not None and track.received_at is not None:
            elapsed = now - track.received_at
            if 0.02 <= elapsed <= self.max_detection_age * 2.0:
                measured = (position - track.position) / elapsed
                speed = float(np.linalg.norm(measured))
                if speed <= self.max_tracking_speed:
                    track.velocity = (
                        self.velocity_alpha * measured
                        + (1.0 - self.velocity_alpha) * track.velocity
                    )
                else:
                    track.velocity = np.zeros(2)
        track.position = position
        track.received_at = now
        track.confidence = float(confidence)

    def _on_vehicle_state(self, vehicle_id, message):
        now = time.monotonic()
        telemetry_age = float(message.telemetry_age_sec)
        position = np.asarray(
            [message.pose.pose.position.x, message.pose.pose.position.y],
            dtype=float,
        )
        with self._lock:
            motion = self.motion[vehicle_id]
            motion.busy = message.state == VehicleState.BUSY
            motion.emergency = bool(message.emergency_stopped)
            motion.locked_zone = str(message.locked_zone)
            if (
                message.state != VehicleState.OFFLINE
                and math.isfinite(telemetry_age)
                and np.all(np.isfinite(position))
            ):
                motion.fleet_position = position
                motion.fleet_position_received_at = (
                    now - max(0.0, telemetry_age)
                )
            elif message.state == VehicleState.OFFLINE:
                motion.fleet_position_received_at = None

    def _on_odom(self, vehicle_id, message):
        linear = message.twist.twist.linear
        speed = math.hypot(float(linear.x), float(linear.y))
        with self._lock:
            self.motion[vehicle_id].odom_speed = speed

    def _on_plan(self, vehicle_id, message):
        points = [
            np.asarray(
                [pose.pose.position.x, pose.pose.position.y], dtype=float
            )
            for pose in message.poses
        ]
        with self._lock:
            self.motion[vehicle_id].plan = points

    def _set_enabled(self, request, response):
        with self._lock:
            self.enabled = bool(request.data)
            held_vehicle = self.held_vehicle
        if not self.enabled and held_vehicle:
            self._request_hold(held_vehicle, False)
        response.success = True
        response.message = (
            'collision supervisor enabled'
            if self.enabled
            else 'collision supervisor disabled; automatic hold released'
        )
        return response

    @staticmethod
    def identity_mismatch_distance(
        vision_position,
        vision_age,
        fleet_position,
        fleet_age,
        max_vision_age,
        max_fleet_age,
    ):
        """Distance between a vehicle's own two position sources, if both fresh."""
        vision = fresh_position(vision_position, vision_age, max_vision_age)
        fleet = fresh_position(fleet_position, fleet_age, max_fleet_age)
        if vision is None or fleet is None:
            return None
        return float(np.linalg.norm(vision - fleet))

    def _warn_on_identity_mismatch(
        self, vehicle_id, vision_position, vision_age, fleet_position, fleet_age
    ):
        """Catch a detector label mapped to the wrong vehicle.

        The two sources track the same robot, so they must roughly agree. When
        the label map is swapped they instead point at different robots, the
        supervisor reads a near-zero separation between the two vehicles and
        latches a hold that the release threshold can never clear.
        """
        distance = self.identity_mismatch_distance(
            vision_position,
            vision_age,
            fleet_position,
            fleet_age,
            self.max_detection_age,
            self.max_fleet_pose_age,
        )
        if distance is None or distance <= self.identity_mismatch_m:
            return
        self.get_logger().error(
            f'{vehicle_id}: vision detection is {distance:.2f}m from its own '
            f'fleet pose (limit {self.identity_mismatch_m:.2f}m). Either AMCL '
            f'is still at its default initial_pose and never localised, or '
            f'agv1_labels/agv2_labels no longer match the YOLO colour classes. '
            f'Navigation goals for {vehicle_id} will go to the wrong place '
            f'until this agrees.',
            throttle_duration_sec=5.0,
        )

    def _evaluate(self):
        now = time.monotonic()
        with self._lock:
            positions = {}
            sources = {}
            for vehicle_id in VEHICLE_IDS:
                track = self.tracks[vehicle_id]
                motion = self.motion[vehicle_id]
                vision_age = (
                    None
                    if track.received_at is None
                    else now - track.received_at
                )
                fleet_age = (
                    None
                    if motion.fleet_position_received_at is None
                    else now - motion.fleet_position_received_at
                )
                positions[vehicle_id], sources[vehicle_id] = (
                    select_fresh_position(
                        track.position,
                        vision_age,
                        motion.fleet_position,
                        fleet_age,
                        self.max_detection_age,
                        self.max_fleet_pose_age,
                    )
                )
                self._warn_on_identity_mismatch(
                    vehicle_id,
                    track.position,
                    vision_age,
                    motion.fleet_position,
                    fleet_age,
                )
            positions_fresh = all(
                positions[vehicle_id] is not None
                for vehicle_id in VEHICLE_IDS
            )
            self._position_sources = sources
            self._active_positions = positions
            self._tracking_error = (
                ''
                if positions_fresh
                else 'missing fresh position: '
                + ', '.join(
                    vehicle_id for vehicle_id in VEHICLE_IDS
                    if positions[vehicle_id] is None
                )
            )
            if not self.enabled or not positions_fresh:
                self._last_minimum_distance = None
                self._last_ttc = None
                if self.enabled and self._tracking_error:
                    self.get_logger().warning(
                        'Collision monitoring inactive: '
                        f'{self._tracking_error}',
                        throttle_duration_sec=5.0,
                    )
                # A hold must not outlive the tracking that justifies it. The
                # only release below this point needs fresh positions, so
                # without this a vehicle held just as tracking degrades -- it
                # stops, so vision may well lose it -- never gets resumed.
                # Disabling the supervisor retries here too, in case the
                # release from _set_enabled found the service unavailable.
                if self.held_vehicle:
                    held_duration = now - float(self._hold_started_at or now)
                    if not self.enabled or held_duration >= self.max_hold_sec:
                        self.get_logger().error(
                            f'Releasing {self.held_vehicle} after '
                            f'{held_duration:.1f}s without confirmed '
                            'separation: '
                            + (self._tracking_error or 'supervisor disabled')
                        )
                        self._request_hold(self.held_vehicle, False)
                self._publish_status(now, positions_fresh)
                return

            times = np.arange(
                0.0,
                self.horizon + self.step * 0.5,
                self.step,
            ).tolist()
            predictions = {
                vehicle_id: self._predict(
                    vehicle_id, positions[vehicle_id], times
                )
                for vehicle_id in VEHICLE_IDS
            }
            minimum_distance, ttc = closest_predicted_approach(
                predictions['agv1'], predictions['agv2'], times
            )
            self._last_minimum_distance = minimum_distance
            self._last_ttc = ttc
            moving = {
                vehicle_id: self._vehicle_is_moving(vehicle_id)
                for vehicle_id in VEHICLE_IDS
            }
            risk = minimum_distance <= self.minimum_separation and any(
                moving.values()
            )

            if self.held_vehicle:
                held_duration = now - float(self._hold_started_at or now)
                safe = minimum_distance >= self.release_separation
                if safe:
                    if self._release_safe_since is None:
                        self._release_safe_since = now
                else:
                    self._release_safe_since = None
                release_stable = (
                    self._release_safe_since is not None
                    and now - self._release_safe_since
                    >= self.release_stable_sec
                )
                if held_duration >= self.minimum_hold_sec and release_stable:
                    self._request_hold(self.held_vehicle, False)
            elif risk and not self._hold_transition:
                yield_vehicle = self._select_yield_vehicle(moving)
                self._request_hold(yield_vehicle, True)

            self._publish_status(now, positions_fresh)

    def _predict(self, vehicle_id, start_position, times):
        track = self.tracks[vehicle_id]
        motion = self.motion[vehicle_id]
        observed_speed = float(np.linalg.norm(track.velocity))
        if motion.busy and motion.plan:
            speed = max(
                observed_speed,
                motion.odom_speed,
                self.nominal_plan_speed,
            )
        else:
            speed = max(observed_speed, motion.odom_speed)
        speed = min(speed, self.max_prediction_speed)
        return predict_positions(
            start_position,
            track.velocity,
            motion.plan if motion.busy else [],
            speed,
            times,
            held=vehicle_id == self.held_vehicle,
        )

    def _vehicle_is_moving(self, vehicle_id):
        track = self.tracks[vehicle_id]
        motion = self.motion[vehicle_id]
        return (
            motion.busy
            or motion.odom_speed >= self.motion_threshold
            or float(np.linalg.norm(track.velocity)) >= self.motion_threshold
        )

    def _select_yield_vehicle(self, moving):
        first, second = VEHICLE_IDS

        # The B-1 occupant must be able to clear the single-entry loading
        # zone before another AGV approaches it.  Give that vehicle right of
        # way even when both vehicles currently own different zone locks.
        first_in_b1 = self.motion[first].locked_zone == 'B-1'
        second_in_b1 = self.motion[second].locked_zone == 'B-1'
        if first_in_b1 != second_in_b1:
            return second if first_in_b1 else first

        if moving[first] != moving[second]:
            return first if moving[first] else second

        first_zone = bool(self.motion[first].locked_zone)
        second_zone = bool(self.motion[second].locked_zone)
        if first_zone != second_zone:
            return second if first_zone else first

        priority = self.preferred_priority
        return second if priority == first else first

    @staticmethod
    def _hold_transition_is_stuck(started_at, timeout_sec, now=None):
        """Detect a hold/release service call that never got a response."""
        now = time.monotonic() if now is None else now
        pending_sec = now - float(started_at or now)
        return pending_sec >= timeout_sec, pending_sec

    def _request_hold(self, vehicle_id, enabled):
        if self._hold_transition:
            stuck, pending_sec = self._hold_transition_is_stuck(
                self._hold_transition_started_at,
                self.hold_transition_timeout_sec,
            )
            if not stuck:
                return
            self.get_logger().error(
                'Previous hold/release service call never completed after '
                f'{pending_sec:.1f}s; clearing the stuck transition so '
                'hold/release evaluation can resume in real time'
            )
            self._hold_transition = False
        client, transport = self._select_hold_client(vehicle_id, enabled)
        service_name = f'/{vehicle_id}/{transport}' if transport else ''
        if client is None:
            self._last_error = (
                f'/{vehicle_id}/safety_hold and '
                f'/{vehicle_id}/emergency_stop services unavailable'
            )
            self.get_logger().error(
                self._last_error,
                throttle_duration_sec=2.0,
            )
            return
        if not client.service_is_ready():
            self._last_error = f'{service_name} service unavailable'
            self.get_logger().error(
                self._last_error,
                throttle_duration_sec=2.0,
            )
            return

        request = SetBool.Request()
        request.data = bool(enabled)
        self._hold_transition = True
        self._hold_transition_started_at = time.monotonic()
        future = client.call_async(request)
        future.add_done_callback(
            lambda result, vid=vehicle_id, state=bool(enabled), mode=transport:
            self._hold_response(vid, state, mode, result)
        )

    def _select_hold_client(self, vehicle_id, enabled):
        """Use safety_hold when available and preserve the latch transport."""
        if not enabled and self._hold_transport:
            clients = (
                self.hold_clients
                if self._hold_transport == 'safety_hold'
                else self.emergency_hold_clients
            )
            return clients[vehicle_id], self._hold_transport

        safety_client = self.hold_clients[vehicle_id]
        if safety_client.service_is_ready():
            return safety_client, 'safety_hold'
        emergency_client = self.emergency_hold_clients[vehicle_id]
        if emergency_client.service_is_ready():
            return emergency_client, 'emergency_stop'
        return None, ''

    def _hold_response(self, vehicle_id, enabled, transport, future):
        try:
            response = future.result()
            success = bool(response.success)
            detail = str(response.message)
        except Exception as exc:  # noqa: BLE001 - ROS future exception
            success = False
            detail = str(exc)

        now = time.monotonic()
        with self._lock:
            self._hold_transition = False
            self._hold_transition_started_at = None
            if success and enabled:
                self.held_vehicle = vehicle_id
                self._hold_transport = transport
                self._hold_started_at = now
                self._release_safe_since = None
                self._last_error = ''
                self.get_logger().warning(
                    f'COLLISION HOLD: {vehicle_id} via {transport}; {detail}'
                )
            elif success:
                self.held_vehicle = ''
                self._hold_transport = ''
                self._hold_started_at = None
                self._release_safe_since = None
                self._last_error = ''
                self.get_logger().info(
                    f'COLLISION RESUME: {vehicle_id} via {transport}; {detail}'
                )
            else:
                self._last_error = (
                    f'{vehicle_id} safety hold request failed: {detail}'
                )
                self.get_logger().error(self._last_error)

    def _publish_status(self, now, positions_fresh):
        ages = {
            vehicle_id: (
                None
                if track.received_at is None
                else round(now - track.received_at, 3)
            )
            for vehicle_id, track in self.tracks.items()
        }
        fleet_ages = {
            vehicle_id: (
                None
                if motion.fleet_position_received_at is None
                else round(now - motion.fleet_position_received_at, 3)
            )
            for vehicle_id, motion in self.motion.items()
        }
        positions = {
            vehicle_id: (
                None
                if track.position is None
                else [round(float(value), 4) for value in track.position]
            )
            for vehicle_id, track in self.tracks.items()
        }
        active_positions = {
            vehicle_id: (
                None
                if position is None
                else [round(float(value), 4) for value in position]
            )
            for vehicle_id, position in self._active_positions.items()
        }
        status = String()
        status.data = json.dumps(
            {
                'enabled': self.enabled,
                'state': (
                    'HOLDING'
                    if self.held_vehicle
                    else 'MONITORING'
                    if positions_fresh
                    else 'WAITING_FOR_POSITION'
                ),
                'held_vehicle': self.held_vehicle,
                'hold_transport': self._hold_transport,
                'minimum_distance_m': self._optional_round(
                    self._last_minimum_distance
                ),
                'time_to_closest_sec': self._optional_round(self._last_ttc),
                'detection_age_sec': ages,
                'fleet_pose_age_sec': fleet_ages,
                'position_source': self._position_sources,
                'active_position_map': active_positions,
                'vision_position_map': positions,
                'last_error': self._last_error or self._tracking_error,
            },
            ensure_ascii=False,
        )
        self.status_publisher.publish(status)

    @staticmethod
    def _optional_round(value):
        return None if value is None else round(float(value), 3)


def main(args=None):
    rclpy.init(args=args)
    node = FleetCollisionSupervisor()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
