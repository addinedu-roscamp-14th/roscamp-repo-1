"""Collect arm2 hand-eye samples with guarded MoveIt motions."""

import copy
import math
import threading
import time

from easy_handeye2_msgs.srv import (
    ComputeCalibration,
    RemoveSample,
    SaveCalibration,
    TakeSample,
)
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    MoveItErrorCodes,
    OrientationConstraint,
    PositionConstraint,
)
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener


GET_CURRENT_TRANSFORMS = '/easy_handeye2/calibration/get_current_transforms'
GET_SAMPLE_LIST = '/easy_handeye2/calibration/get_sample_list'
TAKE_SAMPLE = '/easy_handeye2/calibration/take_sample'
REMOVE_SAMPLE = '/easy_handeye2/calibration/remove_sample'
COMPUTE_CALIBRATION = '/easy_handeye2/calibration/compute_calibration'
SAVE_CALIBRATION = '/easy_handeye2/calibration/save_calibration'


def normalize_quaternion(quaternion):
    """Return a normalized XYZW quaternion tuple."""
    norm = math.sqrt(sum(float(value) ** 2 for value in quaternion))
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero')
    return tuple(float(value) / norm for value in quaternion)


def quaternion_multiply(left, right):
    """Multiply two XYZW quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion((
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ))


def quaternion_from_rpy(roll, pitch, yaw):
    """Create an XYZW quaternion from radian RPY values."""
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion((
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ))


def quaternion_angle_degrees(left, right):
    """Return the shortest angular difference between two quaternions."""
    left = normalize_quaternion(left)
    right = normalize_quaternion(right)
    dot = abs(sum(a * b for a, b in zip(left, right)))
    return math.degrees(2.0 * math.acos(max(-1.0, min(1.0, dot))))


def generate_calibration_targets(home, rotation_degrees, translation_m):
    """Generate the official easy_handeye2 pose pattern around home."""
    targets = []
    angle = math.radians(rotation_degrees)
    base_quaternion = (
        home.pose.orientation.x,
        home.pose.orientation.y,
        home.pose.orientation.z,
        home.pose.orientation.w,
    )
    for scale in (1.0, 0.5):
        scaled_angle = angle * scale
        for roll, pitch, yaw in (
            (scaled_angle, 0.0, 0.0),
            (-scaled_angle, 0.0, 0.0),
            (0.0, scaled_angle, 0.0),
            (0.0, -scaled_angle, 0.0),
            (0.0, 0.0, scaled_angle),
            (0.0, 0.0, -scaled_angle),
        ):
            target = copy.deepcopy(home)
            quaternion = quaternion_multiply(
                base_quaternion, quaternion_from_rpy(roll, pitch, yaw)
            )
            (
                target.pose.orientation.x,
                target.pose.orientation.y,
                target.pose.orientation.z,
                target.pose.orientation.w,
            ) = quaternion
            targets.append(target)

    for dx, dy, dz in (
        (translation_m * 0.5, 0.0, 0.0),
        (-translation_m * 0.5, 0.0, 0.0),
        (0.0, translation_m, 0.0),
        (0.0, -translation_m, 0.0),
        (0.0, 0.0, translation_m / 3.0),
    ):
        target = copy.deepcopy(home)
        target.pose.position.x += dx
        target.pose.position.y += dy
        target.pose.position.z += dz
        targets.append(target)
    return targets


def calibration_target_labels(rotation_degrees, translation_m):
    """Return labels matching the official target generation order."""
    half_rotation = rotation_degrees * 0.5
    return [
        f'Roll +{rotation_degrees:g}deg',
        f'Roll -{rotation_degrees:g}deg',
        f'Pitch +{rotation_degrees:g}deg',
        f'Pitch -{rotation_degrees:g}deg',
        f'Yaw +{rotation_degrees:g}deg',
        f'Yaw -{rotation_degrees:g}deg',
        f'Roll +{half_rotation:g}deg',
        f'Roll -{half_rotation:g}deg',
        f'Pitch +{half_rotation:g}deg',
        f'Pitch -{half_rotation:g}deg',
        f'Yaw +{half_rotation:g}deg',
        f'Yaw -{half_rotation:g}deg',
        f'X +{translation_m * 500.0:g}mm',
        f'X -{translation_m * 500.0:g}mm',
        f'Y +{translation_m * 1000.0:g}mm',
        f'Y -{translation_m * 1000.0:g}mm',
        f'Z +{translation_m * 1000.0 / 3.0:g}mm',
    ]


def transform_is_stable(samples, translation_tolerance, rotation_tolerance):
    """Check robot and tracking transform spread over several readings."""
    if len(samples) < 2:
        return False
    reference = samples[0]
    for sample in samples[1:]:
        for field_name in ('robot', 'tracking'):
            first = getattr(reference, field_name)
            current = getattr(sample, field_name)
            distance = math.sqrt(
                (first.translation.x - current.translation.x) ** 2
                + (first.translation.y - current.translation.y) ** 2
                + (first.translation.z - current.translation.z) ** 2
            )
            if distance > translation_tolerance:
                return False
            first_q = (
                first.rotation.x,
                first.rotation.y,
                first.rotation.z,
                first.rotation.w,
            )
            current_q = (
                current.rotation.x,
                current.rotation.y,
                current.rotation.z,
                current.rotation.w,
            )
            rotation_error = quaternion_angle_degrees(first_q, current_q)
            if rotation_error > rotation_tolerance:
                return False
    return True


def trajectory_is_reasonable(points, limits_degrees):
    """Reject empty plans and excessive per-joint travel."""
    if not points:
        return False
    joint_count = len(points[0].positions)
    if joint_count != len(limits_degrees):
        return False
    for joint_index, limit_degrees in enumerate(limits_degrees):
        positions = [point.positions[joint_index] for point in points]
        travel_degrees = math.degrees(max(positions) - min(positions))
        if travel_degrees > limit_degrees:
            return False
    return True


class Arm2AutoHandeyeSampler(Node):
    """Move around a taught home pose and collect easy_handeye2 samples."""

    def __init__(self):
        """Configure guarded motion, sampling services and status output."""
        super().__init__('arm2_auto_handeye_sampler')
        self.declare_parameter('base_frame', 'arm2/base_link')
        self.declare_parameter('ee_link', 'arm2/TCP')
        self.declare_parameter('moveit_group', 'arm_group')
        self.declare_parameter('move_group_action', '/arm2/move_action')
        self.declare_parameter('stop_service', '/arm2/stop_robot')
        self.declare_parameter('rotation_delta_deg', 25.0)
        self.declare_parameter('translation_delta_m', 0.10)
        self.declare_parameter('velocity_scale', 0.50)
        self.declare_parameter('acceleration_scale', 0.50)
        self.declare_parameter('planning_attempts', 20)
        self.declare_parameter('planning_time_sec', 10.0)
        self.declare_parameter('motion_timeout_sec', 45.0)
        self.declare_parameter('settle_sec', 1.5)
        self.declare_parameter('sample_readings', 3)
        self.declare_parameter('sample_reading_period_sec', 0.20)
        self.declare_parameter('sample_retries', 8)
        self.declare_parameter(
            'marker_pose_topic', '/arm2/gripper_camera/aruco_pose'
        )
        self.declare_parameter('marker_max_age_sec', 0.5)
        self.declare_parameter('marker_detection_timeout_sec', 1.5)
        self.declare_parameter('startup_timeout_sec', 60.0)
        self.declare_parameter('stable_translation_m', 0.003)
        self.declare_parameter('stable_rotation_deg', 3.0)
        self.declare_parameter('minimum_samples', 8)
        self.declare_parameter('clear_existing_samples', True)
        self.declare_parameter('compute_and_save', True)
        self.declare_parameter('position_tolerance_m', 0.005)
        self.declare_parameter('orientation_tolerance_deg', 5.0)
        self.declare_parameter('workspace_min', [-0.28, -0.28, -0.10])
        self.declare_parameter('workspace_max', [0.28, 0.28, 0.35])

        self.base_frame = str(self.get_parameter('base_frame').value)
        self.ee_link = str(self.get_parameter('ee_link').value)
        self.moveit_group = str(self.get_parameter('moveit_group').value)
        self.workspace_min = list(self.get_parameter('workspace_min').value)
        self.workspace_max = list(self.get_parameter('workspace_max').value)
        self.stop_event = threading.Event()
        self.worker_lock = threading.Lock()
        self.worker = None
        self.current_goal_lock = threading.Lock()
        self.current_goal = None
        self.marker_lock = threading.Lock()
        self.marker_detection_count = 0
        self.last_marker_monotonic = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            str(self.get_parameter('move_group_action').value),
        )
        self.stop_robot_client = self.create_client(
            Trigger, str(self.get_parameter('stop_service').value)
        )
        self.current_transforms_client = self.create_client(
            TakeSample, GET_CURRENT_TRANSFORMS
        )
        self.sample_list_client = self.create_client(
            TakeSample, GET_SAMPLE_LIST
        )
        self.take_sample_client = self.create_client(
            TakeSample, TAKE_SAMPLE
        )
        self.remove_sample_client = self.create_client(
            RemoveSample, REMOVE_SAMPLE
        )
        self.compute_client = self.create_client(
            ComputeCalibration, COMPUTE_CALIBRATION
        )
        self.save_client = self.create_client(
            SaveCalibration, SAVE_CALIBRATION
        )

        status_qos = QoSProfile(depth=1)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self.status_publisher = self.create_publisher(
            String, 'auto_handeye/status', status_qos
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter('marker_pose_topic').value),
            self.marker_pose_callback,
            10,
        )
        self.create_service(Trigger, 'start_auto_handeye', self.start_callback)
        self.create_service(Trigger, 'stop_auto_handeye', self.stop_callback)
        self.create_service(
            Trigger, 'preview_auto_handeye', self.preview_callback
        )
        self.publish_status('IDLE: call /arm2/start_auto_handeye to begin')

    def marker_pose_callback(self, message):
        """Record receipt of a genuinely new ArUco pose observation."""
        del message
        with self.marker_lock:
            self.marker_detection_count += 1
            self.last_marker_monotonic = time.monotonic()

    def publish_status(self, message):
        """Publish and log one state transition."""
        self.status_publisher.publish(String(data=message))
        self.get_logger().info(message)

    def start_callback(self, request, response):
        """Start the physical calibration sequence in a worker thread."""
        del request
        with self.worker_lock:
            if self.worker is not None and self.worker.is_alive():
                response.success = False
                response.message = 'Automatic calibration is already running'
                return response
            self.stop_event.clear()
            self.worker = threading.Thread(
                target=self.run_sequence,
                name='arm2-auto-handeye',
                daemon=True,
            )
            self.worker.start()
        response.success = True
        response.message = 'Automatic calibration accepted'
        return response

    def stop_callback(self, request, response):
        """Cancel the active MoveIt goal and stop physical motion."""
        del request
        self.stop_event.set()
        with self.current_goal_lock:
            goal_handle = self.current_goal
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        if self.stop_robot_client.service_is_ready():
            self.stop_robot_client.call_async(Trigger.Request())
        self.publish_status('STOPPING: cancellation requested')
        response.success = True
        response.message = 'Stop requested'
        return response

    def preview_callback(self, request, response):
        """Start the official plan-only starting-pose check."""
        del request
        with self.worker_lock:
            if self.worker is not None and self.worker.is_alive():
                response.success = False
                response.message = 'An automatic check or run is active'
                return response
            self.stop_event.clear()
            self.worker = threading.Thread(
                target=self.run_preview,
                name='arm2-auto-handeye-preview',
                daemon=True,
            )
            self.worker.start()
        response.success = True
        response.message = (
            'Plan-only check accepted; monitor /arm2/auto_handeye/status'
        )
        return response

    def run_preview(self):
        """Check every official target without moving or sampling."""
        try:
            self.wait_for_motion_interfaces()
            home = self.current_pose()
            targets = self.generated_targets(home)
            invalid = [index + 1 for index, pose in enumerate(targets)
                       if not self.pose_in_workspace(pose)]
            if invalid:
                raise RuntimeError(
                    f'generated targets outside workspace: {invalid}'
                )
            self.publish_status(
                f'PREVIEW: home locked; checking {len(targets)} targets'
            )
            self.preflight_targets(targets)
            self.publish_status(
                'READY: all official targets are plannable; '
                'call /arm2/start_auto_handeye'
            )
        except RuntimeError as exc:
            state = 'STOPPED' if self.stop_event.is_set() else 'FAILED'
            self.publish_status(f'{state}: {exc}')
        except Exception as exc:
            self.get_logger().exception(f'Preview failed: {exc}')
            self.publish_status(f'FAILED: {exc}')

    def run_sequence(self):
        """Execute target/home cycles and save a computed calibration."""
        try:
            self.wait_for_interfaces()
            home = self.current_pose()
            targets = self.generated_targets(home)
            invalid = [index + 1 for index, pose in enumerate(targets)
                       if not self.pose_in_workspace(pose)]
            if invalid:
                raise RuntimeError(
                    f'generated targets outside workspace: {invalid}'
                )
            if bool(self.get_parameter('clear_existing_samples').value):
                self.clear_samples()

            self.publish_status(
                f'RUNNING: home locked; {len(targets)} targets prepared'
            )
            self.preflight_targets(targets)
            self.wait_settle()
            successful = 0
            if self.capture_stable_sample():
                successful += 1
                self.publish_status('SAMPLED: home pose (1 sample)')

            for index, target in enumerate(targets, start=1):
                self.raise_if_stopped()
                try:
                    self.publish_status(
                        f'MOVING: target {index}/{len(targets)}'
                    )
                    self.execute_pose_goal(target)
                    self.wait_settle()
                    if self.capture_stable_sample():
                        successful += 1
                        self.publish_status(
                            f'SAMPLED: target {index}/{len(targets)}; '
                            f'run samples={successful}'
                        )
                    else:
                        self.publish_status(
                            f'SKIPPED: target {index}; marker was not stable'
                        )
                except RuntimeError as exc:
                    if self.stop_event.is_set():
                        raise
                    self.get_logger().warning(
                        f'Target {index} skipped: {exc}'
                    )
                finally:
                    if not self.stop_event.is_set():
                        self.publish_status(
                            f'RETURNING: home after target {index}'
                        )
                        self.execute_pose_goal(home)
                        self.wait_settle()

            minimum = int(self.get_parameter('minimum_samples').value)
            if successful < minimum:
                raise RuntimeError(
                    f'only {successful} stable samples; '
                    f'need at least {minimum}'
                )
            if bool(self.get_parameter('compute_and_save').value):
                filepath = self.compute_and_save()
                self.publish_status(
                    f'COMPLETE: {successful} samples; saved={filepath}'
                )
            else:
                self.publish_status(
                    f'COMPLETE: {successful} samples collected; '
                    'compute_and_save is disabled'
                )
        except RuntimeError as exc:
            state = 'STOPPED' if self.stop_event.is_set() else 'FAILED'
            self.publish_status(f'{state}: {exc}')
        except Exception as exc:  # Keep the service node alive on ROS errors.
            self.get_logger().exception(f'Automatic calibration failed: {exc}')
            self.publish_status(f'FAILED: {exc}')
        finally:
            with self.current_goal_lock:
                self.current_goal = None

    def wait_for_interfaces(self):
        """Require MoveIt and all easy_handeye2 services before moving."""
        self.wait_for_motion_interfaces()

        startup_timeout = float(
            self.get_parameter('startup_timeout_sec').value
        )
        self.publish_status(
            'WAITING: show ArUco marker to the gripper camera'
        )
        marker_deadline = time.monotonic() + startup_timeout
        while time.monotonic() < marker_deadline:
            self.raise_if_stopped()
            if self.marker_count() > 0 and self.marker_is_fresh():
                break
            if self.stop_event.wait(0.05):
                self.raise_if_stopped()
        else:
            raise RuntimeError(
                'no fresh ArUco pose on '
                f'{self.get_parameter("marker_pose_topic").value}'
            )

        self.publish_status(
            'WAITING: ArUco acquired; easy_handeye2 is initializing'
        )
        clients = (
            self.current_transforms_client,
            self.sample_list_client,
            self.take_sample_client,
            self.remove_sample_client,
            self.compute_client,
            self.save_client,
        )
        for client in clients:
            if not client.wait_for_service(timeout_sec=startup_timeout):
                raise RuntimeError(
                    f'easy_handeye2 service unavailable: {client.srv_name}'
                )

    def wait_for_motion_interfaces(self):
        """Require only MoveIt and robot TCP TF for a plan-only check."""
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError('MoveIt /arm2/move_action is unavailable')
        try:
            self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'TCP TF unavailable: {self.base_frame} -> {self.ee_link}: '
                f'{exc}'
            ) from exc

    def preflight_targets(self, targets):
        """Plan every official target before allowing physical movement."""
        labels = calibration_target_labels(
            float(self.get_parameter('rotation_delta_deg').value),
            float(self.get_parameter('translation_delta_m').value),
        )
        for index, target in enumerate(targets, start=1):
            self.raise_if_stopped()
            label = labels[index - 1]
            self.publish_status(
                f'PRECHECK: target {index}/{len(targets)} ({label})'
            )
            try:
                result = self.execute_pose_goal(target, plan_only=True)
            except RuntimeError as exc:
                raise RuntimeError(
                    f'precheck target {index}/{len(targets)} '
                    f'({label}) failed: {exc}'
                ) from exc
            points = result.planned_trajectory.joint_trajectory.points
            if not trajectory_is_reasonable(
                points, [90.0, 90.0, 90.0, 90.0, 180.0, 350.0]
            ):
                raise RuntimeError(
                    f'target {index} produced an empty or excessive joint plan'
                )
        self.publish_status(
            f'PRECHECK COMPLETE: all {len(targets)} targets are plannable'
        )

    def current_pose(self):
        """Read the latest TCP pose in the robot base frame."""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, self.ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(f'cannot read current TCP pose: {exc}') from exc
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = Time().to_msg()
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        return pose

    def generated_targets(self, home):
        """Build targets from configured relative deltas."""
        return generate_calibration_targets(
            home,
            float(self.get_parameter('rotation_delta_deg').value),
            float(self.get_parameter('translation_delta_m').value),
        )

    def pose_in_workspace(self, pose):
        """Return whether a target lies inside the configured box."""
        position = pose.pose.position
        values = (position.x, position.y, position.z)
        return all(
            minimum <= value <= maximum
            for value, minimum, maximum in zip(
                values, self.workspace_min, self.workspace_max
            )
        )

    def clear_samples(self):
        """Remove all samples currently stored in easy_handeye2."""
        response = self.call_service(
            self.sample_list_client, TakeSample.Request(), 5.0
        )
        count = len(response.samples.samples)
        for _ in range(count):
            request = RemoveSample.Request()
            request.sample_index = 0
            self.call_service(self.remove_sample_client, request, 5.0)
        self.publish_status(f'PREPARING: cleared {count} existing samples')

    def capture_stable_sample(self):
        """Take one sample only after repeated stable transform readings."""
        retries = int(self.get_parameter('sample_retries').value)
        reading_count = int(self.get_parameter('sample_readings').value)
        reading_period = float(
            self.get_parameter('sample_reading_period_sec').value
        )
        for _ in range(retries):
            self.raise_if_stopped()
            readings = []
            marker_count = self.marker_count()
            for _ in range(reading_count):
                marker_count = self.wait_for_new_marker(marker_count)
                if marker_count is None:
                    readings = []
                    break
                response = self.call_service(
                    self.current_transforms_client,
                    TakeSample.Request(),
                    3.0,
                )
                if not response.samples.samples:
                    readings = []
                    break
                readings.append(response.samples.samples[0])
                if self.stop_event.wait(reading_period):
                    self.raise_if_stopped()
            if readings and transform_is_stable(
                readings,
                float(self.get_parameter('stable_translation_m').value),
                float(self.get_parameter('stable_rotation_deg').value),
            ):
                if not self.marker_is_fresh():
                    continue
                before = self.sample_count()
                response = self.call_service(
                    self.take_sample_client, TakeSample.Request(), 5.0
                )
                return len(response.samples.samples) == before + 1
            if self.stop_event.wait(0.25):
                self.raise_if_stopped()
        return False

    def marker_count(self):
        """Return the number of marker poses received by this node."""
        with self.marker_lock:
            return self.marker_detection_count

    def marker_is_fresh(self):
        """Return whether the most recent marker pose is still current."""
        with self.marker_lock:
            last_marker = self.last_marker_monotonic
        if last_marker is None:
            return False
        maximum_age = float(
            self.get_parameter('marker_max_age_sec').value
        )
        return time.monotonic() - last_marker <= maximum_age

    def wait_for_new_marker(self, previous_count):
        """Wait for a fresh detection newer than the previous observation."""
        timeout = float(
            self.get_parameter('marker_detection_timeout_sec').value
        )
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.raise_if_stopped()
            with self.marker_lock:
                current_count = self.marker_detection_count
                last_marker = self.last_marker_monotonic
            if current_count > previous_count and last_marker is not None:
                maximum_age = float(
                    self.get_parameter('marker_max_age_sec').value
                )
                if time.monotonic() - last_marker <= maximum_age:
                    return current_count
            if self.stop_event.wait(0.02):
                self.raise_if_stopped()
        return None

    def sample_count(self):
        """Return the easy_handeye2 server's current sample count."""
        response = self.call_service(
            self.sample_list_client, TakeSample.Request(), 5.0
        )
        return len(response.samples.samples)

    def compute_and_save(self):
        """Compute the hand-eye transform and persist it through the server."""
        computed = self.call_service(
            self.compute_client, ComputeCalibration.Request(), 20.0
        )
        if not computed.valid:
            raise RuntimeError('easy_handeye2 returned an invalid calibration')
        saved = self.call_service(
            self.save_client, SaveCalibration.Request(), 10.0
        )
        if not saved.success:
            raise RuntimeError('easy_handeye2 failed to save calibration')
        return saved.filepath.data

    def wait_settle(self):
        """Wait for physical settling while remaining interruptible."""
        if self.stop_event.wait(float(self.get_parameter('settle_sec').value)):
            self.raise_if_stopped()

    def execute_pose_goal(self, target, plan_only=False):
        """Plan and execute one collision-checked MoveGroup pose goal."""
        self.raise_if_stopped()
        target = copy.deepcopy(target)
        target.header.stamp = Time().to_msg()
        goal = MoveGroup.Goal()
        request = goal.request
        request.group_name = self.moveit_group
        request.num_planning_attempts = int(
            self.get_parameter('planning_attempts').value
        )
        request.allowed_planning_time = float(
            self.get_parameter('planning_time_sec').value
        )
        request.max_velocity_scaling_factor = float(
            self.get_parameter('velocity_scale').value
        )
        request.max_acceleration_scaling_factor = float(
            self.get_parameter('acceleration_scale').value
        )
        request.start_state.is_diff = True
        request.workspace_parameters.header.frame_id = self.base_frame
        for axis in ('x', 'y', 'z'):
            index = 'xyz'.index(axis)
            setattr(
                request.workspace_parameters.min_corner,
                axis,
                float(self.workspace_min[index]),
            )
            setattr(
                request.workspace_parameters.max_corner,
                axis,
                float(self.workspace_max[index]),
            )
        request.goal_constraints = [self.pose_constraints(target)]
        goal.planning_options.plan_only = plan_only
        goal.planning_options.look_around = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_future = self.move_group_client.send_goal_async(goal)
        planning_timeout = float(
            self.get_parameter('planning_time_sec').value
        ) + 5.0
        self.wait_future(goal_future, planning_timeout)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected the target pose')
        with self.current_goal_lock:
            self.current_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            result_timeout = planning_timeout
            if not plan_only:
                result_timeout += float(
                    self.get_parameter('motion_timeout_sec').value
                )
            self.wait_future(result_future, result_timeout, goal_handle)
            wrapped = result_future.result()
            if wrapped is None:
                raise RuntimeError('MoveIt returned no result')
            error_code = wrapped.result.error_code
            if error_code.val != MoveItErrorCodes.SUCCESS:
                detail = error_code.message or 'no detail'
                raise RuntimeError(
                    f'MoveIt failed: code={error_code.val}, {detail}'
                )
            return wrapped.result
        finally:
            with self.current_goal_lock:
                self.current_goal = None

    def pose_constraints(self, pose):
        """Build MoveIt position and orientation constraints."""
        constraints = Constraints()
        position = PositionConstraint()
        position.header = pose.header
        position.link_name = self.ee_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [
            float(self.get_parameter('position_tolerance_m').value)
        ]
        region_pose = copy.deepcopy(pose.pose)
        region_pose.orientation.x = 0.0
        region_pose.orientation.y = 0.0
        region_pose.orientation.z = 0.0
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives = [region]
        position.constraint_region.primitive_poses = [region_pose]
        position.weight = 1.0

        orientation = OrientationConstraint()
        orientation.header = pose.header
        orientation.link_name = self.ee_link
        orientation.orientation = pose.pose.orientation
        tolerance = math.radians(
            float(self.get_parameter('orientation_tolerance_deg').value)
        )
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def call_service(self, client, request, timeout):
        """Call a service from the worker without nesting an executor."""
        self.raise_if_stopped()
        future = client.call_async(request)
        self.wait_future(future, timeout)
        try:
            response = future.result()
        except Exception as exc:
            raise RuntimeError(
                f'service call failed ({client.srv_name}): {exc}'
            ) from exc
        if response is None:
            raise RuntimeError(f'no response from {client.srv_name}')
        return response

    def wait_future(self, future, timeout, goal_handle=None):
        """Wait for an executor-owned future with stop and timeout checks."""
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return
            if self.stop_event.wait(0.05):
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                self.raise_if_stopped()
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        raise RuntimeError('ROS request timed out')

    def raise_if_stopped(self):
        """Abort the worker when a user stop has been requested."""
        if self.stop_event.is_set():
            raise RuntimeError('automatic calibration stopped by user')


def main(args=None):
    """Run the arm2 automatic hand-eye sampler."""
    rclpy.init(args=args)
    node = Arm2AutoHandeyeSampler()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_event.set()
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
