"""Track a container marker and optionally execute a guarded pick sequence."""

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
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive
from std_msgs.msg import String
from std_srvs.srv import Trigger
from tf2_ros import Buffer, TransformException, TransformListener

try:
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import (
        Constraints,
        JointConstraint,
        MoveItErrorCodes,
        OrientationConstraint,
        PositionConstraint,
    )
    from moveit_msgs.srv import (
        GetCartesianPath,
        GetPositionIK,
        GetStateValidity,
    )
except ImportError:
    ExecuteTrajectory = None
    GetCartesianPath = None
    GetPositionIK = None
    GetStateValidity = None
    JointConstraint = None
    MoveGroup = None


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


class CartesianPlanningError(RuntimeError):
    """A Cartesian request failed before any physical execution started."""


def normalize_quaternion(values):
    """Return a normalized XYZW quaternion."""
    quaternion = np.asarray(values, dtype=np.float64)
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('Quaternion norm is zero')
    return quaternion / norm


def quaternion_multiply(left, right):
    """Multiply two XYZW quaternions."""
    lx, ly, lz, lw = left
    rx, ry, rz, rw = right
    return normalize_quaternion([
        lw * rx + lx * rw + ly * rz - lz * ry,
        lw * ry - lx * rz + ly * rw + lz * rx,
        lw * rz + lx * ry - ly * rx + lz * rw,
        lw * rw - lx * rx - ly * ry - lz * rz,
    ])


def quaternion_from_rpy_degrees(roll, pitch, yaw):
    """Convert degree RPY to an XYZW quaternion."""
    roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return normalize_quaternion([
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ])


def quaternion_to_rpy_degrees(quaternion):
    """Convert an XYZW quaternion to degree RPY."""
    x, y, z, w = normalize_quaternion(quaternion)
    roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch_value = 2.0 * (w * y - z * x)
    pitch = math.asin(max(-1.0, min(1.0, pitch_value)))
    yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return [math.degrees(value) for value in (roll, pitch, yaw)]


def rotate_vector(quaternion, vector):
    """Rotate a 3D vector with an XYZW quaternion."""
    x, y, z, w = normalize_quaternion(quaternion)
    rotation = np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ])
    return rotation @ np.asarray(vector, dtype=np.float64)


def compose_pose(parent_translation, parent_rotation, offset_translation, offset_rotation):
    """Compose parent->marker and marker->target transforms."""
    translation = np.asarray(parent_translation) + rotate_vector(
        parent_rotation, offset_translation
    )
    rotation = quaternion_multiply(parent_rotation, offset_rotation)
    return translation, rotation


def compose_fixed_base_pose(marker_translation, base_offset, base_rotation):
    """Apply a base-frame offset and fixed orientation to a marker position."""
    translation = np.asarray(marker_translation) + np.asarray(base_offset)
    return translation, normalize_quaternion(base_rotation)


def apply_radial_xy_offset(translation, radial_offset):
    """Move a base-frame target radially relative to the base origin."""
    result = np.asarray(translation, dtype=np.float64).copy()
    offset = float(radial_offset)
    if not math.isfinite(offset):
        raise ValueError('target_radial_offset_m must be finite')
    if abs(offset) < 1e-12:
        return result
    radius = float(np.linalg.norm(result[:2]))
    if radius < 1e-9:
        raise ValueError(
            'cannot apply radial offset to target at the base origin'
        )
    if radius + offset <= 0.0:
        raise ValueError(
            'target_radial_offset_m would cross the base origin'
        )
    result[:2] *= (radius + offset) / radius
    return result


def wrap_degrees(angle):
    """Wrap an angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def compose_yaw_follow_pose(
    marker_translation,
    reference_offset,
    fixed_rpy_degrees,
    marker_yaw_degrees,
    reference_marker_yaw_degrees,
):
    """Follow marker yaw while retaining the taught vertical roll/pitch."""
    yaw_delta = wrap_degrees(
        marker_yaw_degrees - reference_marker_yaw_degrees
    )
    angle = math.radians(yaw_delta)
    cosine, sine = math.cos(angle), math.sin(angle)
    offset = np.asarray(reference_offset, dtype=np.float64)
    rotated_offset = np.array([
        cosine * offset[0] - sine * offset[1],
        sine * offset[0] + cosine * offset[1],
        offset[2],
    ])
    translation = np.asarray(marker_translation) + rotated_offset
    roll, pitch, reference_grasp_yaw = fixed_rpy_degrees
    rotation = quaternion_from_rpy_degrees(
        roll,
        pitch,
        reference_grasp_yaw + yaw_delta,
    )
    return translation, rotation, yaw_delta


def compose_symmetric_yaw_follow_poses(
    marker_translation,
    reference_offset,
    fixed_rpy_degrees,
    marker_yaw_degrees,
    reference_marker_yaw_degrees,
):
    """Return two equivalent parallel-gripper attitudes 180 degrees apart."""
    translation, first_rotation, yaw_delta = compose_yaw_follow_pose(
        marker_translation,
        reference_offset,
        fixed_rpy_degrees,
        marker_yaw_degrees,
        reference_marker_yaw_degrees,
    )
    roll, pitch, reference_grasp_yaw = fixed_rpy_degrees
    first_yaw = wrap_degrees(reference_grasp_yaw + yaw_delta)
    second_yaw = wrap_degrees(first_yaw + 180.0)
    second_rotation = quaternion_from_rpy_degrees(
        roll, pitch, second_yaw
    )
    return (
        translation,
        (first_rotation, second_rotation),
        (first_yaw, second_yaw),
        yaw_delta,
    )


def joint_trajectory_metrics(points):
    """Return endpoint travel and total L1 path length in joint radians."""
    if not points:
        raise ValueError('joint trajectory contains no points')
    positions = np.asarray(
        [point.positions for point in points], dtype=np.float64
    )
    if positions.ndim != 2 or positions.shape[1] == 0:
        raise ValueError('joint trajectory points have invalid positions')
    endpoint_travel = float(np.sum(np.abs(positions[-1] - positions[0])))
    if len(positions) == 1:
        return endpoint_travel, 0.0
    path_length = float(np.sum(np.abs(np.diff(positions, axis=0))))
    return endpoint_travel, path_length


def apply_vertical_pick_offsets(nominal_grasp, pregrasp_lift, extra_depth):
    """Keep pregrasp fixed while applying extra depth only to final grasp."""
    if pregrasp_lift < 0.0 or extra_depth < 0.0:
        raise ValueError('vertical pick offsets must be non-negative')
    nominal = np.asarray(nominal_grasp, dtype=np.float64)
    grasp = nominal.copy()
    grasp[2] -= extra_depth
    pregrasp = nominal.copy()
    pregrasp[2] += pregrasp_lift
    return grasp, pregrasp


def lift_distance_candidates(requested, minimum, step):
    """Return descending lift candidates including the exact minimum."""
    if requested <= 0.0 or minimum <= 0.0 or step <= 0.0:
        raise ValueError('lift distances and search step must be positive')
    if minimum > requested:
        raise ValueError('minimum lift cannot exceed requested lift')
    candidates = []
    distance = float(requested)
    while distance > minimum + 1e-9:
        candidates.append(distance)
        distance -= step
    if not candidates or abs(candidates[-1] - minimum) > 1e-9:
        candidates.append(float(minimum))
    return candidates


def cartesian_path_acceptable(
    fraction,
    requested_distance,
    preferred_fraction,
    absolute_min_fraction,
    max_shortfall,
):
    """Accept a nearly complete path only when its residual is bounded."""
    if fraction >= preferred_fraction:
        return True
    if requested_distance is None or fraction < absolute_min_fraction:
        return False
    shortfall = max(0.0, requested_distance * (1.0 - fraction))
    return shortfall <= max_shortfall


def inverted_l_workspace_contains(
    xy,
    horizontal_min,
    horizontal_max,
    vertical_min,
    vertical_max,
):
    """Return whether XY is inside either bar of a right-side bottom L."""
    point = np.asarray(xy, dtype=np.float64)
    horizontal = np.all(point >= horizontal_min) and np.all(
        point <= horizontal_max
    )
    vertical = np.all(point >= vertical_min) and np.all(
        point <= vertical_max
    )
    return bool(horizontal or vertical)


class ContainerPickCoordinator(Node):
    """Filter marker poses, publish grasp targets, and gate robot execution."""

    def __init__(self):
        super().__init__('container_pick_coordinator')
        self._declare_parameters()
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_frame = str(self.get_parameter('marker_frame').value)
        self.execute_motion = bool(self.get_parameter('execute_motion').value)
        self.motion_backend = str(
            self.get_parameter('motion_backend').value
        ).lower()
        self.allow_full_pick = bool(
            self.get_parameter('allow_full_pick').value
        )
        self.offsets_configured = bool(
            self.get_parameter('offsets_configured').value
        )
        self.use_marker_rotation = bool(
            self.get_parameter('use_marker_rotation_for_grasp').value
        )
        self.grasp_orientation_mode = str(
            self.get_parameter('grasp_orientation_mode').value
        ).lower()
        self.grasp_offset = self._vector_parameter('grasp_offset_xyz_m', 3)
        self.grasp_rpy = self._vector_parameter('grasp_offset_rpy_deg', 3)
        self.grasp_rotation = quaternion_from_rpy_degrees(*self.grasp_rpy)
        self.target_radial_offset = float(
            self.get_parameter('target_radial_offset_m').value
        )
        if not math.isfinite(self.target_radial_offset):
            raise ValueError('target_radial_offset_m must be finite')
        self.reference_marker_yaw = float(
            self.get_parameter('reference_marker_yaw_deg').value
        )
        self.max_yaw_spread = float(
            self.get_parameter('max_yaw_spread_deg').value
        )
        self.max_yaw_delta = float(
            self.get_parameter('max_container_yaw_delta_deg').value
        )
        self.grasp_yaw_fallback_scales = [
            float(value) for value in self.get_parameter(
                'grasp_yaw_fallback_scales'
            ).value
        ]
        self.pregrasp_lift = float(
            self.get_parameter('pregrasp_lift_m').value
        )
        self.grasp_extra_depth = float(
            self.get_parameter('grasp_extra_depth_m').value
        )
        self.refresh_marker_before_descent = bool(
            self.get_parameter('refresh_marker_before_descent').value
        )
        self.pregrasp_test_keep_orientation = bool(
            self.get_parameter(
                'pregrasp_test_keep_current_orientation'
            ).value
        )
        self.lift_after_pick = float(
            self.get_parameter('lift_after_pick_m').value
        )
        self.minimum_lift_after_pick = float(
            self.get_parameter('minimum_lift_after_pick_m').value
        )
        self.lift_search_step = float(
            self.get_parameter('lift_search_step_m').value
        )
        self.minimum_samples = int(
            self.get_parameter('minimum_stable_samples').value
        )
        self.max_translation_std = float(
            self.get_parameter('max_translation_std_m').value
        )
        self.max_rotation_spread = float(
            self.get_parameter('max_rotation_spread_deg').value
        )
        self.dry_run_max_rotation_spread = float(
            self.get_parameter('dry_run_max_rotation_spread_deg').value
        )
        self.max_marker_age = float(
            self.get_parameter('max_marker_age_sec').value
        )
        self.workspace_min = self._vector_parameter('workspace_min_xyz_m', 3)
        self.workspace_max = self._vector_parameter('workspace_max_xyz_m', 3)
        self.workspace_xy_shape = str(
            self.get_parameter('workspace_xy_shape').value
        )
        self.workspace_horizontal_min = self._vector_parameter(
            'workspace_horizontal_min_xy_m', 2
        )
        self.workspace_horizontal_max = self._vector_parameter(
            'workspace_horizontal_max_xy_m', 2
        )
        self.workspace_vertical_min = self._vector_parameter(
            'workspace_vertical_min_xy_m', 2
        )
        self.workspace_vertical_max = self._vector_parameter(
            'workspace_vertical_max_xy_m', 2
        )
        self.speed = int(self.get_parameter('speed').value)
        self.motion_timeout = float(
            self.get_parameter('motion_timeout_sec').value
        )
        self.direct_position_tolerance = float(
            self.get_parameter('direct_position_tolerance_m').value
        )
        self.direct_pose_verification = bool(
            self.get_parameter('direct_pose_verification').value
        )
        self.direct_xy_tolerance = float(
            self.get_parameter('direct_xy_tolerance_m').value
        )
        self.direct_angle_tolerance = float(
            self.get_parameter('direct_angle_tolerance_deg').value
        )
        if self.direct_position_tolerance <= 0.0:
            raise ValueError('direct_position_tolerance_m must be positive')
        if self.direct_xy_tolerance <= 0.0:
            raise ValueError('direct_xy_tolerance_m must be positive')
        if self.direct_angle_tolerance <= 0.0:
            raise ValueError('direct_angle_tolerance_deg must be positive')
        self.stabilization_timeout = float(
            self.get_parameter('stabilization_timeout_sec').value
        )
        self.max_joint_delta = float(
            self.get_parameter('max_joint_delta_deg').value
        )
        self.moveit_group = str(self.get_parameter('moveit_group').value)
        self.moveit_ee_link = str(
            self.get_parameter('moveit_ee_link').value
        )
        self.moveit_position_tolerance = float(
            self.get_parameter('moveit_position_tolerance_m').value
        )
        self.moveit_orientation_tolerance = math.radians(float(
            self.get_parameter('moveit_orientation_tolerance_deg').value
        ))
        self.moveit_planning_time = float(
            self.get_parameter('moveit_planning_time_sec').value
        )
        self.moveit_planning_attempts = int(
            self.get_parameter('moveit_planning_attempts').value
        )
        self.moveit_velocity_scale = float(
            self.get_parameter('moveit_velocity_scale').value
        )
        self.moveit_acceleration_scale = float(
            self.get_parameter('moveit_acceleration_scale').value
        )
        self.cartesian_max_step = float(
            self.get_parameter('cartesian_max_step_m').value
        )
        self.cartesian_min_fraction = float(
            self.get_parameter('cartesian_min_fraction').value
        )
        self.cartesian_absolute_min_fraction = float(
            self.get_parameter('cartesian_absolute_min_fraction').value
        )
        self.cartesian_max_shortfall = float(
            self.get_parameter('cartesian_max_shortfall_m').value
        )
        self.cartesian_max_speed = float(
            self.get_parameter('cartesian_max_speed_mps').value
        )
        self.cartesian_joint_jump = math.radians(float(
            self.get_parameter('cartesian_max_joint_jump_deg').value
        ))
        self.moveit_state_settle = float(
            self.get_parameter('moveit_state_settle_sec').value
        )

        if self.minimum_samples < 3:
            raise ValueError('minimum_stable_samples must be at least 3')
        if np.any(self.workspace_min >= self.workspace_max):
            raise ValueError('workspace minimum must be below maximum')
        if self.workspace_xy_shape not in ('box', 'inverted_l'):
            raise ValueError('workspace_xy_shape must be box or inverted_l')
        if np.any(
            self.workspace_horizontal_min >= self.workspace_horizontal_max
        ):
            raise ValueError('horizontal workspace minimum must be below maximum')
        if np.any(self.workspace_vertical_min >= self.workspace_vertical_max):
            raise ValueError('vertical workspace minimum must be below maximum')
        if self.motion_backend not in ('direct', 'moveit'):
            raise ValueError('motion_backend must be direct or moveit')
        if self.grasp_orientation_mode not in (
            'fixed', 'marker_yaw', 'marker_full'
        ):
            raise ValueError(
                'grasp_orientation_mode must be fixed, marker_yaw, or '
                'marker_full'
            )
        if self.grasp_extra_depth < 0.0:
            raise ValueError('grasp_extra_depth_m must be non-negative')
        if not (
            0.0 <= self.cartesian_absolute_min_fraction
            <= self.cartesian_min_fraction <= 1.0
        ):
            raise ValueError(
                'Cartesian path fractions must satisfy '
                '0 <= absolute <= preferred <= 1'
            )
        if self.cartesian_max_shortfall < 0.0:
            raise ValueError('cartesian_max_shortfall_m must be non-negative')
        if any(
            scale < 0.0 or scale >= 1.0
            for scale in self.grasp_yaw_fallback_scales
        ):
            raise ValueError(
                'grasp_yaw_fallback_scales values must be within [0, 1)'
            )
        lift_distance_candidates(
            self.lift_after_pick,
            self.minimum_lift_after_pick,
            self.lift_search_step,
        )

        self.buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.history = deque(maxlen=max(100, self.minimum_samples * 3))
        self.history_lock = threading.Lock()
        self.last_transform_stamp = None
        self.last_tracking_error = ''
        self.last_tracking_error_time = 0.0
        self.motion_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motion_thread = None
        self.robot = None
        self.move_group_client = None
        self.cartesian_path_client = None
        self.position_ik_client = None
        self.state_validity_client = None
        self.execute_trajectory_client = None
        self.gripper_open_client = None
        self.gripper_close_client = None
        self.stop_robot_client = None
        self.current_moveit_goal = None
        self.moveit_goal_lock = threading.Lock()
        self.last_status_text = ''

        self.status_publisher = self.create_publisher(
            String, '/arm/container_pick/status', 10
        )
        self.marker_pose_publisher = self.create_publisher(
            PoseStamped, '/arm/container_pick/marker_pose', 10
        )
        target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grasp_pose_publisher = self.create_publisher(
            PoseStamped, '/arm/container_pick/grasp_pose', target_qos
        )
        self.pregrasp_pose_publisher = self.create_publisher(
            PoseStamped, '/arm/container_pick/pregrasp_pose', target_qos
        )
        self.create_service(Trigger, '/arm/pick_container', self.start_pick)
        self.create_service(
            Trigger, '/arm/preview_pregrasp', self.preview_pregrasp
        )
        self.create_service(
            Trigger, '/arm/move_to_pregrasp', self.start_pregrasp_test
        )
        self.create_service(
            Trigger,
            '/arm/move_to_symmetric_pregrasp',
            self.start_symmetric_pregrasp_test,
        )
        self.create_service(Trigger, '/arm/stop_pick', self.stop_pick)
        self.create_timer(0.1, self.update_tracking)
        self.create_timer(1.0, self._republish_status)

        if self.execute_motion and self.motion_backend == 'direct':
            self._connect_robot()
            self.create_timer(0.1, self.publish_joint_states)
        elif self.execute_motion:
            self._initialize_moveit_clients()

        mode = 'EXECUTE' if self.execute_motion else 'DRY-RUN'
        self.publish_status(
            f'{mode}/{self.motion_backend}: tracking '
            f'{self.base_frame} -> {self.marker_frame}'
        )

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'arm/base_link')
        self.declare_parameter('marker_frame', 'arm/container_marker')
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('motion_backend', 'direct')
        self.declare_parameter('allow_full_pick', False)
        self.declare_parameter('offsets_configured', False)
        self.declare_parameter('use_marker_rotation_for_grasp', False)
        self.declare_parameter('grasp_orientation_mode', 'fixed')
        self.declare_parameter('grasp_offset_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_offset_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('target_radial_offset_m', 0.0)
        self.declare_parameter('reference_marker_yaw_deg', 0.0)
        self.declare_parameter('max_yaw_spread_deg', 5.0)
        self.declare_parameter('max_container_yaw_delta_deg', 90.0)
        self.declare_parameter(
            'grasp_yaw_fallback_scales', [0.75, 0.5, 0.25, 0.0]
        )
        self.declare_parameter('pregrasp_lift_m', 0.08)
        self.declare_parameter('grasp_extra_depth_m', 0.0)
        self.declare_parameter('refresh_marker_before_descent', True)
        self.declare_parameter(
            'pregrasp_test_keep_current_orientation', True
        )
        self.declare_parameter('lift_after_pick_m', 0.08)
        self.declare_parameter('minimum_lift_after_pick_m', 0.05)
        self.declare_parameter('lift_search_step_m', 0.02)
        self.declare_parameter('minimum_stable_samples', 5)
        self.declare_parameter('max_translation_std_m', 0.005)
        self.declare_parameter('max_rotation_spread_deg', 5.0)
        self.declare_parameter('dry_run_max_rotation_spread_deg', 25.0)
        self.declare_parameter('max_marker_age_sec', 2.0)
        self.declare_parameter('workspace_min_xyz_m', [-0.28, -0.28, 0.02])
        self.declare_parameter('workspace_max_xyz_m', [0.28, 0.28, 0.30])
        self.declare_parameter('workspace_xy_shape', 'box')
        self.declare_parameter(
            'workspace_horizontal_min_xy_m', [-0.28, -0.28]
        )
        self.declare_parameter(
            'workspace_horizontal_max_xy_m', [0.28, 0.0]
        )
        self.declare_parameter(
            'workspace_vertical_min_xy_m', [0.0, -0.28]
        )
        self.declare_parameter(
            'workspace_vertical_max_xy_m', [0.28, 0.28]
        )
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 10)
        self.declare_parameter('motion_timeout_sec', 15.0)
        self.declare_parameter('direct_pose_verification', True)
        self.declare_parameter('direct_xy_tolerance_m', 0.005)
        self.declare_parameter('direct_position_tolerance_m', 0.015)
        self.declare_parameter('direct_angle_tolerance_deg', 6.0)
        self.declare_parameter('stabilization_timeout_sec', 12.0)
        self.declare_parameter('max_joint_delta_deg', 60.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('moveit_group', 'arm_group')
        self.declare_parameter('moveit_ee_link', 'TCP')
        self.declare_parameter('moveit_position_tolerance_m', 0.005)
        self.declare_parameter('moveit_orientation_tolerance_deg', 5.0)
        self.declare_parameter('moveit_planning_time_sec', 5.0)
        self.declare_parameter('moveit_planning_attempts', 10)
        self.declare_parameter('moveit_velocity_scale', 0.35)
        self.declare_parameter('moveit_acceleration_scale', 0.25)
        self.declare_parameter('cartesian_max_step_m', 0.005)
        self.declare_parameter('cartesian_min_fraction', 0.97)
        self.declare_parameter('cartesian_absolute_min_fraction', 0.90)
        self.declare_parameter('cartesian_max_shortfall_m', 0.005)
        self.declare_parameter('cartesian_max_speed_mps', 0.04)
        self.declare_parameter('cartesian_max_joint_jump_deg', 10.0)
        self.declare_parameter('moveit_state_settle_sec', 0.2)

    def _vector_parameter(self, name, length):
        values = np.asarray(self.get_parameter(name).value, dtype=np.float64)
        if values.shape != (length,):
            raise ValueError(f'{name} must contain {length} values')
        return values

    def _connect_robot(self):
        port = str(self.get_parameter('serial_port').value)
        baud = int(self.get_parameter('baud_rate').value)
        self.robot = MyCobot280(port, baud)
        time.sleep(1.0)
        self.robot.set_fresh_mode(1)
        if self.robot.is_power_on() != 1:
            self.robot.power_on()
            time.sleep(0.5)
        self.robot.focus_all_servos()
        time.sleep(0.5)
        if self.robot.is_all_servo_enable() != 1:
            raise RuntimeError('Robot servos are not enabled')

    def _initialize_moveit_clients(self):
        if MoveGroup is None:
            raise RuntimeError(
                'moveit_msgs is unavailable; install ros-jazzy-moveit'
            )
        self.move_group_client = ActionClient(
            self, MoveGroup, '/move_action'
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, '/compute_cartesian_path'
        )
        self.position_ik_client = self.create_client(
            GetPositionIK, '/compute_ik'
        )
        self.state_validity_client = self.create_client(
            GetStateValidity, '/check_state_validity'
        )
        self.gripper_open_client = self.create_client(
            Trigger, '/arm/gripper/open'
        )
        self.gripper_close_client = self.create_client(
            Trigger, '/arm/gripper/close'
        )
        self.stop_robot_client = self.create_client(
            Trigger, '/arm/stop_robot'
        )

    def publish_status(self, text):
        self.last_status_text = text
        message = String()
        message.data = text
        self.status_publisher.publish(message)
        self.get_logger().info(text)

    def _republish_status(self):
        if self.last_status_text:
            message = String()
            message.data = self.last_status_text
            self.status_publisher.publish(message)

    def update_tracking(self):
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.marker_frame, Time()
            )
        except TransformException as exc:
            error = str(exc)
            now = time.monotonic()
            if now - self.last_tracking_error_time >= 5.0:
                self.get_logger().warning(
                    'Cannot collect marker sample: '
                    f'{self.base_frame} -> {self.marker_frame}: {error}'
                )
                self.last_tracking_error = error
                self.last_tracking_error_time = now
            return

        stamp = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec
        )
        if stamp == self.last_transform_stamp:
            return
        self.last_transform_stamp = stamp
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
        with self.history_lock:
            self.history.append((stamp, translation, rotation))
        self.marker_pose_publisher.publish(
            self.make_pose(translation, rotation, transform.header.stamp)
        )

    def stable_marker_pose(self, max_rotation_spread=None, yaw_only=False):
        with self.history_lock:
            samples = list(self.history)[-self.minimum_samples:]
        if len(samples) < self.minimum_samples:
            return None, f'need {self.minimum_samples - len(samples)} more samples'

        now_ns = self.get_clock().now().nanoseconds
        age = (now_ns - samples[-1][0]) / 1e9
        if age < 0.0 or age > self.max_marker_age:
            return None, f'marker age {age:.2f}s exceeds limit'

        translations = np.array([sample[1] for sample in samples])
        translation = np.mean(translations, axis=0)
        translation_std = np.std(translations, axis=0)
        if float(np.max(translation_std)) > self.max_translation_std:
            return None, (
                'marker translation is unstable: std_mm='
                f'{np.round(translation_std * 1000.0, 2).tolist()}'
            )

        if yaw_only:
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
                return None, (
                    f'marker yaw spread {yaw_spread:.2f}deg exceeds limit'
                )
            rotation = quaternion_from_rpy_degrees(
                0.0, 0.0, math.degrees(mean_yaw)
            )
            return (translation, rotation), 'stable'

        reference = samples[-1][2]
        aligned = []
        for sample in samples:
            quaternion = sample[2]
            aligned.append(quaternion if np.dot(reference, quaternion) >= 0 else -quaternion)
        rotation = normalize_quaternion(np.mean(aligned, axis=0))
        spread = max(
            math.degrees(2.0 * math.acos(min(1.0, abs(float(np.dot(rotation, q))))))
            for q in aligned
        )
        rotation_limit = (
            self.max_rotation_spread
            if max_rotation_spread is None else max_rotation_spread
        )
        if spread > rotation_limit:
            return None, f'marker rotation spread {spread:.2f}deg exceeds limit'
        return (translation, rotation), 'stable'

    def calculate_targets(
        self, validate_workspace=True, max_rotation_spread=None
    ):
        mode = self.grasp_orientation_mode
        if self.use_marker_rotation and mode == 'fixed':
            mode = 'marker_full'
        rotation_limit = max_rotation_spread
        if rotation_limit is None and mode == 'fixed':
            rotation_limit = self.dry_run_max_rotation_spread
        marker_pose, reason = self.stable_marker_pose(
            rotation_limit,
            yaw_only=(mode == 'marker_yaw'),
        )
        if marker_pose is None:
            return None, reason
        marker_translation, marker_rotation = marker_pose
        if mode == 'marker_full':
            grasp_translation, grasp_rotation = compose_pose(
                marker_translation,
                marker_rotation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        elif mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            grasp_translation, grasp_rotation, yaw_delta = (
                compose_yaw_follow_pose(
                    marker_translation,
                    self.grasp_offset,
                    self.grasp_rpy,
                    marker_yaw,
                    self.reference_marker_yaw,
                )
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                return None, (
                    f'container yaw delta {yaw_delta:.2f}deg exceeds limit'
                )
        else:
            grasp_translation, grasp_rotation = compose_fixed_base_pose(
                marker_translation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        grasp_translation = apply_radial_xy_offset(
            grasp_translation, self.target_radial_offset
        )
        grasp_translation, pregrasp_translation = apply_vertical_pick_offsets(
            grasp_translation,
            self.pregrasp_lift,
            self.grasp_extra_depth,
        )
        if validate_workspace:
            if not self.in_workspace(grasp_translation):
                return None, f'grasp target outside workspace: {grasp_translation}'
            if not self.in_workspace(pregrasp_translation):
                return None, (
                    f'pregrasp target outside workspace: {pregrasp_translation}'
                )
        stamp = self.get_clock().now().to_msg()
        grasp = self.make_pose(grasp_translation, grasp_rotation, stamp)
        pregrasp = self.make_pose(pregrasp_translation, grasp_rotation, stamp)
        self.grasp_pose_publisher.publish(grasp)
        self.pregrasp_pose_publisher.publish(pregrasp)
        return (grasp, pregrasp), 'targets valid'

    def calculate_symmetric_pregrasp_candidates(self):
        """Build both 180-degree-equivalent marker-yaw pregrasp poses."""
        marker_pose, reason = self.stable_marker_pose(yaw_only=True)
        if marker_pose is None:
            return None, reason
        marker_translation, marker_rotation = marker_pose
        marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
        (
            nominal_grasp,
            rotations,
            yaws,
            yaw_delta,
        ) = compose_symmetric_yaw_follow_poses(
            marker_translation,
            self.grasp_offset,
            self.grasp_rpy,
            marker_yaw,
            self.reference_marker_yaw,
        )
        if abs(yaw_delta) > self.max_yaw_delta:
            return None, (
                f'container yaw delta {yaw_delta:.2f}deg exceeds limit'
            )
        _grasp_translation, pregrasp_translation = (
            apply_vertical_pick_offsets(
                nominal_grasp,
                self.pregrasp_lift,
                self.grasp_extra_depth,
            )
        )
        if not self.in_workspace(pregrasp_translation):
            return None, (
                'symmetric pregrasp target outside workspace: '
                f'{pregrasp_translation}'
            )
        stamp = self.get_clock().now().to_msg()
        candidates = []
        for branch, (rotation, yaw) in enumerate(
            zip(rotations, yaws), start=1
        ):
            candidates.append({
                'branch': branch,
                'yaw': yaw,
                'pose': self.make_pose(
                    pregrasp_translation, rotation, stamp
                ),
            })
        return candidates, (
            f'marker_yaw={marker_yaw:.2f}deg, '
            f'yaw_delta={yaw_delta:.2f}deg'
        )

    def wait_for_new_stable_targets(self):
        with self.history_lock:
            self.history.clear()
        deadline = time.monotonic() + self.stabilization_timeout
        reason = 'no new marker samples'
        last_report_time = 0.0
        last_report_reason = ''
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                return None, 'pick stopped'
            targets, reason = self.calculate_targets()
            if targets is not None:
                return targets, reason
            now = time.monotonic()
            if (
                reason != last_report_reason
                or now - last_report_time >= 1.0
            ):
                self.publish_status(f'PICK: waiting for marker: {reason}')
                last_report_reason = reason
                last_report_time = now
        return None, f'marker did not stabilize: {reason}'

    def in_workspace(self, translation):
        if not (
            np.all(translation >= self.workspace_min)
            and np.all(translation <= self.workspace_max)
        ):
            return False
        if self.workspace_xy_shape == 'box':
            return True
        return inverted_l_workspace_contains(
            translation[:2],
            self.workspace_horizontal_min,
            self.workspace_horizontal_max,
            self.workspace_vertical_min,
            self.workspace_vertical_max,
        )

    def make_pose(self, translation, rotation, stamp):
        message = PoseStamped()
        message.header.stamp = stamp
        message.header.frame_id = self.base_frame
        message.pose.position.x, message.pose.position.y, message.pose.position.z = (
            float(value) for value in translation
        )
        message.pose.orientation.x, message.pose.orientation.y = (
            float(rotation[0]), float(rotation[1])
        )
        message.pose.orientation.z = float(rotation[2])
        message.pose.orientation.w = float(rotation[3])
        return message

    def start_pick(self, _request, response):
        if self.execute_motion and not self.allow_full_pick:
            response.success = False
            response.message = (
                'Full pick is locked; use /arm/move_to_pregrasp'
            )
            return response
        if self.execute_motion and not self.offsets_configured:
            response.success = False
            response.message = (
                'Set offsets_configured=true after measuring grasp offset'
            )
            return response
        if not self.execute_motion:
            targets, reason = self.calculate_targets(
                validate_workspace=False,
                max_rotation_spread=self.dry_run_max_rotation_spread,
            )
            if targets is None:
                response.success = False
                response.message = reason
                return response
            response.success = True
            response.message = 'DRY-RUN: targets published; no robot command sent'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'Pick sequence is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_pick_after_stabilization,
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = 'Pick accepted; waiting for fresh stable marker'
        return response

    def execute_pick_after_stabilization(self):
        """Acquire fresh marker samples before starting physical motion."""
        self.publish_status('PICK: waiting for fresh stable marker')
        targets, reason = self.wait_for_new_stable_targets()
        if targets is None:
            self.publish_status(f'PICK FAILED: {reason}')
            return
        self.publish_status('PICK: fresh marker target locked')
        self.execute_pick(targets)

    def start_pregrasp_test(self, _request, response):
        if not self.execute_motion:
            response.success = False
            response.message = 'Start launch with execute_motion:=true'
            return response
        if not self.offsets_configured:
            response.success = False
            response.message = 'Grasp offsets are not configured'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        targets, reason = self.calculate_targets(validate_workspace=False)
        if targets is None:
            response.success = False
            response.message = reason
            return response
        _grasp, pregrasp = targets
        pregrasp_translation = np.array([
            pregrasp.pose.position.x,
            pregrasp.pose.position.y,
            pregrasp.pose.position.z,
        ])
        if not self.in_workspace(pregrasp_translation):
            response.success = False
            response.message = (
                f'pregrasp target outside workspace: {pregrasp_translation}'
            )
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_pregrasp_test,
            args=(pregrasp,),
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = 'Pregrasp-only motion accepted'
        return response

    def start_symmetric_pregrasp_test(self, _request, response):
        """Plan both symmetric attitudes and execute only the best plan."""
        if not self.execute_motion:
            response.success = False
            response.message = 'Start launch with execute_motion:=true'
            return response
        if self.motion_backend != 'moveit':
            response.success = False
            response.message = (
                'Symmetric pregrasp test requires motion_backend=moveit'
            )
            return response
        if not self.offsets_configured:
            response.success = False
            response.message = 'Grasp offsets are not configured'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        candidates, reason = self.calculate_symmetric_pregrasp_candidates()
        if candidates is None:
            response.success = False
            response.message = reason
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_symmetric_pregrasp_test,
            args=(candidates, reason),
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            'Symmetric pregrasp test accepted; planning both yaw branches'
        )
        return response

    def preview_pregrasp(self, _request, response):
        if not self.offsets_configured:
            response.success = False
            response.message = 'Grasp offsets are not configured'
            return response
        targets, reason = self.calculate_targets(validate_workspace=False)
        if targets is None:
            response.success = False
            response.message = reason
            return response
        grasp, pregrasp = targets
        translation = np.array([
            pregrasp.pose.position.x,
            pregrasp.pose.position.y,
            pregrasp.pose.position.z,
        ])
        if not self.in_workspace(translation):
            response.success = False
            response.message = f'pregrasp outside workspace: {translation}'
            return response
        radius = float(np.linalg.norm(translation[:2]))
        grasp_translation = np.round([
            grasp.pose.position.x,
            grasp.pose.position.y,
            grasp.pose.position.z,
        ], 4).tolist()
        grasp_yaw = quaternion_to_rpy_degrees([
            grasp.pose.orientation.x,
            grasp.pose.orientation.y,
            grasp.pose.orientation.z,
            grasp.pose.orientation.w,
        ])[2]
        response.success = True
        response.message = (
            'PREVIEW ONLY: no motion; pregrasp_m='
            f'{np.round(translation, 4).tolist()}, grasp_m='
            f'{grasp_translation}, '
            f'grasp_yaw_deg={grasp_yaw:.2f}, '
            f'xy_radius_m={radius:.4f}'
        )
        return response

    def stop_pick(self, _request, response):
        self.stop_event.set()
        if self.motion_backend == 'moveit':
            self._stop_moveit_motion()
        elif self.robot is not None:
            try:
                with self.serial_lock:
                    self.robot.stop()
            except Exception as exc:
                response.success = False
                response.message = f'Stop command failed: {exc}'
                return response
        self.publish_status('STOP requested')
        response.success = True
        response.message = 'Stop command sent'
        return response

    def execute_pick(self, initial_targets):
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            grasp, pregrasp = initial_targets
            self.publish_status('PICK: opening gripper')
            self.command_gripper(open_gripper=True)
            self.publish_status(
                'PICK: moving to pregrasp'
            )
            self.move_to_pose(
                pregrasp,
                keep_current_orientation=(
                    self.pregrasp_test_keep_orientation
                ),
            )
            if self.pregrasp_test_keep_orientation:
                self.publish_status('PICK: aligning at pregrasp')
                self.move_to_pose(pregrasp)
            if self.refresh_marker_before_descent:
                refreshed, reason = self.wait_for_new_stable_targets()
                if refreshed is None:
                    raise RuntimeError(
                        f'failed to refresh marker pose: {reason}'
                    )
                grasp, pregrasp = refreshed
            else:
                self.publish_status(
                    'PICK: marker refresh skipped; using locked base target'
                )
            self.publish_status('PICK: descending to grasp pose')
            try:
                self.move_cartesian_to_pose(grasp)
            except CartesianPlanningError as initial_error:
                grasp = self.move_with_yaw_fallbacks(
                    grasp, pregrasp, initial_error
                )
            self.publish_status('PICK: closing gripper')
            self.command_gripper(open_gripper=False)
            self.publish_status('PICK: finding vertical lift path')
            self.move_adaptive_cartesian_lift(grasp)
            self.publish_status('PICK: completed')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'PICK FAILED: {exc}')
        finally:
            self.motion_lock.release()

    def move_with_yaw_fallbacks(self, grasp, pregrasp, initial_error):
        """Retry descent with progressively reduced marker-yaw following."""
        if (
            self.grasp_orientation_mode != 'marker_yaw'
            or not self.grasp_yaw_fallback_scales
        ):
            raise initial_error
        target_rpy = quaternion_to_rpy_degrees([
            grasp.pose.orientation.x,
            grasp.pose.orientation.y,
            grasp.pose.orientation.z,
            grasp.pose.orientation.w,
        ])
        yaw_delta = wrap_degrees(target_rpy[2] - self.grasp_rpy[2])
        failures = [f'full yaw: {initial_error}']
        for scale in self.grasp_yaw_fallback_scales:
            candidate_yaw = self.grasp_rpy[2] + yaw_delta * scale
            candidate_rotation = quaternion_from_rpy_degrees(
                self.grasp_rpy[0], self.grasp_rpy[1], candidate_yaw
            )
            candidate_grasp = copy.deepcopy(grasp)
            candidate_pregrasp = copy.deepcopy(pregrasp)
            for pose in (candidate_grasp, candidate_pregrasp):
                pose.pose.orientation.x = float(candidate_rotation[0])
                pose.pose.orientation.y = float(candidate_rotation[1])
                pose.pose.orientation.z = float(candidate_rotation[2])
                pose.pose.orientation.w = float(candidate_rotation[3])
            self.publish_status(
                'PICK: retrying descent with '
                f'{scale * 100.0:.0f}% container yaw '
                f'(target={candidate_yaw:.2f}deg)'
            )
            try:
                self.move_to_pose(candidate_pregrasp)
                self.move_cartesian_to_pose(candidate_grasp)
                self.publish_status(
                    'PICK: descent succeeded with '
                    f'{scale * 100.0:.0f}% container yaw'
                )
                return candidate_grasp
            except (CartesianPlanningError, RuntimeError) as exc:
                failures.append(f'{scale:.2f}: {exc}')
        raise RuntimeError(
            'No reachable grasp-yaw fallback: ' + '; '.join(failures)
        )

    def execute_pregrasp_test(self, pregrasp):
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self.publish_status('PREGRASP TEST: moving to target')
            self.move_to_pose(
                pregrasp,
                keep_current_orientation=(
                    self.pregrasp_test_keep_orientation
                ),
            )
            self.publish_status('PREGRASP TEST: target reached; stopped')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'PREGRASP TEST FAILED: {exc}')
        finally:
            self.motion_lock.release()

    def execute_symmetric_pregrasp_test(self, candidates, marker_detail):
        """Choose the least-moving reachable symmetric plan and execute it."""
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self.publish_status(
                'SYMMETRIC PREGRASP: planning two branches; '
                f'{marker_detail}'
            )
            feasible = []
            failures = []
            for candidate in candidates:
                branch = candidate['branch']
                yaw = candidate['yaw']
                try:
                    trajectory, planning_time = (
                        self._plan_moveit_pose_goal(candidate['pose'])
                    )
                    endpoint_travel, path_length = (
                        joint_trajectory_metrics(
                            trajectory.joint_trajectory.points
                        )
                    )
                    candidate.update({
                        'trajectory': trajectory,
                        'planning_time': planning_time,
                        'endpoint_travel': endpoint_travel,
                        'path_length': path_length,
                    })
                    feasible.append(candidate)
                    self.publish_status(
                        'SYMMETRIC PREGRASP: '
                        f'branch={branch}, yaw={yaw:.2f}deg reachable, '
                        f'endpoint_travel={math.degrees(endpoint_travel):.1f}'
                        'deg, '
                        f'path_cost={math.degrees(path_length):.1f}deg, '
                        f'planning_time={planning_time:.3f}s'
                    )
                except RuntimeError as exc:
                    failures.append(
                        f'branch={branch}, yaw={yaw:.2f}deg: {exc}'
                    )
                    self.publish_status(
                        'SYMMETRIC PREGRASP: '
                        f'branch={branch}, yaw={yaw:.2f}deg unreachable'
                    )
            if not feasible:
                raise RuntimeError(
                    'neither symmetric pregrasp is reachable: '
                    + '; '.join(failures)
                )
            selected = min(
                feasible,
                key=lambda item: (
                    item['endpoint_travel'],
                    item['path_length'],
                    item['planning_time'],
                ),
            )
            self.pregrasp_pose_publisher.publish(selected['pose'])
            self.publish_status(
                'SYMMETRIC PREGRASP: selected '
                f"branch={selected['branch']}, "
                f"yaw={selected['yaw']:.2f}deg; executing planned trajectory"
            )
            self._execute_moveit_trajectory(selected['trajectory'])
            self.publish_status(
                'SYMMETRIC PREGRASP: selected target reached; stopped'
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'SYMMETRIC PREGRASP FAILED: {exc}')
        finally:
            self.motion_lock.release()

    def move_to_pose(
        self,
        pose,
        keep_current_orientation=False,
        minimum_safe_z=None,
    ):
        if self.stop_event.is_set():
            raise RuntimeError('pick stopped')
        translation = np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ])
        if not self.in_workspace(translation):
            raise RuntimeError(f'motion target outside workspace: {translation}')
        if self.motion_backend == 'moveit':
            self._move_to_pose_moveit(pose, keep_current_orientation)
            return
        rpy = quaternion_to_rpy_degrees([
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        ])
        coords = [*(translation * 1000.0), *rpy]
        if keep_current_orientation:
            with self.serial_lock:
                current_coords = self.robot.get_coords()
            if not (
                isinstance(current_coords, (list, tuple))
                and len(current_coords) == 6
            ):
                raise RuntimeError(
                    f'Failed to read current controller pose: {current_coords}'
                )
            coords[3:] = [float(value) for value in current_coords[3:]]

        self.publish_status(
            'DIRECT send_coords -> '
            f'{np.round(coords, 3).tolist()}'
        )
        with self.serial_lock:
            self.robot.send_coords(
                [float(value) for value in coords],
                self.speed,
                0,
            )
        deadline = time.monotonic() + self.motion_timeout
        if not self.direct_pose_verification:
            stopped_samples = 0
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.1):
                    raise RuntimeError('pick stopped')
                with self.serial_lock:
                    moving = self.robot.is_moving()
                if moving == 1:
                    stopped_samples = 0
                    continue
                if moving == 0:
                    stopped_samples += 1
                    if stopped_samples < 3:
                        continue
                    with self.serial_lock:
                        measured = self.robot.get_coords()
                    self.publish_status(
                        'DIRECT motion stopped; pose error ignored: '
                        f'target={np.round(coords, 2).tolist()}, '
                        f'actual={measured}'
                    )
                    return
                stopped_samples = 0
            raise RuntimeError(
                'direct send_coords motion did not stop before timeout'
            )

        last_report_time = 0.0
        last_measured = None
        last_xy_error = None
        last_z_error = None
        last_angle_error = None
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                raise RuntimeError('pick stopped')
            with self.serial_lock:
                measured = self.robot.get_coords()
            if isinstance(measured, (list, tuple)) and len(measured) == 6:
                last_measured = [float(value) for value in measured]
                last_xy_error = max(
                    abs(actual - target)
                    for actual, target in zip(
                        last_measured[:2], coords[:2]
                    )
                )
                last_z_error = abs(last_measured[2] - coords[2])
                last_angle_error = max(
                    abs(wrap_degrees(actual - target))
                    for actual, target in zip(
                        last_measured[3:], coords[3:]
                    )
                )
                xy_reached = (
                    last_xy_error
                    <= self.direct_xy_tolerance * 1000.0
                )
                if minimum_safe_z is None:
                    z_reached = (
                        last_z_error
                        <= self.direct_position_tolerance * 1000.0
                    )
                else:
                    z_reached = (
                        last_measured[2] >= float(minimum_safe_z) * 1000.0
                    )
                if (
                    xy_reached
                    and z_reached
                    and last_angle_error <= self.direct_angle_tolerance
                ):
                    return
                now = time.monotonic()
                if now - last_report_time >= 1.0:
                    z_detail = (
                        f'z_error={last_z_error:.1f}mm'
                        if minimum_safe_z is None
                        else (
                            f'z={last_measured[2]:.1f}mm/'
                            f'safe_z>={float(minimum_safe_z) * 1000.0:.1f}mm'
                        )
                    )
                    self.publish_status(
                        'DIRECT waiting: '
                        f'xy_error={last_xy_error:.1f}mm, '
                        f'{z_detail}, '
                        f'angle_error={last_angle_error:.1f}deg, '
                        f'actual={np.round(last_measured, 2).tolist()}'
                    )
                    last_report_time = now
        raise RuntimeError(
            'direct send_coords motion timed out: '
            f'target={np.round(coords, 3).tolist()}, '
            f'xy_error={last_xy_error}, z_error={last_z_error}, '
            f'minimum_safe_z={minimum_safe_z}, '
            f'angle_error={last_angle_error}, actual={last_measured}'
        )

    def _move_to_pose_moveit(self, pose, keep_current_orientation):
        target = copy.deepcopy(pose)
        if keep_current_orientation:
            try:
                current = self.buffer.lookup_transform(
                    self.base_frame, self.moveit_ee_link, Time()
                )
            except TransformException as exc:
                raise RuntimeError(
                    f'current TCP transform unavailable: {exc}'
                ) from exc
            target.pose.orientation = current.transform.rotation
            try:
                self._execute_moveit_pose_goal(target)
                return
            except RuntimeError as exc:
                if self.stop_event.is_set():
                    raise
                self.get_logger().warning(
                    'MoveIt failed with current TCP orientation; retrying '
                    f'the taught grasp orientation: {exc}'
                )
        self._execute_moveit_pose_goal(pose)

    def _plan_moveit_pose_goal(self, target):
        """Plan one pose without executing and return its trajectory."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action server is unavailable')

        target = copy.deepcopy(target)
        target.header.stamp = Time().to_msg()
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
        request.workspace_parameters.header.frame_id = self.base_frame
        request.workspace_parameters.min_corner.x = float(
            self.workspace_min[0]
        )
        request.workspace_parameters.min_corner.y = float(
            self.workspace_min[1]
        )
        request.workspace_parameters.min_corner.z = float(
            self.workspace_min[2]
        )
        request.workspace_parameters.max_corner.x = float(
            self.workspace_max[0]
        )
        request.workspace_parameters.max_corner.y = float(
            self.workspace_max[1]
        )
        request.workspace_parameters.max_corner.z = float(
            self.workspace_max[2]
        )
        request.goal_constraints = [self._moveit_pose_constraints(target)]

        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_future = self.move_group_client.send_goal_async(goal)
        self._wait_future(goal_future, self.moveit_planning_time + 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected the plan-only pose goal')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                self.moveit_planning_time + 10.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('MoveIt returned no plan-only result')
            result = wrapped_result.result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                detail = result.error_code.message or 'no detail'
                raise RuntimeError(
                    'MoveIt plan-only failed: '
                    f'code={result.error_code.val}, message={detail}'
                )
            if not result.planned_trajectory.joint_trajectory.points:
                raise RuntimeError('MoveIt plan-only trajectory is empty')
            return result.planned_trajectory, float(result.planning_time)
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _plan_moveit_joint_goal(
        self, joint_names, joint_positions, tolerance=1e-3
    ):
        """Plan a collision-checked joint target without executing it."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action server is unavailable')
        if JointConstraint is None:
            raise RuntimeError('moveit_msgs JointConstraint is unavailable')
        if len(joint_names) != len(joint_positions):
            raise ValueError('joint target names and positions differ')

        constraints = Constraints()
        for name, position in zip(joint_names, joint_positions):
            joint = JointConstraint()
            joint.joint_name = str(name)
            joint.position = float(position)
            joint.tolerance_above = float(tolerance)
            joint.tolerance_below = float(tolerance)
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
        goal.planning_options.plan_only = True
        goal.planning_options.look_around = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_future = self.move_group_client.send_goal_async(goal)
        self._wait_future(goal_future, self.moveit_planning_time + 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected the plan-only joint goal')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                self.moveit_planning_time + 10.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('MoveIt returned no joint plan result')
            result = wrapped_result.result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                detail = result.error_code.message or 'no detail'
                raise RuntimeError(
                    'MoveIt joint plan failed: '
                    f'code={result.error_code.val}, message={detail}'
                )
            if not result.planned_trajectory.joint_trajectory.points:
                raise RuntimeError('MoveIt joint plan trajectory is empty')
            return result.planned_trajectory, float(result.planning_time)
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def solve_collision_free_ik(self, target, seed_positions, timeout=0.2):
        """Return a collision-free IK result for an exact TCP target."""
        client = self.position_ik_client
        if client is None or not client.wait_for_service(timeout_sec=2.0):
            raise RuntimeError('MoveIt /compute_ik service is unavailable')
        request = GetPositionIK.Request()
        ik = request.ik_request
        ik.group_name = self.moveit_group
        ik.robot_state.is_diff = True
        ik.robot_state.joint_state.name = list(JOINT_NAMES)
        ik.robot_state.joint_state.position = [
            float(value) for value in seed_positions
        ]
        ik.avoid_collisions = True
        ik.ik_link_name = self.moveit_ee_link
        ik.pose_stamped = copy.deepcopy(target)
        ik.pose_stamped.header.stamp = Time().to_msg()
        ik.timeout.sec = int(timeout)
        ik.timeout.nanosec = int((timeout % 1.0) * 1e9)

        future = client.call_async(request)
        self._wait_future(future, timeout + 2.0)
        response = future.result()
        if response is None:
            return None
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            return None
        positions_by_name = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if not all(name in positions_by_name for name in JOINT_NAMES):
            return None
        return np.asarray(
            [positions_by_name[name] for name in JOINT_NAMES],
            dtype=np.float64,
        )

    def plan_cartesian_from_joint_state(
        self, target, joint_positions
    ):
        """Plan, but never execute, a Cartesian path from a joint state."""
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'MoveIt /compute_cartesian_path service is unavailable'
            )
        request = GetCartesianPath.Request()
        request.header = copy.deepcopy(target.header)
        request.header.stamp = Time().to_msg()
        request.start_state.is_diff = True
        request.start_state.joint_state.name = list(JOINT_NAMES)
        request.start_state.joint_state.position = [
            float(value) for value in joint_positions
        ]
        request.group_name = self.moveit_group
        request.link_name = self.moveit_ee_link
        request.waypoints = [copy.deepcopy(target.pose)]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = self.cartesian_joint_jump
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.moveit_velocity_scale
        request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scale
        )
        request.cartesian_speed_limited_link = self.moveit_ee_link
        request.max_cartesian_speed = self.cartesian_max_speed
        return self._compute_cartesian_path(request)

    def _execute_moveit_trajectory(self, trajectory):
        """Execute a previously collision-checked MoveIt trajectory."""
        if not trajectory.joint_trajectory.points:
            raise RuntimeError('MoveIt trajectory is empty')
        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                'MoveIt /execute_trajectory action is unavailable'
            )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
        goal_future = self.execute_trajectory_client.send_goal_async(goal)
        self._wait_future(goal_future, 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected the selected trajectory')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                self.motion_timeout + 30.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('MoveIt execution returned no result')
            error = wrapped_result.result.error_code
            if error.val != MoveItErrorCodes.SUCCESS:
                detail = error.message or 'no detail'
                raise RuntimeError(
                    'MoveIt execution failed: '
                    f'code={error.val}, message={detail}'
                )
            if self.stop_event.wait(self.moveit_state_settle):
                raise RuntimeError('pick stopped')
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _execute_moveit_pose_goal(self, target):
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action server is unavailable')

        target = copy.deepcopy(target)
        # The target is already expressed in base_frame. Plan against the
        # latest TF tree instead of the old marker observation timestamp.
        target.header.stamp = Time().to_msg()
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
        request.workspace_parameters.header.frame_id = self.base_frame
        request.workspace_parameters.min_corner.x = float(
            self.workspace_min[0]
        )
        request.workspace_parameters.min_corner.y = float(
            self.workspace_min[1]
        )
        request.workspace_parameters.min_corner.z = float(
            self.workspace_min[2]
        )
        request.workspace_parameters.max_corner.x = float(
            self.workspace_max[0]
        )
        request.workspace_parameters.max_corner.y = float(
            self.workspace_max[1]
        )
        request.workspace_parameters.max_corner.z = float(
            self.workspace_max[2]
        )
        request.goal_constraints = [self._moveit_pose_constraints(target)]

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
            raise RuntimeError('MoveIt rejected the pose goal')
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
                raise RuntimeError('MoveIt returned no result')
            result = wrapped_result.result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                detail = result.error_code.message or 'no detail'
                raise RuntimeError(
                    'MoveIt failed: '
                    f'code={result.error_code.val}, message={detail}'
                )
            if self.stop_event.wait(self.moveit_state_settle):
                raise RuntimeError('pick stopped')
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _stop_moveit_motion(self):
        with self.moveit_goal_lock:
            moveit_goal = self.current_moveit_goal
        if moveit_goal is not None:
            moveit_goal.cancel_goal_async()
        if self.stop_robot_client is not None:
            self.stop_robot_client.call_async(Trigger.Request())

    def _stop_active_motion(self):
        if self.motion_backend == 'moveit':
            self._stop_moveit_motion()
            return
        if self.robot is None:
            return
        try:
            with self.serial_lock:
                self.robot.stop()
        except Exception:
            pass

    def _moveit_pose_constraints(self, pose):
        constraints = Constraints()
        position = PositionConstraint()
        position.header = pose.header
        position.link_name = self.moveit_ee_link
        region = SolidPrimitive()
        region.type = SolidPrimitive.SPHERE
        region.dimensions = [self.moveit_position_tolerance]
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
        orientation.link_name = self.moveit_ee_link
        orientation.orientation = pose.pose.orientation
        orientation.absolute_x_axis_tolerance = (
            self.moveit_orientation_tolerance
        )
        orientation.absolute_y_axis_tolerance = (
            self.moveit_orientation_tolerance
        )
        orientation.absolute_z_axis_tolerance = (
            self.moveit_orientation_tolerance
        )
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]
        return constraints

    def move_cartesian_to_pose(self, target):
        """Execute a collision-checked straight TCP path to one waypoint."""
        if self.motion_backend != 'moveit':
            self.move_to_pose(target)
            return
        if self.stop_event.is_set():
            raise RuntimeError('pick stopped')
        translation = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ])
        if not self.in_workspace(translation):
            raise CartesianPlanningError(
                f'Cartesian target outside workspace: {translation}'
            )
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            raise CartesianPlanningError(
                'MoveIt /compute_cartesian_path service is unavailable'
            )

        requested_distance = self._cartesian_request_distance(target)
        request = GetCartesianPath.Request()
        request.header = copy.deepcopy(target.header)
        request.header.stamp = Time().to_msg()
        request.start_state.is_diff = True
        request.group_name = self.moveit_group
        request.link_name = self.moveit_ee_link
        request.waypoints = [copy.deepcopy(target.pose)]
        request.max_step = self.cartesian_max_step
        request.jump_threshold = 0.0
        request.prismatic_jump_threshold = 0.0
        request.revolute_jump_threshold = self.cartesian_joint_jump
        request.avoid_collisions = True
        request.max_velocity_scaling_factor = self.moveit_velocity_scale
        request.max_acceleration_scaling_factor = (
            self.moveit_acceleration_scale
        )
        request.cartesian_speed_limited_link = self.moveit_ee_link
        request.max_cartesian_speed = self.cartesian_max_speed

        response = self._compute_cartesian_path(request)
        if response is None:
            raise CartesianPlanningError(
                'MoveIt returned no Cartesian path response'
            )
        if response.error_code.val != MoveItErrorCodes.SUCCESS:
            detail = response.error_code.message or 'no detail'
            raise CartesianPlanningError(
                'Cartesian planning failed: '
                f'code={response.error_code.val}, message={detail}'
            )
        if not cartesian_path_acceptable(
            response.fraction,
            requested_distance,
            self.cartesian_min_fraction,
            self.cartesian_absolute_min_fraction,
            self.cartesian_max_shortfall,
        ):
            diagnostics = self._diagnose_cartesian_limits(
                request, response.fraction
            )
            distance_detail = ''
            if requested_distance is not None:
                shortfall_mm = (
                    requested_distance * (1.0 - response.fraction) * 1000.0
                )
                distance_detail = (
                    f', shortfall={shortfall_mm:.1f}mm exceeds '
                    f'{self.cartesian_max_shortfall * 1000.0:.1f}mm'
                )
            raise CartesianPlanningError(
                'Cartesian path rejected: '
                f'fraction={response.fraction:.3f} is below '
                f'{self.cartesian_min_fraction:.3f}{distance_detail}; '
                f'{diagnostics}'
            )
        if response.fraction < self.cartesian_min_fraction:
            shortfall_mm = (
                requested_distance * (1.0 - response.fraction) * 1000.0
            )
            self.publish_status(
                'PICK: accepting bounded Cartesian shortfall: '
                f'fraction={response.fraction:.3f}, '
                f'shortfall={shortfall_mm:.1f}mm'
            )
        if not response.solution.joint_trajectory.points:
            raise CartesianPlanningError(
                'Cartesian path contains no trajectory points'
            )

        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                'MoveIt /execute_trajectory action is unavailable'
            )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = response.solution
        goal_future = self.execute_trajectory_client.send_goal_async(goal)
        self._wait_future(goal_future, 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected Cartesian trajectory')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                self.motion_timeout + 30.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('Cartesian execution returned no result')
            error = wrapped_result.result.error_code
            if error.val != MoveItErrorCodes.SUCCESS:
                detail = error.message or 'no detail'
                raise RuntimeError(
                    'Cartesian execution failed: '
                    f'code={error.val}, message={detail}'
                )
            if self.stop_event.wait(self.moveit_state_settle):
                raise RuntimeError('pick stopped')
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _cartesian_request_distance(self, target):
        """Return current TCP-to-target distance, or None if TF is absent."""
        try:
            current = self.buffer.lookup_transform(
                target.header.frame_id,
                self.moveit_ee_link,
                Time(),
            )
        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot bound Cartesian shortfall without TCP TF: {exc}'
            )
            return None
        current_xyz = np.array([
            current.transform.translation.x,
            current.transform.translation.y,
            current.transform.translation.z,
        ])
        target_xyz = np.array([
            target.pose.position.x,
            target.pose.position.y,
            target.pose.position.z,
        ])
        return float(np.linalg.norm(target_xyz - current_xyz))

    def _compute_cartesian_path(self, request):
        """Call MoveIt's Cartesian planner and return its response."""
        path_future = self.cartesian_path_client.call_async(request)
        self._wait_future(path_future, self.moveit_planning_time + 5.0)
        return path_future.result()

    def _diagnose_cartesian_limits(self, safe_request, collision_fraction):
        """Plan unsafe diagnostic variants without executing any of them."""
        variants = (
            ('no_collision', False, self.cartesian_joint_jump),
            ('no_joint_jump', True, 0.0),
            ('ik_only', False, 0.0),
        )
        results = []
        no_collision_response = None
        for name, avoid_collisions, joint_jump in variants:
            request = copy.deepcopy(safe_request)
            request.avoid_collisions = avoid_collisions
            request.revolute_jump_threshold = joint_jump
            try:
                response = self._compute_cartesian_path(request)
            except Exception as exc:
                results.append(f'{name}=error({exc})')
                continue
            if response is None:
                results.append(f'{name}=no_response')
            elif response.error_code.val != MoveItErrorCodes.SUCCESS:
                results.append(
                    f'{name}=error_code({response.error_code.val})'
                )
            else:
                results.append(f'{name}={response.fraction:.3f}')
                if name == 'no_collision':
                    no_collision_response = response
        summary = 'diagnostic plan-only fractions: ' + ', '.join(results)
        if (
            no_collision_response is not None
            and no_collision_response.solution.joint_trajectory.points
        ):
            contact_detail = self._diagnose_collision_contacts(
                no_collision_response.solution.joint_trajectory,
                collision_fraction,
            )
            summary += f'; {contact_detail}'
        return summary

    def _diagnose_collision_contacts(self, trajectory, collision_fraction):
        """Find the first colliding state on a non-executed IK trajectory."""
        client = self.state_validity_client
        if client is None or not client.wait_for_service(timeout_sec=1.0):
            return 'collision contacts unavailable (/check_state_validity)'

        points = trajectory.points
        point_count = len(points)
        estimated_index = int(
            math.floor(float(collision_fraction) * max(0, point_count - 1))
        )
        start_index = max(0, estimated_index - 3)
        for index in range(start_index, point_count):
            request = GetStateValidity.Request()
            request.group_name = self.moveit_group
            request.robot_state.is_diff = True
            request.robot_state.joint_state.header = copy.deepcopy(
                trajectory.header
            )
            request.robot_state.joint_state.name = list(
                trajectory.joint_names
            )
            request.robot_state.joint_state.position = list(
                points[index].positions
            )
            try:
                future = client.call_async(request)
                self._wait_future(future, 2.0)
                response = future.result()
            except Exception as exc:
                return f'collision contact query failed: {exc}'
            if response is None:
                return 'collision contact query returned no response'
            if response.valid:
                continue

            progress = (
                index / (point_count - 1)
                if point_count > 1 else 1.0
            )
            if not response.contacts:
                return (
                    f'first invalid state={index + 1}/{point_count} '
                    f'({progress:.3f}), but MoveIt returned no contacts'
                )
            contacts = []
            seen_pairs = set()
            for contact in response.contacts:
                pair = tuple(sorted((
                    contact.contact_body_1,
                    contact.contact_body_2,
                )))
                if pair in seen_pairs:
                    continue
                seen_pairs.add(pair)
                contacts.append(
                    f'{contact.contact_body_1}<->{contact.contact_body_2} '
                    f'at [{contact.position.x:.4f}, '
                    f'{contact.position.y:.4f}, '
                    f'{contact.position.z:.4f}]m, '
                    f'depth={contact.depth * 1000.0:.2f}mm'
                )
            return (
                f'first invalid state={index + 1}/{point_count} '
                f'({progress:.3f}); collision contacts: '
                + ' | '.join(contacts)
            )
        return (
            'no invalid sampled state found on the non-executed IK path; '
            'collision may occur between sampled trajectory points'
        )

    def move_adaptive_cartesian_lift(self, grasp):
        """Execute the largest fully feasible vertical lift."""
        failures = []
        for distance in lift_distance_candidates(
            self.lift_after_pick,
            self.minimum_lift_after_pick,
            self.lift_search_step,
        ):
            lift = copy.deepcopy(grasp)
            lift.pose.position.z += distance
            try:
                self.move_cartesian_to_pose(lift)
                self.publish_status(
                    f'PICK: lifted container {distance * 100.0:.1f} cm'
                )
                return
            except CartesianPlanningError as exc:
                failures.append(f'{distance:.3f}m: {exc}')
                self.get_logger().warning(
                    f'Lift {distance * 100.0:.1f} cm is not feasible; '
                    'trying a shorter path'
                )
        raise RuntimeError(
            'No safe vertical lift path found: ' + '; '.join(failures)
        )

    def _wait_future(self, future, timeout, goal_handle=None):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if future.done():
                return
            if self.stop_event.wait(0.05):
                if goal_handle is not None:
                    goal_handle.cancel_goal_async()
                raise RuntimeError('pick stopped')
        if goal_handle is not None:
            goal_handle.cancel_goal_async()
        raise RuntimeError('MoveIt request timed out')

    def command_gripper(self, open_gripper):
        if self.motion_backend == 'moveit':
            client = (
                self.gripper_open_client
                if open_gripper else self.gripper_close_client
            )
            if not client.wait_for_service(timeout_sec=3.0):
                raise RuntimeError('JetCobot gripper service is unavailable')
            future = client.call_async(Trigger.Request())
            self._wait_future(future, 5.0)
            result = future.result()
            if result is None or not result.success:
                message = 'no response' if result is None else result.message
                raise RuntimeError(f'gripper command failed: {message}')
            return
        value_name = 'gripper_open_value' if open_gripper else 'gripper_closed_value'
        value = int(self.get_parameter(value_name).value)
        speed = int(self.get_parameter('gripper_speed').value)
        with self.serial_lock:
            self.robot.set_gripper_value(value, speed)
        if self.stop_event.wait(1.0):
            raise RuntimeError('pick stopped')

    def publish_joint_states(self):
        if self.robot is None or not self.serial_lock.acquire(blocking=False):
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
        message.name = JOINT_NAMES
        message.position = [math.radians(float(value)) for value in angles]
        self.joint_state_publisher.publish(message)

    @property
    def joint_state_publisher(self):
        if not hasattr(self, '_joint_state_publisher'):
            self._joint_state_publisher = self.create_publisher(
                JointState, '/arm/joint_states', 10
            )
        return self._joint_state_publisher


def main(args=None):
    """Run the container pick coordinator."""
    rclpy.init(args=args)
    node = ContainerPickCoordinator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.robot is not None:
            try:
                node.robot.stop()
            except Exception:
                pass
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
