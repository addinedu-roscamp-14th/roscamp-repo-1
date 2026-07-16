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

from ._joint_limits import JOINT_LIMITS_DEG

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


def wrap_degrees(angle):
    """Wrap an angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def select_layer_index(marker_z, taught_marker_z_values, max_error):
    """Select the nearest taught layer, rejecting an unknown marker height."""
    errors = [
        abs(float(marker_z) - float(taught_z))
        for taught_z in taught_marker_z_values
    ]
    index = int(np.argmin(errors))
    if errors[index] > float(max_error):
        return None, errors[index]
    return index, errors[index]


def interpolate_half_turn_profile(
    marker_yaw, base_yaw, quarter_yaw, base_values, quarter_values
):
    """Interpolate calibration values periodically over a 180deg grasp axis."""
    phase = (float(marker_yaw) - float(base_yaw)) % 180.0
    quarter_phase = (float(quarter_yaw) - float(base_yaw)) % 180.0
    if not 1e-6 < quarter_phase < 180.0 - 1e-6:
        raise ValueError('angle calibration yaws must define two distinct axes')
    base = np.asarray(base_values, dtype=np.float64)
    quarter = np.asarray(quarter_values, dtype=np.float64)
    if phase <= quarter_phase:
        weight = phase / quarter_phase
    else:
        # Return smoothly to the base profile at the equivalent 180deg axis.
        weight = 1.0 - (phase - quarter_phase) / (180.0 - quarter_phase)
    return (1.0 - weight) * base + weight * quarter


def interpolate_periodic_profiles(marker_yaw, sample_yaws, sample_values):
    """Circularly interpolate measured profiles over a 180deg gripper axis."""
    yaws = np.asarray(sample_yaws, dtype=np.float64)
    values = np.asarray(sample_values, dtype=np.float64)
    if yaws.ndim != 1 or len(yaws) < 2 or values.shape[0] != len(yaws):
        raise ValueError('periodic calibration requires matching samples')
    phases = np.mod(yaws, 180.0)
    order = np.argsort(phases)
    phases = phases[order]
    values = values[order]
    if np.any(np.diff(phases) < 1e-6):
        raise ValueError('periodic calibration yaws must be distinct')

    query = float(marker_yaw) % 180.0
    right = int(np.searchsorted(phases, query, side='right'))
    if right == 0:
        left = len(phases) - 1
        left_phase = phases[left] - 180.0
        right_phase = phases[0]
        left_value, right_value = values[left], values[0]
    elif right == len(phases):
        left = len(phases) - 1
        left_phase = phases[left]
        right_phase = phases[0] + 180.0
        left_value, right_value = values[left], values[0]
    else:
        left = right - 1
        left_phase, right_phase = phases[left], phases[right]
        left_value, right_value = values[left], values[right]
    weight = (query - left_phase) / (right_phase - left_phase)
    return (1.0 - weight) * left_value + weight * right_value


def compose_yaw_follow_pose(
    marker_translation,
    reference_offset,
    fixed_rpy_degrees,
    marker_yaw_degrees,
    reference_marker_yaw_degrees,
    gripper_yaw_symmetry_degrees=360.0,
    rotate_xy_with_marker=True,
):
    """Follow marker yaw, choosing the nearest equivalent gripper yaw."""
    yaw_delta = wrap_degrees(
        marker_yaw_degrees - reference_marker_yaw_degrees
    )
    offset = np.asarray(reference_offset, dtype=np.float64)
    if rotate_xy_with_marker:
        angle = math.radians(yaw_delta)
        cosine, sine = math.cos(angle), math.sin(angle)
        rotated_offset = np.array([
            cosine * offset[0] - sine * offset[1],
            sine * offset[0] + cosine * offset[1],
            offset[2],
        ])
    else:
        # Hand-taught offsets include base-frame calibration error and must
        # not swing to the opposite side when the centered marker is reversed.
        rotated_offset = offset.copy()
    translation = np.asarray(marker_translation) + rotated_offset
    roll, pitch, reference_grasp_yaw = fixed_rpy_degrees
    symmetry = float(gripper_yaw_symmetry_degrees)
    orientation_delta = (
        (yaw_delta + symmetry / 2.0) % symmetry - symmetry / 2.0
    )
    rotation = quaternion_from_rpy_degrees(
        roll,
        pitch,
        reference_grasp_yaw + orientation_delta,
    )
    return translation, rotation, yaw_delta


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
        self.marker_translation_correction = self._vector_parameter(
            'marker_translation_correction_xyz_m', 3
        )
        self.container_offset = self._vector_parameter(
            'container_offset_xyz_m', 3
        )
        self.grasp_offset = self._vector_parameter('grasp_offset_xyz_m', 3)
        self.grasp_rpy = self._vector_parameter('grasp_offset_rpy_deg', 3)
        self.grasp_rotation = quaternion_from_rpy_degrees(*self.grasp_rpy)
        self.reference_marker_yaw = float(
            self.get_parameter('reference_marker_yaw_deg').value
        )
        self.auto_layer_selection = bool(
            self.get_parameter('auto_layer_selection_enabled').value
        )
        self.layer_max_z_error = float(
            self.get_parameter('layer_max_z_error_m').value
        )
        self.layer_profiles = []
        for layer in range(1, 4):
            rpy = self._vector_parameter(
                f'layer{layer}_grasp_offset_rpy_deg', 3
            )
            self.layer_profiles.append({
                'marker_z': float(
                    self.get_parameter(f'layer{layer}_marker_z_m').value
                ),
                'offset': self._vector_parameter(
                    f'layer{layer}_grasp_offset_xyz_m', 3
                ),
                'rpy': rpy,
                'rotation': quaternion_from_rpy_degrees(*rpy),
                'reference_yaw': float(self.get_parameter(
                    f'layer{layer}_reference_marker_yaw_deg'
                ).value),
            })
        self.layer3_angle90_enabled = bool(
            self.get_parameter('layer3_angle90_enabled').value
        )
        self.layer3_angle90_offset = self._vector_parameter(
            'layer3_angle90_grasp_offset_xyz_m', 3
        )
        self.layer3_angle90_reference_yaw = float(
            self.get_parameter('layer3_angle90_reference_marker_yaw_deg').value
        )
        self.layer3_multi_angle_enabled = bool(
            self.get_parameter('layer3_multi_angle_enabled').value
        )
        self.layer3_calibration_yaws = np.asarray(
            self.get_parameter('layer3_calibration_yaws_deg').value,
            dtype=np.float64,
        )
        flat_offsets = np.asarray(
            self.get_parameter('layer3_calibration_offsets_xyz_m').value,
            dtype=np.float64,
        )
        if flat_offsets.size != self.layer3_calibration_yaws.size * 3:
            raise ValueError(
                'layer3 calibration offsets must contain three values per yaw'
            )
        self.layer3_calibration_offsets = flat_offsets.reshape((-1, 3))
        self.layer2_multi_angle_enabled = bool(
            self.get_parameter('layer2_multi_angle_enabled').value
        )
        self.layer2_calibration_yaws = np.asarray(
            self.get_parameter('layer2_calibration_yaws_deg').value,
            dtype=np.float64,
        )
        flat_layer2_offsets = np.asarray(
            self.get_parameter('layer2_calibration_offsets_xyz_m').value,
            dtype=np.float64,
        )
        if flat_layer2_offsets.size != self.layer2_calibration_yaws.size * 3:
            raise ValueError(
                'layer2 calibration offsets must contain three values per yaw'
            )
        self.layer2_calibration_offsets = flat_layer2_offsets.reshape((-1, 3))
        self.layer1_multi_angle_enabled = bool(
            self.get_parameter('layer1_multi_angle_enabled').value
        )
        self.layer1_calibration_yaws = np.asarray(
            self.get_parameter('layer1_calibration_yaws_deg').value,
            dtype=np.float64,
        )
        flat_layer1_offsets = np.asarray(
            self.get_parameter('layer1_calibration_offsets_xyz_m').value,
            dtype=np.float64,
        )
        if flat_layer1_offsets.size != self.layer1_calibration_yaws.size * 3:
            raise ValueError(
                'layer1 calibration offsets must contain three values per yaw'
            )
        self.layer1_calibration_offsets = flat_layer1_offsets.reshape((-1, 3))
        self.last_selected_layer = None
        self.max_yaw_spread = float(
            self.get_parameter('max_yaw_spread_deg').value
        )
        self.max_yaw_delta = float(
            self.get_parameter('max_container_yaw_delta_deg').value
        )
        self.gripper_yaw_symmetry = float(
            self.get_parameter('gripper_yaw_symmetry_deg').value
        )
        self.rotate_grasp_xy_with_marker = bool(
            self.get_parameter('rotate_grasp_xy_with_marker').value
        )
        self.force_vertical_gripper = bool(
            self.get_parameter('force_vertical_gripper').value
        )
        self.vertical_gripper_roll = float(
            self.get_parameter('vertical_gripper_roll_deg').value
        )
        self.vertical_gripper_pitch = float(
            self.get_parameter('vertical_gripper_pitch_deg').value
        )
        self.pregrasp_lift = float(
            self.get_parameter('pregrasp_lift_m').value
        )
        self.grasp_extra_depth = float(
            self.get_parameter('grasp_extra_depth_m').value
        )
        self.return_to_start_after_pick = bool(
            self.get_parameter('return_to_start_after_pick').value
        )
        self.return_joint_angles = self._vector_parameter(
            'return_joint_angles_deg', 6
        )
        self.approach_via_safe_joint_pose = bool(
            self.get_parameter('approach_via_safe_joint_pose').value
        )
        self.safe_approach_joint_angles = self._vector_parameter(
            'safe_approach_joint_angles_deg', 6
        )
        self.dual_view_sampling = bool(
            self.get_parameter('dual_view_marker_sampling_enabled').value
        )
        self.dual_view_fuse_translation = float(
            self.get_parameter(
                'dual_view_fuse_translation_threshold_m'
            ).value
        )
        self.dual_view_reject_translation = float(
            self.get_parameter(
                'dual_view_reject_translation_threshold_m'
            ).value
        )
        self.dual_view_fuse_yaw = float(
            self.get_parameter('dual_view_fuse_yaw_threshold_deg').value
        )
        self.dual_view_reject_yaw = float(
            self.get_parameter('dual_view_reject_yaw_threshold_deg').value
        )
        self.dual_view_new_weight = float(
            self.get_parameter('dual_view_new_pose_weight').value
        )
        if not 0.0 <= self.dual_view_new_weight <= 1.0:
            raise ValueError('dual_view_new_pose_weight must be in [0, 1]')
        self.refresh_marker_before_descent = bool(
            self.get_parameter('refresh_marker_before_descent').value
        )
        self.marker_refresh_timeout = float(
            self.get_parameter('marker_refresh_timeout_sec').value
        )
        self.marker_refresh_fallback = bool(
            self.get_parameter('marker_refresh_fallback_to_locked').value
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
        self.initial_marker_sampling = float(
            self.get_parameter('initial_marker_sampling_sec').value
        )
        self.use_all_marker_samples = bool(
            self.get_parameter('use_all_stable_marker_samples').value
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
        self.speed = int(self.get_parameter('speed').value)
        self.motion_timeout = float(
            self.get_parameter('motion_timeout_sec').value
        )
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
        if not 0.0 < self.gripper_yaw_symmetry <= 360.0:
            raise ValueError(
                'gripper_yaw_symmetry_deg must be in (0, 360]'
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
        self.marker_update_event = threading.Event()
        self.last_transform_stamp = None
        self.motion_lock = threading.Lock()
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motion_thread = None
        self.robot = None
        self.move_group_client = None
        self.cartesian_path_client = None
        self.execute_trajectory_client = None
        self.gripper_open_client = None
        self.gripper_close_client = None
        self.stop_robot_client = None
        self.current_moveit_goal = None
        self.moveit_goal_lock = threading.Lock()
        self.last_status_text = ''

        self.status_publisher = self.create_publisher(
            String, '/arm2/container_pick/status', 10
        )
        self.marker_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/marker_pose', 10
        )
        target_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.grasp_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/grasp_pose', target_qos
        )
        self.pregrasp_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/pregrasp_pose', target_qos
        )
        self.create_service(Trigger, '/arm2/pick_container', self.start_pick)
        self.create_service(
            Trigger, '/arm2/preview_pregrasp', self.preview_pregrasp
        )
        self.create_service(
            Trigger, '/arm2/move_to_pregrasp', self.start_pregrasp_test
        )
        self.create_service(Trigger, '/arm2/stop_pick', self.stop_pick)
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
        self.declare_parameter('base_frame', 'arm2/base_link')
        self.declare_parameter('marker_frame', 'arm2/container_marker')
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('motion_backend', 'direct')
        self.declare_parameter('allow_full_pick', False)
        self.declare_parameter('offsets_configured', False)
        self.declare_parameter('use_marker_rotation_for_grasp', False)
        self.declare_parameter('grasp_orientation_mode', 'fixed')
        self.declare_parameter(
            'marker_translation_correction_xyz_m', [0.0, 0.0, 0.0]
        )
        self.declare_parameter('container_offset_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_offset_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_offset_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('reference_marker_yaw_deg', 0.0)
        self.declare_parameter('auto_layer_selection_enabled', False)
        self.declare_parameter('layer_max_z_error_m', 0.025)
        for layer in range(1, 4):
            self.declare_parameter(f'layer{layer}_marker_z_m', 0.0)
            self.declare_parameter(
                f'layer{layer}_grasp_offset_xyz_m', [0.0, 0.0, 0.0]
            )
            self.declare_parameter(
                f'layer{layer}_grasp_offset_rpy_deg', [0.0, 0.0, 0.0]
            )
            self.declare_parameter(
                f'layer{layer}_reference_marker_yaw_deg', 0.0
            )
        self.declare_parameter('layer3_angle90_enabled', False)
        self.declare_parameter(
            'layer3_angle90_grasp_offset_xyz_m', [0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            'layer3_angle90_reference_marker_yaw_deg', 90.0
        )
        self.declare_parameter('layer3_multi_angle_enabled', False)
        self.declare_parameter(
            'layer3_calibration_yaws_deg', [0.0, 90.0]
        )
        self.declare_parameter(
            'layer3_calibration_offsets_xyz_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('layer2_multi_angle_enabled', False)
        self.declare_parameter(
            'layer2_calibration_yaws_deg', [0.0, 90.0]
        )
        self.declare_parameter(
            'layer2_calibration_offsets_xyz_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('layer1_multi_angle_enabled', False)
        self.declare_parameter(
            'layer1_calibration_yaws_deg', [0.0, 90.0]
        )
        self.declare_parameter(
            'layer1_calibration_offsets_xyz_m',
            [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        )
        self.declare_parameter('max_yaw_spread_deg', 8.0)
        self.declare_parameter('max_container_yaw_delta_deg', 90.0)
        self.declare_parameter('gripper_yaw_symmetry_deg', 360.0)
        self.declare_parameter('rotate_grasp_xy_with_marker', True)
        self.declare_parameter('force_vertical_gripper', False)
        self.declare_parameter('vertical_gripper_roll_deg', 180.0)
        self.declare_parameter('vertical_gripper_pitch_deg', 0.0)
        self.declare_parameter('pregrasp_lift_m', 0.08)
        self.declare_parameter('grasp_extra_depth_m', 0.0)
        self.declare_parameter('return_to_start_after_pick', True)
        self.declare_parameter(
            'return_joint_angles_deg',
            [0.0, 45.0, -85.0, -25.0, 0.0, 45.0],
        )
        self.declare_parameter('approach_via_safe_joint_pose', False)
        self.declare_parameter(
            'safe_approach_joint_angles_deg',
            [0.0, 45.0, -85.0, -25.0, 0.0, 45.0],
        )
        self.declare_parameter('dual_view_marker_sampling_enabled', False)
        self.declare_parameter(
            'dual_view_fuse_translation_threshold_m', 0.005
        )
        self.declare_parameter(
            'dual_view_reject_translation_threshold_m', 0.015
        )
        self.declare_parameter('dual_view_fuse_yaw_threshold_deg', 3.0)
        self.declare_parameter('dual_view_reject_yaw_threshold_deg', 10.0)
        self.declare_parameter('dual_view_new_pose_weight', 0.25)
        self.declare_parameter('refresh_marker_before_descent', True)
        self.declare_parameter('marker_refresh_timeout_sec', 2.0)
        self.declare_parameter('marker_refresh_fallback_to_locked', True)
        self.declare_parameter(
            'pregrasp_test_keep_current_orientation', True
        )
        self.declare_parameter('lift_after_pick_m', 0.08)
        self.declare_parameter('minimum_lift_after_pick_m', 0.05)
        self.declare_parameter('lift_search_step_m', 0.02)
        self.declare_parameter('minimum_stable_samples', 5)
        self.declare_parameter('initial_marker_sampling_sec', 0.0)
        self.declare_parameter('use_all_stable_marker_samples', False)
        self.declare_parameter('max_translation_std_m', 0.005)
        self.declare_parameter('max_rotation_spread_deg', 5.0)
        self.declare_parameter('dry_run_max_rotation_spread_deg', 25.0)
        self.declare_parameter('max_marker_age_sec', 2.0)
        self.declare_parameter('workspace_min_xyz_m', [-0.28, -0.28, 0.02])
        self.declare_parameter('workspace_max_xyz_m', [0.28, 0.28, 0.30])
        self.declare_parameter('serial_port', '/dev/jetcobot')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 10)
        self.declare_parameter('motion_timeout_sec', 15.0)
        self.declare_parameter('stabilization_timeout_sec', 12.0)
        self.declare_parameter('max_joint_delta_deg', 60.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 30)
        self.declare_parameter('moveit_group', 'arm_group')
        self.declare_parameter('moveit_ee_link', 'TCP')
        self.declare_parameter('moveit_position_tolerance_m', 0.005)
        self.declare_parameter('moveit_orientation_tolerance_deg', 5.0)
        self.declare_parameter('moveit_planning_time_sec', 5.0)
        self.declare_parameter('moveit_planning_attempts', 10)
        self.declare_parameter('moveit_velocity_scale', 0.1)
        self.declare_parameter('moveit_acceleration_scale', 0.1)
        self.declare_parameter('cartesian_max_step_m', 0.005)
        self.declare_parameter('cartesian_min_fraction', 0.98)
        self.declare_parameter('cartesian_max_speed_mps', 0.02)
        self.declare_parameter('cartesian_max_joint_jump_deg', 10.0)
        self.declare_parameter('moveit_state_settle_sec', 0.3)

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
        self.gripper_open_client = self.create_client(
            Trigger, '/arm2/gripper/open'
        )
        self.gripper_close_client = self.create_client(
            Trigger, '/arm2/gripper/close'
        )
        self.stop_robot_client = self.create_client(
            Trigger, '/arm2/stop_robot'
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
        except TransformException:
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
        self.marker_update_event.set()
        self.marker_pose_publisher.publish(
            self.make_pose(translation, rotation, transform.header.stamp)
        )

    def stable_marker_pose(self, max_rotation_spread=None, yaw_only=False):
        with self.history_lock:
            history = list(self.history)
        samples = (
            history if self.use_all_marker_samples
            else history[-self.minimum_samples:]
        )
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
        grasp_offset = self.grasp_offset
        grasp_rpy = self.grasp_rpy
        grasp_rotation = self.grasp_rotation
        reference_marker_yaw = self.reference_marker_yaw
        selected_layer = None
        if self.auto_layer_selection:
            index, z_error = select_layer_index(
                marker_translation[2],
                [profile['marker_z'] for profile in self.layer_profiles],
                self.layer_max_z_error,
            )
            if index is None:
                return None, (
                    'marker height does not match a taught layer: '
                    f'z={marker_translation[2]:.4f}m, '
                    f'nearest_error={z_error * 1000.0:.1f}mm'
                )
            profile = self.layer_profiles[index]
            grasp_offset = profile['offset']
            grasp_rpy = profile['rpy']
            grasp_rotation = profile['rotation']
            reference_marker_yaw = profile['reference_yaw']
            selected_layer = index + 1
            if selected_layer != self.last_selected_layer:
                self.get_logger().info(
                    f'AUTO LAYER: {selected_layer} '
                    f'(marker_z={marker_translation[2]:.4f}m, '
                    f'taught_z={profile["marker_z"]:.4f}m, '
                    f'error={z_error * 1000.0:.1f}mm)'
                )
                self.last_selected_layer = selected_layer
        if self.force_vertical_gripper:
            # Preserve the taught/marker-following yaw, but remove the
            # hand-taught roll/pitch tilt so descent is truly vertical.
            grasp_rpy = np.array([
                self.vertical_gripper_roll,
                self.vertical_gripper_pitch,
                float(grasp_rpy[2]),
            ])
            grasp_rotation = quaternion_from_rpy_degrees(*grasp_rpy)
        # Apply a measured correction in the robot base frame.  This is kept
        # separate from the marker-to-grasp offset so the physical grasp
        # geometry remains meaningful and the workspace guard stays active.
        marker_translation = (
            marker_translation + self.marker_translation_correction
        )
        # Convert the detected marker center into the physical container
        # reference point before applying the container-to-TCP grasp offset.
        marker_translation = marker_translation + self.container_offset
        if mode == 'marker_full':
            grasp_translation, grasp_rotation = compose_pose(
                marker_translation,
                marker_rotation,
                grasp_offset,
                grasp_rotation,
            )
        elif mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            if selected_layer == 1 and self.layer1_multi_angle_enabled:
                grasp_offset = interpolate_periodic_profiles(
                    marker_yaw,
                    self.layer1_calibration_yaws,
                    self.layer1_calibration_offsets,
                )
            elif selected_layer == 2 and self.layer2_multi_angle_enabled:
                grasp_offset = interpolate_periodic_profiles(
                    marker_yaw,
                    self.layer2_calibration_yaws,
                    self.layer2_calibration_offsets,
                )
            elif selected_layer == 3 and self.layer3_multi_angle_enabled:
                grasp_offset = interpolate_periodic_profiles(
                    marker_yaw,
                    self.layer3_calibration_yaws,
                    self.layer3_calibration_offsets,
                )
            elif selected_layer == 3 and self.layer3_angle90_enabled:
                grasp_offset = interpolate_half_turn_profile(
                    marker_yaw,
                    reference_marker_yaw,
                    self.layer3_angle90_reference_yaw,
                    grasp_offset,
                    self.layer3_angle90_offset,
                )
            grasp_translation, grasp_rotation, yaw_delta = (
                compose_yaw_follow_pose(
                    marker_translation,
                    grasp_offset,
                    grasp_rpy,
                    marker_yaw,
                    reference_marker_yaw,
                    self.gripper_yaw_symmetry,
                    self.rotate_grasp_xy_with_marker,
                )
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                return None, (
                    f'container yaw delta {yaw_delta:.2f}deg exceeds limit'
                )
        else:
            grasp_translation, grasp_rotation = compose_fixed_base_pose(
                marker_translation,
                grasp_offset,
                grasp_rotation,
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

    def wait_for_new_stable_targets(self, timeout=None, collection_sec=0.0):
        self.marker_update_event.clear()
        with self.history_lock:
            self.history.clear()
        wait_timeout = (
            self.stabilization_timeout if timeout is None else float(timeout)
        )
        collection_deadline = time.monotonic() + max(0.0, collection_sec)
        deadline = time.monotonic() + wait_timeout
        reason = 'no new marker samples'
        last_report_time = 0.0
        last_report_reason = ''
        while time.monotonic() < deadline:
            if self.stop_event.is_set():
                return None, 'pick stopped'
            if time.monotonic() >= collection_deadline:
                targets, reason = self.calculate_targets()
                if targets is not None:
                    return targets, reason
            else:
                with self.history_lock:
                    sample_count = len(self.history)
                reason = (
                    f'collecting samples for {collection_sec:.1f}s: '
                    f'{sample_count} received'
                )
            now = time.monotonic()
            if (
                reason != last_report_reason
                or now - last_report_time >= 1.0
            ):
                self.publish_status(f'PICK: waiting for marker: {reason}')
                last_report_reason = reason
                last_report_time = now
            # Recalculate as soon as a genuinely new TF sample arrives.  The
            # short timeout also keeps stop requests responsive if tracking
            # is lost completely.
            remaining = max(0.0, deadline - time.monotonic())
            self.marker_update_event.wait(min(0.05, remaining))
            self.marker_update_event.clear()
        return None, f'marker did not stabilize: {reason}'

    def in_workspace(self, translation):
        return bool(np.all(translation >= self.workspace_min) and np.all(
            translation <= self.workspace_max
        ))

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
                'Full pick is locked; use /arm2/move_to_pregrasp'
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
        startup_targets = None
        startup_layer = None
        if self.dual_view_sampling:
            self.publish_status(
                'PICK: sampling marker for 2.0s at new startup pose'
            )
            startup_targets, reason = self.wait_for_new_stable_targets(
                collection_sec=self.initial_marker_sampling
            )
            startup_layer = self.last_selected_layer
            if startup_targets is None:
                self.publish_status(
                    f'PICK FAILED: startup-view marker unavailable: {reason}'
                )
                return
        if self.approach_via_safe_joint_pose:
            self.publish_status(
                'PICK: moving to calibrated camera pose before sampling'
            )
            try:
                self.move_to_joint_angles(self.safe_approach_joint_angles)
            except Exception as exc:
                self._stop_active_motion()
                self.publish_status(
                    f'PICK FAILED: calibrated camera pose unavailable: {exc}'
                )
                if not self.stop_event.is_set():
                    try:
                        self.move_to_joint_angles(self.return_joint_angles)
                    except Exception as return_exc:
                        self.get_logger().error(
                            f'Startup recovery failed: {return_exc}'
                        )
                return
        self.publish_status(
            'PICK: sampling marker for 2.0s at calibrated camera pose'
        )
        targets, reason = self.wait_for_new_stable_targets(
            collection_sec=self.initial_marker_sampling
        )
        calibrated_layer = self.last_selected_layer
        if targets is None:
            self.publish_status(f'PICK FAILED: {reason}')
            self._return_to_startup_after_sampling_failure()
            return
        if startup_targets is not None:
            targets, reason = self._validate_and_fuse_dual_view_targets(
                startup_targets,
                startup_layer,
                targets,
                calibrated_layer,
            )
            if targets is None:
                self.publish_status(f'PICK FAILED: {reason}')
                self._return_to_startup_after_sampling_failure()
                return
        self.publish_status('PICK: fresh marker target locked')
        self.execute_pick(targets)

    def _return_to_startup_after_sampling_failure(self):
        """Return from the calibrated camera pose after a rejected sample."""
        if self.stop_event.is_set() or not self.approach_via_safe_joint_pose:
            return
        try:
            self.move_to_joint_angles(self.return_joint_angles)
        except Exception as exc:
            self.get_logger().error(f'Startup recovery failed: {exc}')

    @staticmethod
    def _pose_xyz(pose):
        return np.array([
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        ], dtype=np.float64)

    @staticmethod
    def _pose_yaw(pose):
        orientation = pose.pose.orientation
        return quaternion_to_rpy_degrees([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])[2]

    def _validate_and_fuse_dual_view_targets(
        self,
        startup_targets,
        startup_layer,
        calibrated_targets,
        calibrated_layer,
    ):
        """Validate two camera views and conservatively fuse their positions."""
        if startup_layer != calibrated_layer:
            return None, (
                'dual-view layer mismatch: '
                f'startup={startup_layer}, calibrated={calibrated_layer}'
            )
        startup_grasp, startup_pregrasp = startup_targets
        calibrated_grasp, calibrated_pregrasp = calibrated_targets
        translation_delta = float(np.linalg.norm(
            self._pose_xyz(startup_grasp) - self._pose_xyz(calibrated_grasp)
        ))
        yaw_delta = abs(wrap_degrees(
            self._pose_yaw(startup_grasp) - self._pose_yaw(calibrated_grasp)
        ))
        self.get_logger().info(
            'DUAL VIEW: '
            f'layer={calibrated_layer}, position_delta='
            f'{translation_delta * 1000.0:.1f}mm, yaw_delta={yaw_delta:.1f}deg'
        )
        if (
            translation_delta > self.dual_view_reject_translation
            or yaw_delta > self.dual_view_reject_yaw
        ):
            return None, (
                'dual-view disagreement exceeds safety limit: '
                f'{translation_delta * 1000.0:.1f}mm, {yaw_delta:.1f}deg'
            )
        if (
            translation_delta <= self.dual_view_fuse_translation
            and yaw_delta <= self.dual_view_fuse_yaw
        ):
            result = []
            for startup_pose, calibrated_pose in (
                (startup_grasp, calibrated_grasp),
                (startup_pregrasp, calibrated_pregrasp),
            ):
                fused = copy.deepcopy(calibrated_pose)
                xyz = (
                    self.dual_view_new_weight * self._pose_xyz(startup_pose)
                    + (1.0 - self.dual_view_new_weight)
                    * self._pose_xyz(calibrated_pose)
                )
                fused.pose.position.x = float(xyz[0])
                fused.pose.position.y = float(xyz[1])
                fused.pose.position.z = float(xyz[2])
                result.append(fused)
            self.publish_status(
                'PICK: dual-view position fused '
                f'(new={self.dual_view_new_weight:.0%}, calibrated='
                f'{1.0 - self.dual_view_new_weight:.0%})'
            )
            return tuple(result), 'dual-view targets fused'
        self.publish_status(
            'PICK: dual-view difference is moderate; using calibrated view'
        )
        return calibrated_targets, 'calibrated-view targets selected'

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
        grasp, pregrasp = targets
        grasp_translation = np.array([
            grasp.pose.position.x,
            grasp.pose.position.y,
            grasp.pose.position.z,
        ])
        if not self.in_workspace(grasp_translation):
            response.success = False
            response.message = (
                f'grasp target outside workspace: {grasp_translation}'
            )
            return response
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
        grasp_translation = np.array([
            grasp.pose.position.x,
            grasp.pose.position.y,
            grasp.pose.position.z,
        ])
        if not self.in_workspace(grasp_translation):
            response.success = False
            response.message = (
                f'grasp outside workspace: {grasp_translation}'
            )
            return response
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
        return_completed = False
        pick_failure = None
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
                self.publish_status(
                    'PICK: refreshing marker at pregrasp for fine correction'
                )
                refreshed, reason = self.wait_for_new_stable_targets(
                    timeout=self.marker_refresh_timeout,
                    collection_sec=min(1.0, self.marker_refresh_timeout),
                )
                if refreshed is None:
                    if not self.marker_refresh_fallback:
                        raise RuntimeError(
                            f'failed to refresh marker pose: {reason}'
                        )
                    self.get_logger().warning(
                        'Pregrasp marker refresh unavailable; using the '
                        f'previous locked target: {reason}'
                    )
                    self.publish_status(
                        'PICK: refresh fallback; using locked base target'
                    )
                else:
                    grasp, refreshed_pregrasp = refreshed
                    self.publish_status(
                        'PICK: pregrasp marker correction applied'
                    )
                    # Apply the new XY estimate while still safely above the
                    # container.  The following Cartesian grasp segment can
                    # then remain a straight vertical descent.
                    self.publish_status(
                        'PICK: aligning XY at refreshed pregrasp'
                    )
                    self.move_cartesian_to_pose(refreshed_pregrasp)
            else:
                self.publish_status(
                    'PICK: marker refresh skipped; using locked base target'
                )
            self.publish_status('PICK: descending to grasp pose')
            self.move_cartesian_to_pose(grasp)
            self.publish_status('PICK: closing gripper')
            self.command_gripper(open_gripper=False)
            self.publish_status('PICK: finding vertical lift path')
            self.move_adaptive_cartesian_lift(grasp)
            if self.return_to_start_after_pick:
                self.publish_status('PICK: returning to startup position')
                self.move_to_joint_angles(self.return_joint_angles)
                return_completed = True
        except Exception as exc:
            pick_failure = str(exc)
            self._stop_active_motion()
        finally:
            # A normal stage failure is different from an operator STOP.
            # Recover to the startup pose after both successful and failed
            # picks, but never restart motion after an explicit stop request.
            if (
                self.return_to_start_after_pick
                and not return_completed
                and not self.stop_event.is_set()
            ):
                self.publish_status(
                    'PICK: recovery return to startup position'
                )
                try:
                    self.move_to_joint_angles(self.return_joint_angles)
                    return_completed = True
                except Exception as return_exc:
                    detail = f'startup return failed: {return_exc}'
                    pick_failure = (
                        f'{pick_failure}; {detail}'
                        if pick_failure else detail
                    )
            if pick_failure is not None:
                suffix = (
                    '; startup position restored'
                    if return_completed else ''
                )
                self.publish_status(f'PICK FAILED: {pick_failure}{suffix}')
            else:
                self.publish_status('PICK: completed')
            self.motion_lock.release()

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

    def move_to_pose(self, pose, keep_current_orientation=False):
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
        with self.serial_lock:
            current = self.robot.get_angles()
            target_orientation = coords[3:].copy()
            if keep_current_orientation:
                current_coords = self.robot.get_coords()
                if not isinstance(current_coords, (list, tuple)) or len(
                    current_coords
                ) != 6:
                    raise RuntimeError(
                        f'Failed to read current TCP pose: {current_coords}'
                    )
                coords[3:] = [float(value) for value in current_coords[3:]]
            solution = self.robot.solve_inv_kinematics(coords, current)
            if keep_current_orientation and not (
                isinstance(solution, (list, tuple)) and len(solution) == 6
            ):
                coords[3:] = target_orientation
                self.get_logger().warning(
                    'IK failed with current TCP orientation; retrying the '
                    'taught grasp orientation at pregrasp'
                )
                solution = self.robot.solve_inv_kinematics(coords, current)
        if not isinstance(solution, (list, tuple)) or len(solution) != 6:
            raise RuntimeError(f'IK failed for target {coords}')
        if not isinstance(current, (list, tuple)) or len(current) != 6:
            raise RuntimeError(f'Failed to read current joints: {current}')
        deltas = [
            abs(float(target) - float(start))
            for target, start in zip(solution, current)
        ]
        if max(deltas) > self.max_joint_delta:
            raise RuntimeError(
                f'IK branch jump rejected: deltas={np.round(deltas, 1).tolist()}'
            )
        for index, (angle, limits) in enumerate(zip(solution, JOINT_LIMITS_DEG)):
            if not limits[0] <= float(angle) <= limits[1]:
                raise RuntimeError(f'IK J{index + 1} outside limits: {angle}')
        with self.serial_lock:
            self.robot.send_angles([float(value) for value in solution], self.speed)
        deadline = time.monotonic() + self.motion_timeout
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                raise RuntimeError('pick stopped')
            with self.serial_lock:
                measured = self.robot.get_angles()
            if isinstance(measured, (list, tuple)) and len(measured) == 6:
                error = max(abs(float(a) - float(b)) for a, b in zip(measured, solution))
                if error <= 2.0:
                    return
        raise RuntimeError('robot motion timed out')

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

    def move_to_joint_angles(self, angles_degrees):
        """Return through a collision-checked joint-space MoveIt plan."""
        if self.stop_event.is_set():
            raise RuntimeError('pick stopped')
        angles = np.asarray(angles_degrees, dtype=np.float64)
        if angles.shape != (6,):
            raise RuntimeError('return joint target must contain six angles')
        for index, (angle, limits) in enumerate(zip(angles, JOINT_LIMITS_DEG)):
            if not limits[0] <= float(angle) <= limits[1]:
                raise RuntimeError(
                    f'return J{index + 1} outside limits: {angle}'
                )
        if self.motion_backend != 'moveit':
            with self.serial_lock:
                self.robot.send_angles(angles.tolist(), self.speed)
            return
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /move_action server is unavailable')

        constraints = Constraints()
        tolerance = math.radians(2.0)
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
        request.max_acceleration_scaling_factor = self.moveit_acceleration_scale
        request.start_state.is_diff = True
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        goal_future = self.move_group_client.send_goal_async(goal)
        self._wait_future(goal_future, self.moveit_planning_time + 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('MoveIt rejected the startup return goal')
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
                raise RuntimeError('MoveIt returned no startup return result')
            result = wrapped_result.result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                detail = result.error_code.message or 'no detail'
                raise RuntimeError(
                    'startup return failed: '
                    f'code={result.error_code.val}, message={detail}'
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
        # Marker-derived targets are locked in the base frame.  Their camera
        # timestamp becomes stale during approach and must not be reused for
        # later MoveIt TF lookups.
        target.header.stamp = Time().to_msg()
        target_rpy = quaternion_to_rpy_degrees([
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ])
        self.get_logger().info(
            'MoveIt target: '
            f'frame={target.header.frame_id}, '
            f'xyz_m=['
            f'{target.pose.position.x:.4f}, '
            f'{target.pose.position.y:.4f}, '
            f'{target.pose.position.z:.4f}], '
            f'rpy_deg=['
            f'{target_rpy[0]:.1f}, {target_rpy[1]:.1f}, '
            f'{target_rpy[2]:.1f}]'
        )

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

        request = GetCartesianPath.Request()
        request.header = copy.deepcopy(target.header)
        # The waypoint is already expressed in the fixed base frame.  Ask TF
        # for the latest state instead of the old marker acquisition time.
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

        path_future = self.cartesian_path_client.call_async(request)
        self._wait_future(path_future, self.moveit_planning_time + 5.0)
        response = path_future.result()
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
        if response.fraction < self.cartesian_min_fraction:
            raise CartesianPlanningError(
                'Cartesian path rejected: '
                f'fraction={response.fraction:.3f} is below '
                f'{self.cartesian_min_fraction:.3f}'
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

    def move_adaptive_cartesian_lift(self, grasp):
        """Lift vertically first, then reconfigure joints for full height."""
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
                if distance + 1e-9 < self.lift_after_pick:
                    full_lift = copy.deepcopy(grasp)
                    full_lift.pose.position.z += self.lift_after_pick
                    self.publish_status(
                        'PICK: vertical IK limit reached; planning joint-space '
                        f'lift to {self.lift_after_pick * 100.0:.1f} cm'
                    )
                    try:
                        # MoveIt may change the elbow/wrist branch while
                        # retaining the vertical TCP target.  This gets past
                        # the local Cartesian IK limit after initial clearance.
                        self.move_to_pose(full_lift)
                        self.publish_status(
                            'PICK: full lift reached through planned path'
                        )
                    except RuntimeError as exc:
                        # The container already has verified vertical
                        # clearance.  Startup return is also collision planned
                        # and remains the safe recovery path.
                        self.get_logger().warning(
                            'Full-height lift plan was not feasible; '
                            f'returning from {distance * 100.0:.1f} cm '
                            f'clearance instead: {exc}'
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
                JointState, '/arm2/joint_states', 10
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
