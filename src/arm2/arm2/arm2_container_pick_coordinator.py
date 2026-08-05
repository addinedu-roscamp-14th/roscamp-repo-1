"""Track a container marker and optionally execute a guarded pick sequence."""

from collections import deque
import copy
import math
import threading
import time

from builtin_interfaces.msg import Duration
from control_msgs.action import FollowJointTrajectory
from geometry_msgs.msg import PoseStamped
from arm2_interfaces.srv import TransferById
import numpy as np
from pymycobot.mycobot280 import MyCobot280
from rcl_interfaces.msg import SetParametersResult
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
from trajectory_msgs.msg import JointTrajectoryPoint

from .arm2_joint_limits import JOINT_LIMITS_DEG

try:
    from moveit_msgs.action import ExecuteTrajectory, MoveGroup
    from moveit_msgs.msg import (
        Constraints,
        JointConstraint,
        MoveItErrorCodes,
        OrientationConstraint,
        PositionConstraint,
    )
    from moveit_msgs.srv import GetCartesianPath, GetPositionIK
except ImportError:
    ExecuteTrajectory = None
    GetCartesianPath = None
    GetPositionIK = None
    MoveGroup = None


JOINT_NAMES = [f'{index}_Joint' for index in range(1, 7)]


class CartesianPlanningError(RuntimeError):
    """A Cartesian request failed, optionally after safe partial segments."""

    def __init__(self, message, executed_segments=0):
        super().__init__(message)
        self.executed_segments = executed_segments


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


def apply_base_frame_correction(translation, correction):
    """Apply an empirical correction without rotating it with the marker."""
    return (
        np.asarray(translation, dtype=np.float64)
        + np.asarray(correction, dtype=np.float64)
    )


def apply_marker_yaw_correction(
    translation,
    correction,
    marker_yaw_degrees,
):
    """Apply an XYZ correction expressed in the marker's yaw frame."""
    correction = np.asarray(correction, dtype=np.float64)
    if correction.shape != (3,):
        raise ValueError('marker correction must contain three values')
    angle = math.radians(float(marker_yaw_degrees))
    cosine, sine = math.cos(angle), math.sin(angle)
    rotated = np.array([
        cosine * correction[0] - sine * correction[1],
        sine * correction[0] + cosine * correction[1],
        correction[2],
    ])
    return np.asarray(translation, dtype=np.float64) + rotated


def bounded_visual_servo_step(
    error_xy,
    yaw_error_degrees,
    xy_gain,
    yaw_gain,
    max_xy_step,
    max_yaw_step_degrees,
):
    """Return gain-scaled XY/yaw corrections with independent hard limits."""
    xy_step = np.asarray(error_xy, dtype=np.float64) * float(xy_gain)
    norm = float(np.linalg.norm(xy_step))
    if norm > max_xy_step:
        xy_step *= float(max_xy_step) / norm
    yaw_step = float(yaw_error_degrees) * float(yaw_gain)
    yaw_step = float(np.clip(
        yaw_step,
        -float(max_yaw_step_degrees),
        float(max_yaw_step_degrees),
    ))
    return xy_step, yaw_step


def visual_servo_within_tolerance(
    error_xy,
    yaw_error_degrees,
    xy_tolerance,
    yaw_tolerance_degrees,
):
    """Return whether XY and yaw errors satisfy refinement tolerances."""
    return (
        float(np.max(np.abs(error_xy))) <= float(xy_tolerance)
        and abs(float(yaw_error_degrees)) <= float(yaw_tolerance_degrees)
    )


def wrap_degrees(angle):
    """Wrap an angle to [-180, 180)."""
    return (float(angle) + 180.0) % 360.0 - 180.0


def duration_message(seconds):
    """Convert positive fractional seconds to a ROS Duration message."""
    seconds = float(seconds)
    whole_seconds = int(seconds)
    nanoseconds = int(round((seconds - whole_seconds) * 1_000_000_000))
    if nanoseconds >= 1_000_000_000:
        whole_seconds += 1
        nanoseconds -= 1_000_000_000
    return Duration(sec=whole_seconds, nanosec=nanoseconds)


def trajectory_joint_travel_degrees(start, trajectory, joint_name):
    """Return cumulative travel for one named trajectory joint."""
    names = list(trajectory.joint_trajectory.joint_names)
    if joint_name not in names:
        return math.inf
    joint_index = names.index(joint_name)
    previous = float(start)
    travel = 0.0
    for point in trajectory.joint_trajectory.points:
        if len(point.positions) <= joint_index:
            return math.inf
        following = float(point.positions[joint_index])
        travel += abs(math.degrees(following - previous))
        previous = following
    return travel


def wrap_symmetric_degrees(angle, symmetry_degrees):
    """Wrap an angle using the rotational symmetry of the grasped object."""
    period = float(symmetry_degrees)
    if period <= 0.0 or period > 360.0:
        raise ValueError('yaw symmetry must be in the range (0, 360]')
    return (float(angle) + period / 2.0) % period - period / 2.0


def nearest_symmetric_yaw_degrees(
    target_yaw, current_yaw, symmetry_degrees
):
    """Choose the target-equivalent yaw closest to the current TCP yaw."""
    delta = wrap_symmetric_degrees(
        float(target_yaw) - float(current_yaw),
        symmetry_degrees,
    )
    return wrap_degrees(float(current_yaw) + delta)


def symmetric_marker_yaw_degrees(
    marker_yaw,
    reference_marker_yaw,
    symmetry_degrees,
):
    """Map equivalent marker headings onto one correction-frame heading."""
    return float(reference_marker_yaw) + wrap_symmetric_degrees(
        float(marker_yaw) - float(reference_marker_yaw),
        symmetry_degrees,
    )


def compose_yaw_follow_pose(
    marker_translation,
    reference_offset,
    fixed_rpy_degrees,
    marker_yaw_degrees,
    reference_marker_yaw_degrees,
    rotate_offset=True,
    yaw_symmetry_degrees=360.0,
):
    """Follow marker yaw while retaining the taught vertical roll/pitch."""
    yaw_delta = wrap_symmetric_degrees(
        marker_yaw_degrees - reference_marker_yaw_degrees,
        yaw_symmetry_degrees,
    )
    angle = math.radians(yaw_delta)
    cosine, sine = math.cos(angle), math.sin(angle)
    offset = np.asarray(reference_offset, dtype=np.float64)
    if rotate_offset:
        offset = np.array([
            cosine * offset[0] - sine * offset[1],
            sine * offset[0] + cosine * offset[1],
            offset[2],
        ])
    translation = np.asarray(marker_translation) + offset
    roll, pitch, reference_grasp_yaw = fixed_rpy_degrees
    rotation = quaternion_from_rpy_degrees(
        roll,
        pitch,
        reference_grasp_yaw + yaw_delta,
    )
    return translation, rotation, yaw_delta


def apply_vertical_pick_offsets(
    nominal_grasp,
    pregrasp_lift,
    extra_depth,
    stop_above=0.0,
):
    """Keep pregrasp fixed while adjusting only the final grasp height."""
    if pregrasp_lift < 0.0 or extra_depth < 0.0 or stop_above < 0.0:
        raise ValueError('vertical pick offsets must be non-negative')
    if stop_above > pregrasp_lift:
        raise ValueError('grasp stop-above distance exceeds pregrasp lift')
    nominal = np.asarray(nominal_grasp, dtype=np.float64)
    grasp = nominal.copy()
    grasp[2] += stop_above - extra_depth
    pregrasp = nominal.copy()
    pregrasp[2] += pregrasp_lift
    return grasp, pregrasp


def calculate_stack_poses(
    source_marker,
    destination_marker,
    grasp_translation,
    container_height,
    approach_clearance,
    xy_offset,
):
    """Calculate TCP release and approach positions for marker-on-marker stacking."""
    if container_height <= 0.0 or approach_clearance <= 0.0:
        raise ValueError('stack height and approach clearance must be positive')
    source = np.asarray(source_marker, dtype=np.float64)
    destination = np.asarray(destination_marker, dtype=np.float64)
    grasp = np.asarray(grasp_translation, dtype=np.float64)
    offset = np.asarray(xy_offset, dtype=np.float64)
    if source.shape != (3,) or destination.shape != (3,) or grasp.shape != (3,):
        raise ValueError('stack pose inputs must be XYZ vectors')
    if offset.shape != (2,):
        raise ValueError('stack XY offset must contain two values')

    marker_to_tcp = grasp - source
    release = destination + marker_to_tcp
    release[:2] += offset
    release[2] += container_height
    approach = release.copy()
    approach[2] += approach_clearance
    return release, approach


def calculate_heading_aligned_stack_poses(
    destination_marker,
    grasp_offset,
    grasp_rpy_degrees,
    destination_yaw_degrees,
    reference_marker_yaw_degrees,
    container_height,
    approach_clearance,
    extra_depth,
    xy_offset,
    yaw_symmetry_degrees=360.0,
    z_offset=0.0,
):
    """Place the held container with its heading aligned to the destination."""
    if container_height <= 0.0 or approach_clearance <= 0.0:
        raise ValueError('stack height and approach clearance must be positive')
    if extra_depth < 0.0:
        raise ValueError('extra grasp depth must be non-negative')
    offset = np.asarray(xy_offset, dtype=np.float64)
    if offset.shape != (2,):
        raise ValueError('stack XY offset must contain two values')

    release, rotation, yaw_delta = compose_yaw_follow_pose(
        destination_marker,
        grasp_offset,
        grasp_rpy_degrees,
        destination_yaw_degrees,
        reference_marker_yaw_degrees,
        yaw_symmetry_degrees=yaw_symmetry_degrees,
    )
    release = np.asarray(release, dtype=np.float64)
    release[:2] += offset
    release[2] += container_height - extra_depth + float(z_offset)
    approach = release.copy()
    approach[2] += approach_clearance
    return release, approach, rotation, yaw_delta


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


def stack_layer_z_offset(base_offset, container_height, placed_count):
    """Return placement Z offset for the next zero-based stack layer."""
    if container_height <= 0.0:
        raise ValueError('container height must be positive')
    if placed_count < 0:
        raise ValueError('placed count must be non-negative')
    return float(base_offset) + float(container_height) * int(placed_count)


def grouped_marker_locks_satisfied(
    locked,
    required_indices,
    any_of_indices,
):
    """Require every source lock and at least one alternative target lock."""
    return (
        all(locked[index] is not None for index in required_indices)
        and any(locked[index] is not None for index in any_of_indices)
    )


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
    if requested_distance is None:
        return False
    shortfall = max(0.0, requested_distance * (1.0 - fraction))
    # Percentage thresholds are misleading for millimetre-scale settling:
    # 80% of a 4.5 mm correction leaves less than 1 mm. Permit only that
    # tightly bounded case; longer paths retain the normal fraction limit.
    if (
        0.0 < fraction < absolute_min_fraction
        and requested_distance <= max_shortfall
    ):
        return shortfall <= 0.001
    if fraction < absolute_min_fraction:
        return False
    return shortfall <= max_shortfall


def cartesian_segment_executable(
    fraction,
    requested_distance,
    minimum_fraction,
    minimum_progress,
    segment_count,
    maximum_segments,
):
    """Return whether a safe partial Cartesian result may be executed."""
    if requested_distance is None or segment_count >= maximum_segments:
        return False
    return (
        fraction >= minimum_fraction
        and requested_distance * fraction >= minimum_progress
    )


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
        super().__init__('arm2_container_pick_coordinator')
        self._declare_parameters()
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.marker_frame = str(self.get_parameter('marker_frame').value)
        self.stack_target_frame = str(
            self.get_parameter('stack_target_frame').value
        )
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
        self.rotate_grasp_offset_with_marker_yaw = bool(
            self.get_parameter(
                'rotate_grasp_offset_with_marker_yaw'
            ).value
        )
        self.grasp_offset = self._vector_parameter('grasp_offset_xyz_m', 3)
        self.pick_correction = self._vector_parameter(
            'pick_correction_xyz_m', 3
        )
        self.id_transfer_pick_correction = self._vector_parameter(
            'id_transfer_pick_correction_xyz_m', 3
        )
        self.place_correction = self._vector_parameter(
            'place_correction_xyz_m', 3
        )
        self.saved_destination_correction = self._vector_parameter(
            'saved_destination_correction_xyz_m', 3
        )
        self.id_transfer_correction = self._vector_parameter(
            'id_transfer_correction_xyz_m', 3
        )
        self.trailer_correction = self._vector_parameter(
            'trailer_correction_xyz_m', 3
        )
        self.grasp_rpy = self._vector_parameter('grasp_offset_rpy_deg', 3)
        self.grasp_rotation = quaternion_from_rpy_degrees(*self.grasp_rpy)
        self.reference_marker_yaw = float(
            self.get_parameter('reference_marker_yaw_deg').value
        )
        self.container_yaw_symmetry = float(
            self.get_parameter('container_yaw_symmetry_deg').value
        )
        self.prefer_current_gripper_yaw = bool(
            self.get_parameter('prefer_current_gripper_yaw').value
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
        self.grasp_stop_above = float(
            self.get_parameter('grasp_stop_above_m').value
        )
        self.refresh_marker_before_descent = bool(
            self.get_parameter('refresh_marker_before_descent').value
        )
        self.visual_servo_enabled = bool(
            self.get_parameter('visual_servo_enabled').value
        )
        self.visual_servo_samples = int(
            self.get_parameter('visual_servo_samples').value
        )
        self.visual_servo_xy_tolerance = float(
            self.get_parameter('visual_servo_xy_tolerance_m').value
        )
        self.visual_servo_yaw_tolerance = float(
            self.get_parameter('visual_servo_yaw_tolerance_deg').value
        )
        self.visual_servo_xy_gain = float(
            self.get_parameter('visual_servo_xy_gain').value
        )
        self.visual_servo_yaw_gain = float(
            self.get_parameter('visual_servo_yaw_gain').value
        )
        self.visual_servo_max_xy_step = float(
            self.get_parameter('visual_servo_max_xy_step_m').value
        )
        self.visual_servo_max_yaw_step = float(
            self.get_parameter('visual_servo_max_yaw_step_deg').value
        )
        self.visual_servo_max_initial_error = float(
            self.get_parameter(
                'visual_servo_max_initial_error_m'
            ).value
        )
        self.visual_servo_max_iterations = int(
            self.get_parameter('visual_servo_max_iterations').value
        )
        self.visual_servo_required_successes = int(
            self.get_parameter(
                'visual_servo_required_consecutive_successes'
            ).value
        )
        self.visual_servo_timeout = float(
            self.get_parameter('visual_servo_timeout_sec').value
        )
        self.visual_servo_marker_loss_timeout = float(
            self.get_parameter(
                'visual_servo_marker_loss_timeout_sec'
            ).value
        )
        self.visual_servo_settle = float(
            self.get_parameter('visual_servo_settle_sec').value
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
        self.bad_pick_branch_j2 = float(
            self.get_parameter('bad_pick_branch_j2_deg').value
        )
        self.bad_pick_branch_j3 = float(
            self.get_parameter('bad_pick_branch_j3_deg').value
        )
        self.bad_pick_branch_tolerance = float(
            self.get_parameter('bad_pick_branch_tolerance_deg').value
        )
        self.stack_container_height = float(
            self.get_parameter('stack_container_height_m').value
        )
        self.stack_approach_clearance = float(
            self.get_parameter('stack_approach_clearance_m').value
        )
        self.stack_minimum_approach_clearance = float(
            self.get_parameter(
                'stack_minimum_approach_clearance_m'
            ).value
        )
        self.stack_approach_search_step = float(
            self.get_parameter('stack_approach_search_step_m').value
        )
        self.stack_yaw_fallback_offsets = [
            float(value) for value in self.get_parameter(
                'stack_yaw_fallback_offsets_deg'
            ).value
        ]
        self.stack_xy_offset = self._vector_parameter(
            'stack_xy_offset_m', 2
        )
        self.stack_z_offset = float(
            self.get_parameter('stack_z_offset_m').value
        )
        self.max_stack_levels = int(
            self.get_parameter('max_stack_levels').value
        )
        self.stack_source_orientation_mode = str(
            self.get_parameter('stack_source_orientation_mode').value
        ).lower()
        self.stack_segmented_descent_min_fraction = float(
            self.get_parameter(
                'stack_segmented_descent_min_fraction'
            ).value
        )
        self.stack_segmented_descent_max_segments = int(
            self.get_parameter(
                'stack_segmented_descent_max_segments'
            ).value
        )
        self.stack_pose_goal_finish_max_distance = float(
            self.get_parameter(
                'stack_pose_goal_finish_max_distance_m'
            ).value
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
        self.move_group_action = str(
            self.get_parameter('move_group_action').value
        )
        self.execute_trajectory_action = str(
            self.get_parameter('execute_trajectory_action').value
        )
        self.compute_cartesian_path_service = str(
            self.get_parameter('compute_cartesian_path_service').value
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
        self.ik_timeout = float(
            self.get_parameter('ik_timeout_sec').value
        )
        self.motion_ik_timeout = float(
            self.get_parameter('motion_ik_timeout_sec').value
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
        self.stack_preflight_joint_margin = math.radians(float(
            self.get_parameter('stack_preflight_joint_margin_deg').value
        ))
        self.max_j6_trajectory_travel = float(
            self.get_parameter('max_j6_trajectory_travel_deg').value
        )
        self.scan_marker_pause = float(
            self.get_parameter('scan_marker_pause_sec').value
        )
        self.scan_timeout = float(
            self.get_parameter('scan_timeout_sec').value
        )
        self.prefer_z_last_motion = bool(
            self.get_parameter('prefer_z_last_motion').value
        )
        zone_j1_values = self._vector_parameter(
            'destination_zone_j1_deg', 3
        )
        self.destination_zone_j1 = {
            f'A-{index + 1}': math.radians(float(value))
            for index, value in enumerate(zone_j1_values)
        }
        self.j2_fallback_min_tcp_height = float(
            self.get_parameter('j2_fallback_min_tcp_height_m').value
        )
        self.j3_fallback_min_tcp_height = float(
            self.get_parameter('j3_fallback_min_tcp_height_m').value
        )
        self.release_verify_xy_tolerance = float(
            self.get_parameter('release_verify_xy_tolerance_m').value
        )
        self.release_verify_z_tolerance = float(
            self.get_parameter('release_verify_z_tolerance_m').value
        )
        self.release_verify_yaw_tolerance = float(
            self.get_parameter('release_verify_yaw_tolerance_deg').value
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
        if self.visual_servo_samples < 3:
            raise ValueError('visual_servo_samples must be at least 3')
        if (
            self.visual_servo_xy_tolerance <= 0.0
            or self.visual_servo_yaw_tolerance <= 0.0
        ):
            raise ValueError('visual servo tolerances must be positive')
        if not 0.0 < self.visual_servo_xy_gain <= 1.0:
            raise ValueError('visual_servo_xy_gain must be within (0, 1]')
        if not 0.0 < self.visual_servo_yaw_gain <= 1.0:
            raise ValueError('visual_servo_yaw_gain must be within (0, 1]')
        if (
            self.visual_servo_max_xy_step <= 0.0
            or self.visual_servo_max_yaw_step <= 0.0
            or self.visual_servo_max_initial_error <= 0.0
        ):
            raise ValueError('visual servo limits must be positive')
        if self.visual_servo_max_iterations < 1:
            raise ValueError('visual_servo_max_iterations must be positive')
        if self.visual_servo_required_successes < 1:
            raise ValueError(
                'visual_servo_required_consecutive_successes must be positive'
            )
        if (
            self.visual_servo_timeout <= 0.0
            or self.visual_servo_marker_loss_timeout <= 0.0
            or self.visual_servo_settle < 0.0
        ):
            raise ValueError('visual servo timing values are invalid')
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
        if self.ik_timeout <= 0.0:
            raise ValueError('ik_timeout_sec must be positive')
        if self.motion_ik_timeout <= 0.0:
            raise ValueError('motion_ik_timeout_sec must be positive')
        if self.stack_preflight_joint_margin < 0.0:
            raise ValueError(
                'stack_preflight_joint_margin_deg must be non-negative'
            )
        if self.max_j6_trajectory_travel <= 0.0:
            raise ValueError(
                'max_j6_trajectory_travel_deg must be positive'
            )
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
        if self.stack_container_height <= 0.0:
            raise ValueError('stack_container_height_m must be positive')
        if self.max_stack_levels < 1:
            raise ValueError('max_stack_levels must be positive')
        if self.stack_approach_clearance <= 0.0:
            raise ValueError('stack_approach_clearance_m must be positive')
        if any(
            not math.isfinite(offset) or abs(offset) > 180.0
            for offset in self.stack_yaw_fallback_offsets
        ):
            raise ValueError(
                'stack_yaw_fallback_offsets_deg values must be finite and '
                'within [-180, 180]'
            )
        if self.stack_source_orientation_mode not in ('fixed', 'marker_yaw'):
            raise ValueError(
                'stack_source_orientation_mode must be fixed or marker_yaw'
            )
        if not 0.0 < self.stack_segmented_descent_min_fraction < 1.0:
            raise ValueError(
                'stack_segmented_descent_min_fraction must be within (0, 1)'
            )
        if self.stack_segmented_descent_max_segments < 1:
            raise ValueError(
                'stack_segmented_descent_max_segments must be positive'
            )
        if self.stack_pose_goal_finish_max_distance <= 0.0:
            raise ValueError(
                'stack_pose_goal_finish_max_distance_m must be positive'
            )
        if not 0.0 < self.bad_pick_branch_tolerance <= 30.0:
            raise ValueError(
                'bad_pick_branch_tolerance_deg must be within (0, 30]'
            )
        if self.scan_marker_pause < 0.5:
            raise ValueError('scan_marker_pause_sec must be at least 0.5')
        if self.scan_timeout <= 0.0:
            raise ValueError('scan_timeout_sec must be positive')
        if not (
            self.workspace_min[2]
            <= self.j2_fallback_min_tcp_height
            <= self.workspace_max[2]
        ):
            raise ValueError(
                'j2_fallback_min_tcp_height_m must be inside Z workspace'
            )
        if not (
            self.workspace_min[2]
            <= self.j3_fallback_min_tcp_height
            <= self.workspace_max[2]
        ):
            raise ValueError(
                'j3_fallback_min_tcp_height_m must be inside Z workspace'
            )
        if (
            self.release_verify_xy_tolerance <= 0.0
            or self.release_verify_z_tolerance <= 0.0
            or self.release_verify_yaw_tolerance <= 0.0
        ):
            raise ValueError('release verification tolerances must be positive')
        lift_distance_candidates(
            self.stack_approach_clearance,
            self.stack_minimum_approach_clearance,
            self.stack_approach_search_step,
        )

        self.buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
        self.listener = TransformListener(self.buffer, self)
        self.history = deque(maxlen=max(100, self.minimum_samples * 3))
        self.stack_target_history = deque(
            maxlen=max(100, self.minimum_samples * 3)
        )
        self.source_frames = {
            0: self.marker_frame,
            1: 'arm2/container_marker_1',
            2: 'arm2/container_marker_2',
            3: 'arm2/container_marker_3',
            4: 'arm2/container_marker_4',
            5: 'arm2/container_marker_5',
            6: 'arm2/container_marker_6',
            7: 'arm2/container_marker_7',
            8: 'arm2/container_marker_8',
        }
        self.destination_ids = {
            'A-1-1': 11,
            'A-1-2': 12,
            'A-2-1': 13,
            'A-2-2': 14,
            'A-3-1': 15,
            'A-3-2': 16,
        }
        self.destination_frames = {
            marker_id: f'arm2/destination_marker_{marker_id}'
            for marker_id in self.destination_ids.values()
        }
        self.trailer_frames = {
            9: 'arm2/trailer_marker_9',
            10: 'arm2/trailer_marker_10',
        }
        tracked_frames = {
            **self.source_frames,
            **self.trailer_frames,
            **self.destination_frames,
        }
        history_size = max(100, self.minimum_samples * 3)
        self.marker_histories = {
            marker_id: (
                self.history if marker_id == 0 else deque(maxlen=history_size)
            )
            for marker_id in tracked_frames
        }
        self.marker_stamp_attributes = {}
        for marker_id in tracked_frames:
            attribute = f'last_marker_{marker_id}_stamp'
            self.marker_stamp_attributes[marker_id] = attribute
            setattr(self, attribute, None)
        self.history_lock = threading.Lock()
        self.last_transform_stamp = None
        self.last_stack_target_stamp = None
        self.last_stack_target_a2_stamp = None
        self.last_stack_target_a3_stamp = None
        self.saved_destination_poses = {}
        self.saved_marker_poses = {}
        self.saved_destination_stack_counts = {
            name: 0 for name in self.destination_ids
        }
        self.saved_destination_stack_counts['TRAILER'] = 0
        self.saved_destination_stack_counts.update({
            f'ID-{marker_id}': 0 for marker_id in range(9)
        })
        self.tracking_errors = {}
        # During scan-and-transfer, each marker becomes immutable as soon as
        # its stationary pose has been accepted.  The next scan command clears
        # these locks and starts a new acquisition.
        self.scan_locked_frames = set()
        self.motion_lock = threading.Lock()
        self.tracking_suspended = threading.Event()
        # Destination IDs 11-16 are only camera-sampled while the explicit
        # destination scan is running.  Once accepted, their base-frame poses
        # in saved_destination_poses are the sole source used by transfers.
        self.destination_scan_active = threading.Event()
        self.serial_lock = threading.Lock()
        self.stop_event = threading.Event()
        self.motion_thread = None
        self.stack_level_lock = threading.Lock()
        self.placed_stack_count = 0
        self.robot = None
        self.move_group_client = None
        self.cartesian_path_client = None
        self.position_ik_client = None
        self.execute_trajectory_client = None
        self.follow_joint_trajectory_client = None
        self.gripper_open_client = None
        self.gripper_close_client = None
        self.stop_robot_client = None
        self.return_home_client = None
        self.go_a1_client = None
        self.go_a2_client = None
        self.go_a3_client = None
        self.sweep_joint1_client = None
        self.pause_sweep_client = None
        self.resume_sweep_client = None
        self.scan_state_client = None
        self.current_moveit_goal = None
        self.moveit_goal_lock = threading.Lock()
        self.joint_state_lock = threading.Lock()
        self.latest_joint_positions = None
        self.latest_joint_state_time = None
        self.j2_fallback_used = False
        self.j3_fallback_used = False
        self.ik_seed_fallback_index = 0
        self.ik_seed_fallback_solutions = set()
        self.last_status_text = ''
        self.add_on_set_parameters_callback(
            self._on_tuning_parameters_changed
        )

        self.status_publisher = self.create_publisher(
            String, '/arm2/container_pick/status', 10
        )
        self.marker_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/marker_pose', 10
        )
        self.stack_target_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/stack_target_pose', 10
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
        self.stack_release_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/stack_release_pose', target_qos
        )
        self.stack_approach_pose_publisher = self.create_publisher(
            PoseStamped, '/arm2/container_pick/stack_approach_pose', target_qos
        )
        self.create_service(Trigger, '/arm2/pick_container', self.start_pick)
        self.create_service(Trigger, '/arm2/preview_stack', self.preview_stack)
        self.create_service(Trigger, '/arm2/stack_container', self.start_stack)
        self.create_service(
            Trigger, '/arm2/preview_pregrasp', self.preview_pregrasp
        )
        self.create_service(
            Trigger, '/arm2/move_to_pregrasp', self.start_pregrasp_test
        )
        self.create_service(Trigger, '/arm2/stop_pick', self.stop_pick)
        self.create_service(
            Trigger,
            '/arm2/scan_and_transfer',
            self.start_scan_and_transfer,
        )
        self.create_service(
            Trigger,
            '/arm2/reset_stack_level',
            self.reset_stack_level,
        )
        self.create_service(
            Trigger, '/arm2/scan_destinations', self.start_destination_scan
        )
        self.create_service(
            TransferById,
            '/arm2/transfer_by_id',
            self.start_id_to_id_transfer,
        )
        for destination_name, service_name in (
            ('A-1-1', '/arm2/transfer_to_a1_1'),
            ('A-1-2', '/arm2/transfer_to_a1_2'),
            ('A-2-1', '/arm2/transfer_to_a2_1'),
            ('A-2-2', '/arm2/transfer_to_a2_2'),
            ('A-3-1', '/arm2/transfer_to_a3_1'),
            ('A-3-2', '/arm2/transfer_to_a3_2'),
        ):
            self.create_service(
                Trigger,
                service_name,
                lambda request, response, name=destination_name:
                    self.start_saved_destination_transfer(
                        request, response, name
                    ),
            )
        for marker_id in range(9):
            self.create_service(
                Trigger,
                f'/arm2/load_id{marker_id}_to_trailer',
                lambda request, response, selected_id=marker_id:
                    self.start_loading_transfer(
                        request, response, selected_id
                    ),
            )
        self.create_timer(0.1, self.update_tracking)
        self.create_timer(1.0, self._republish_status)
        self.create_subscription(
            JointState,
            '/arm2/joint_states',
            self._joint_state_callback,
            10,
        )

        if self.execute_motion and self.motion_backend == 'direct':
            self._connect_robot()
            self.create_timer(0.1, self.publish_joint_states)
        elif self.execute_motion:
            self._initialize_moveit_clients()

        mode = 'EXECUTE' if self.execute_motion else 'DRY-RUN'
        self.publish_status(
            f'{mode}/{self.motion_backend}: tracking '
            f'{self.marker_frame} and {self.stack_target_frame} in '
            f'{self.base_frame}'
        )
        self.get_logger().info(
            'Loaded grasp tuning: '
            f'grasp_offset={self.grasp_offset.tolist()}, '
            f'pick_correction={self.pick_correction.tolist()}, '
            'id_transfer_pick_correction='
            f'{self.id_transfer_pick_correction.tolist()}, '
            f'place_correction={self.place_correction.tolist()}, '
            'saved_destination_correction='
            f'{self.saved_destination_correction.tolist()}, '
            f'id_transfer_correction={self.id_transfer_correction.tolist()}, '
            f'trailer_correction={self.trailer_correction.tolist()}, '
            f'grasp_rpy={self.grasp_rpy.tolist()}, '
            f'reference_yaw={self.reference_marker_yaw:.3f}deg, '
            f'yaw_symmetry={self.container_yaw_symmetry:.1f}deg'
        )

    def _on_tuning_parameters_changed(self, parameters):
        """Apply safe RQt tuning updates to the active target calculations."""
        tuning_names = {
            'allow_full_pick',
            'offsets_configured',
            'grasp_offset_xyz_m',
            'pick_correction_xyz_m',
            'id_transfer_pick_correction_xyz_m',
            'place_correction_xyz_m',
            'saved_destination_correction_xyz_m',
            'id_transfer_correction_xyz_m',
            'trailer_correction_xyz_m',
            'grasp_offset_rpy_deg',
            'reference_marker_yaw_deg',
            'container_yaw_symmetry_deg',
            'rotate_grasp_offset_with_marker_yaw',
        }
        requested = {
            parameter.name: parameter.value
            for parameter in parameters
            if parameter.name in tuning_names
        }
        if not requested:
            return SetParametersResult(successful=True)
        if self.motion_lock.locked():
            return SetParametersResult(
                successful=False,
                reason='grasp tuning cannot change while robot motion is active',
            )

        try:
            grasp_offset = self.grasp_offset
            if 'grasp_offset_xyz_m' in requested:
                grasp_offset = self._validated_tuning_vector(
                    'grasp_offset_xyz_m',
                    requested['grasp_offset_xyz_m'],
                    limit=0.5,
                )

            pick_correction = self.pick_correction
            if 'pick_correction_xyz_m' in requested:
                pick_correction = self._validated_tuning_vector(
                    'pick_correction_xyz_m',
                    requested['pick_correction_xyz_m'],
                    limit=0.2,
                )

            id_transfer_pick_correction = self.id_transfer_pick_correction
            if 'id_transfer_pick_correction_xyz_m' in requested:
                id_transfer_pick_correction = self._validated_tuning_vector(
                    'id_transfer_pick_correction_xyz_m',
                    requested['id_transfer_pick_correction_xyz_m'],
                    limit=0.2,
                )

            place_correction = self.place_correction
            if 'place_correction_xyz_m' in requested:
                place_correction = self._validated_tuning_vector(
                    'place_correction_xyz_m',
                    requested['place_correction_xyz_m'],
                    limit=0.2,
                )

            saved_destination_correction = self.saved_destination_correction
            if 'saved_destination_correction_xyz_m' in requested:
                saved_destination_correction = self._validated_tuning_vector(
                    'saved_destination_correction_xyz_m',
                    requested['saved_destination_correction_xyz_m'],
                    limit=0.2,
                )

            id_transfer_correction = self.id_transfer_correction
            if 'id_transfer_correction_xyz_m' in requested:
                id_transfer_correction = self._validated_tuning_vector(
                    'id_transfer_correction_xyz_m',
                    requested['id_transfer_correction_xyz_m'],
                    limit=0.2,
                )

            trailer_correction = self.trailer_correction
            if 'trailer_correction_xyz_m' in requested:
                trailer_correction = self._validated_tuning_vector(
                    'trailer_correction_xyz_m',
                    requested['trailer_correction_xyz_m'],
                    limit=0.2,
                )

            grasp_rpy = self.grasp_rpy
            if 'grasp_offset_rpy_deg' in requested:
                grasp_rpy = self._validated_tuning_vector(
                    'grasp_offset_rpy_deg',
                    requested['grasp_offset_rpy_deg'],
                    limit=360.0,
                )

            reference_yaw = float(requested.get(
                'reference_marker_yaw_deg',
                self.reference_marker_yaw,
            ))
            yaw_symmetry = float(requested.get(
                'container_yaw_symmetry_deg',
                self.container_yaw_symmetry,
            ))
            if not math.isfinite(reference_yaw):
                raise ValueError('reference_marker_yaw_deg must be finite')
            if not 0.0 < yaw_symmetry <= 360.0:
                raise ValueError(
                    'container_yaw_symmetry_deg must be within (0, 360]'
                )
        except (TypeError, ValueError) as exc:
            return SetParametersResult(successful=False, reason=str(exc))

        self.grasp_offset = grasp_offset
        self.pick_correction = pick_correction
        self.id_transfer_pick_correction = id_transfer_pick_correction
        self.place_correction = place_correction
        self.saved_destination_correction = saved_destination_correction
        self.id_transfer_correction = id_transfer_correction
        self.trailer_correction = trailer_correction
        self.grasp_rpy = grasp_rpy
        self.grasp_rotation = quaternion_from_rpy_degrees(*grasp_rpy)
        self.reference_marker_yaw = reference_yaw
        self.container_yaw_symmetry = yaw_symmetry
        if 'allow_full_pick' in requested:
            self.allow_full_pick = bool(requested['allow_full_pick'])
        if 'offsets_configured' in requested:
            self.offsets_configured = bool(requested['offsets_configured'])
        if 'rotate_grasp_offset_with_marker_yaw' in requested:
            self.rotate_grasp_offset_with_marker_yaw = bool(
                requested['rotate_grasp_offset_with_marker_yaw']
            )
        self.get_logger().info(
            'Applied grasp tuning: '
            f'allow_full_pick={self.allow_full_pick}, '
            f'offsets_configured={self.offsets_configured}, '
            f'grasp_offset={self.grasp_offset.tolist()}, '
            f'pick_correction={self.pick_correction.tolist()}, '
            'id_transfer_pick_correction='
            f'{self.id_transfer_pick_correction.tolist()}, '
            f'place_correction={self.place_correction.tolist()}, '
            'saved_destination_correction='
            f'{self.saved_destination_correction.tolist()}, '
            f'id_transfer_correction={self.id_transfer_correction.tolist()}, '
            f'trailer_correction={self.trailer_correction.tolist()}, '
            f'grasp_rpy={self.grasp_rpy.tolist()}'
        )
        return SetParametersResult(successful=True)

    @staticmethod
    def _validated_tuning_vector(name, values, limit):
        vector = np.asarray(values, dtype=np.float64)
        if vector.shape != (3,):
            raise ValueError(f'{name} must contain exactly 3 values')
        if not np.all(np.isfinite(vector)):
            raise ValueError(f'{name} must contain only finite values')
        if np.any(np.abs(vector) > limit):
            raise ValueError(
                f'{name} magnitude must not exceed {limit:g}'
            )
        return vector

    def _declare_parameters(self):
        self.declare_parameter('base_frame', 'arm2/base_link')
        self.declare_parameter('marker_frame', 'arm2/container_marker')
        self.declare_parameter(
            'stack_target_frame', 'arm2/stack_target_marker'
        )
        self.declare_parameter('execute_motion', False)
        self.declare_parameter('motion_backend', 'direct')
        self.declare_parameter('allow_full_pick', False)
        self.declare_parameter('offsets_configured', False)
        self.declare_parameter('use_marker_rotation_for_grasp', False)
        self.declare_parameter('grasp_orientation_mode', 'fixed')
        self.declare_parameter(
            'rotate_grasp_offset_with_marker_yaw', True
        )
        self.declare_parameter('grasp_offset_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('pick_correction_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'id_transfer_pick_correction_xyz_m', [0.0, 0.0, 0.0]
        )
        self.declare_parameter('place_correction_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter(
            'saved_destination_correction_xyz_m', [0.0, 0.0, 0.0]
        )
        self.declare_parameter(
            'id_transfer_correction_xyz_m', [0.0, 0.0, 0.0]
        )
        self.declare_parameter('trailer_correction_xyz_m', [0.0, 0.0, 0.0])
        self.declare_parameter('grasp_offset_rpy_deg', [0.0, 0.0, 0.0])
        self.declare_parameter('reference_marker_yaw_deg', 0.0)
        self.declare_parameter('container_yaw_symmetry_deg', 360.0)
        self.declare_parameter('prefer_current_gripper_yaw', True)
        self.declare_parameter('max_yaw_spread_deg', 8.0)
        self.declare_parameter('max_container_yaw_delta_deg', 90.0)
        self.declare_parameter(
            'grasp_yaw_fallback_scales', [0.75, 0.5, 0.25, 0.0]
        )
        self.declare_parameter('pregrasp_lift_m', 0.08)
        self.declare_parameter('grasp_extra_depth_m', 0.0)
        self.declare_parameter('grasp_stop_above_m', 0.0)
        self.declare_parameter('refresh_marker_before_descent', True)
        self.declare_parameter('visual_servo_enabled', False)
        self.declare_parameter('visual_servo_samples', 10)
        self.declare_parameter('visual_servo_xy_tolerance_m', 0.002)
        self.declare_parameter('visual_servo_yaw_tolerance_deg', 2.0)
        self.declare_parameter('visual_servo_xy_gain', 0.6)
        self.declare_parameter('visual_servo_yaw_gain', 0.6)
        self.declare_parameter('visual_servo_max_xy_step_m', 0.005)
        self.declare_parameter('visual_servo_max_yaw_step_deg', 2.0)
        self.declare_parameter('visual_servo_max_initial_error_m', 0.020)
        self.declare_parameter('visual_servo_max_iterations', 5)
        self.declare_parameter(
            'visual_servo_required_consecutive_successes', 3
        )
        self.declare_parameter('visual_servo_timeout_sec', 10.0)
        self.declare_parameter(
            'visual_servo_marker_loss_timeout_sec', 2.0
        )
        self.declare_parameter('visual_servo_settle_sec', 0.6)
        self.declare_parameter(
            'pregrasp_test_keep_current_orientation', True
        )
        self.declare_parameter('lift_after_pick_m', 0.14)
        self.declare_parameter('minimum_lift_after_pick_m', 0.14)
        self.declare_parameter('lift_search_step_m', 0.02)
        self.declare_parameter('bad_pick_branch_j2_deg', -62.31)
        self.declare_parameter('bad_pick_branch_j3_deg', 85.69)
        self.declare_parameter('bad_pick_branch_tolerance_deg', 12.0)
        self.declare_parameter('stack_container_height_m', 0.035)
        self.declare_parameter('stack_approach_clearance_m', 0.08)
        self.declare_parameter(
            'stack_minimum_approach_clearance_m', 0.03
        )
        self.declare_parameter('stack_approach_search_step_m', 0.01)
        self.declare_parameter(
            'stack_yaw_fallback_offsets_deg',
            [0.0, 15.0, -15.0, 30.0, -30.0, 180.0],
        )
        self.declare_parameter('stack_xy_offset_m', [0.0, 0.0])
        self.declare_parameter('stack_z_offset_m', 0.0)
        self.declare_parameter('max_stack_levels', 3)
        self.declare_parameter('stack_source_orientation_mode', 'marker_yaw')
        self.declare_parameter(
            'stack_segmented_descent_min_fraction', 0.65
        )
        self.declare_parameter('stack_segmented_descent_max_segments', 5)
        self.declare_parameter(
            'stack_pose_goal_finish_max_distance_m', 0.075
        )
        self.declare_parameter('minimum_stable_samples', 5)
        self.declare_parameter('max_translation_std_m', 0.005)
        self.declare_parameter('max_rotation_spread_deg', 5.0)
        self.declare_parameter('dry_run_max_rotation_spread_deg', 25.0)
        self.declare_parameter('max_marker_age_sec', 2.0)
        self.declare_parameter('workspace_min_xyz_m', [-0.30, -0.30, 0.005])
        self.declare_parameter('workspace_max_xyz_m', [0.30, 0.30, 0.32])
        self.declare_parameter('workspace_xy_shape', 'box')
        self.declare_parameter(
            'workspace_horizontal_min_xy_m', [-0.30, -0.30]
        )
        self.declare_parameter(
            'workspace_horizontal_max_xy_m', [0.30, 0.0]
        )
        self.declare_parameter(
            'workspace_vertical_min_xy_m', [0.0, -0.30]
        )
        self.declare_parameter(
            'workspace_vertical_max_xy_m', [0.30, 0.30]
        )
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 1000000)
        self.declare_parameter('speed', 10)
        self.declare_parameter('motion_timeout_sec', 15.0)
        self.declare_parameter('stabilization_timeout_sec', 12.0)
        self.declare_parameter('max_joint_delta_deg', 75.0)
        self.declare_parameter('gripper_open_value', 100)
        self.declare_parameter('gripper_closed_value', 20)
        self.declare_parameter('gripper_speed', 50)
        self.declare_parameter('moveit_group', 'arm_group')
        self.declare_parameter('moveit_ee_link', 'arm2/TCP')
        self.declare_parameter('move_group_action', '/arm2/move_action')
        self.declare_parameter(
            'execute_trajectory_action', '/arm2/execute_trajectory'
        )
        self.declare_parameter(
            'compute_cartesian_path_service',
            '/arm2/compute_cartesian_path',
        )
        self.declare_parameter('moveit_position_tolerance_m', 0.005)
        self.declare_parameter('moveit_orientation_tolerance_deg', 7.0)
        self.declare_parameter('moveit_planning_time_sec', 15.0)
        self.declare_parameter('moveit_planning_attempts', 30)
        self.declare_parameter('ik_timeout_sec', 0.3)
        self.declare_parameter('motion_ik_timeout_sec', 3.0)
        self.declare_parameter('moveit_velocity_scale', 0.35)
        self.declare_parameter('moveit_acceleration_scale', 0.25)
        self.declare_parameter('cartesian_max_step_m', 0.005)
        self.declare_parameter('cartesian_min_fraction', 0.95)
        self.declare_parameter('cartesian_absolute_min_fraction', 0.85)
        self.declare_parameter('cartesian_max_shortfall_m', 0.008)
        self.declare_parameter('cartesian_max_speed_mps', 0.04)
        self.declare_parameter('cartesian_max_joint_jump_deg', 20.0)
        self.declare_parameter('moveit_state_settle_sec', 0.4)
        self.declare_parameter('stack_preflight_joint_margin_deg', 0.9)
        self.declare_parameter('max_j6_trajectory_travel_deg', 170.0)
        self.declare_parameter('scan_marker_pause_sec', 0.5)
        self.declare_parameter('scan_timeout_sec', 90.0)
        self.declare_parameter('prefer_z_last_motion', False)
        self.declare_parameter(
            'destination_zone_j1_deg', [16.96, -28.74, -79.98]
        )
        self.declare_parameter('j2_fallback_min_tcp_height_m', 0.12)
        self.declare_parameter('j3_fallback_min_tcp_height_m', 0.12)
        self.declare_parameter('release_verify_xy_tolerance_m', 0.01)
        self.declare_parameter('release_verify_z_tolerance_m', 0.008)
        self.declare_parameter('release_verify_yaw_tolerance_deg', 3.0)

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
            self, MoveGroup, self.move_group_action
        )
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, self.execute_trajectory_action
        )
        self.follow_joint_trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/arm2/arm_group_controller/follow_joint_trajectory',
        )
        self.cartesian_path_client = self.create_client(
            GetCartesianPath, self.compute_cartesian_path_service
        )
        self.position_ik_client = self.create_client(
            GetPositionIK, '/arm2/compute_ik'
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
        self.return_home_client = self.create_client(
            Trigger, '/arm2/return_home'
        )
        self.go_a1_client = self.create_client(
            Trigger, '/arm2/go_a1_pose'
        )
        self.go_a2_client = self.create_client(
            Trigger, '/arm2/go_a2_pose'
        )
        self.go_a3_client = self.create_client(
            Trigger, '/arm2/go_a3_pose'
        )
        self.sweep_joint1_client = self.create_client(
            Trigger, '/arm2/scan_joint1'
        )
        self.pause_sweep_client = self.create_client(
            Trigger, '/arm2/pause_joint1_sweep'
        )
        self.resume_sweep_client = self.create_client(
            Trigger, '/arm2/resume_joint1_sweep'
        )
        self.scan_state_client = self.create_client(
            Trigger, '/arm2/joint1_scan_state'
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
        # The target is locked in base_frame before motion starts. The wrist
        # camera may lose the marker while rotating, descending, or lifting.
        if self.tracking_suspended.is_set() or (
            self.motion_lock.locked()
            and not self.refresh_marker_before_descent
            and not self.visual_servo_enabled
        ):
            return

        for marker_id, frame in self.source_frames.items():
            self._collect_marker_sample(
                frame,
                self.marker_histories[marker_id],
                self.marker_stamp_attributes[marker_id],
                self.marker_pose_publisher,
                f'container ID {marker_id}',
                warn_if_missing=False,
            )
        if self.destination_scan_active.is_set():
            for marker_id, frame in self.destination_frames.items():
                self._collect_marker_sample(
                    frame,
                    self.marker_histories[marker_id],
                    self.marker_stamp_attributes[marker_id],
                    self.stack_target_pose_publisher,
                    f'destination ID {marker_id}',
                    warn_if_missing=False,
                )
        for marker_id, frame in self.trailer_frames.items():
            self._collect_marker_sample(
                frame,
                self.marker_histories[marker_id],
                self.marker_stamp_attributes[marker_id],
                self.stack_target_pose_publisher,
                f'trailer ID {marker_id}',
                warn_if_missing=False,
            )

    def _collect_marker_sample(
        self,
        frame,
        history,
        stamp_attribute,
        publisher,
        label,
        warn_if_missing=True,
    ):
        with self.history_lock:
            if frame in self.scan_locked_frames:
                return
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, frame, Time()
            )
        except TransformException as exc:
            if not warn_if_missing:
                return
            error = str(exc)
            error_key = (
                'marker transform is older than the robot TF buffer'
                if 'extrapolation into the past' in error.lower()
                else error
            )
            previous_error, previous_time = self.tracking_errors.get(
                frame, ('', 0.0)
            )
            now = time.monotonic()
            if error_key != previous_error or now - previous_time >= 5.0:
                self.get_logger().warning(
                    f'Cannot collect {label} marker sample: '
                    f'{self.base_frame} -> {frame}: {error}'
                )
                self.tracking_errors[frame] = (error_key, now)
            return

        stamp = (
            transform.header.stamp.sec * 1_000_000_000
            + transform.header.stamp.nanosec
        )
        if stamp == getattr(self, stamp_attribute):
            return
        setattr(self, stamp_attribute, stamp)
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
            history.append((stamp, translation, rotation))
        publisher.publish(
            self.make_pose(translation, rotation, transform.header.stamp)
        )

    def stable_marker_pose(
        self,
        max_rotation_spread=None,
        yaw_only=False,
        history=None,
        position_only=False,
        sample_count=None,
    ):
        selected_history = self.history if history is None else history
        required_samples = (
            self.minimum_samples
            if sample_count is None else int(sample_count)
        )
        with self.history_lock:
            samples = list(selected_history)[-required_samples:]
        if len(samples) < required_samples:
            return None, f'need {required_samples - len(samples)} more samples'

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

        if position_only:
            return (translation, samples[-1][2]), 'stable'

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
        self,
        validate_workspace=True,
        max_rotation_spread=None,
        orientation_mode=None,
    ):
        mode = orientation_mode or self.grasp_orientation_mode
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
        return self.calculate_targets_from_marker_pose(
            marker_pose,
            validate_workspace=validate_workspace,
            orientation_mode=mode,
        )

    def calculate_targets_from_marker_pose(
        self,
        marker_pose,
        validate_workspace=True,
        orientation_mode=None,
        pick_correction=None,
    ):
        """Build pick targets from a marker pose locked during scanning."""
        if pick_correction is None:
            pick_correction = self.pick_correction
        mode = orientation_mode or self.grasp_orientation_mode
        marker_translation, marker_rotation = marker_pose
        correction_yaw = None
        if mode == 'marker_full':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            correction_yaw = symmetric_marker_yaw_degrees(
                marker_yaw,
                self.reference_marker_yaw,
                self.container_yaw_symmetry,
            )
            grasp_translation, grasp_rotation = compose_pose(
                marker_translation,
                marker_rotation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        elif mode == 'marker_yaw':
            marker_yaw = quaternion_to_rpy_degrees(marker_rotation)[2]
            correction_yaw = symmetric_marker_yaw_degrees(
                marker_yaw,
                self.reference_marker_yaw,
                self.container_yaw_symmetry,
            )
            grasp_translation, grasp_rotation, yaw_delta = (
                compose_yaw_follow_pose(
                    marker_translation,
                    self.grasp_offset,
                    self.grasp_rpy,
                    marker_yaw,
                    self.reference_marker_yaw,
                    self.rotate_grasp_offset_with_marker_yaw,
                    self.container_yaw_symmetry,
                )
            )
            if abs(yaw_delta) > self.max_yaw_delta:
                return None, (
                    f'container yaw delta {yaw_delta:.2f}deg exceeds limit'
                )
            current_yaw = self._current_tcp_yaw_degrees()
            if (
                self.prefer_current_gripper_yaw
                and current_yaw is not None
                and self.container_yaw_symmetry < 360.0
            ):
                target_rpy = quaternion_to_rpy_degrees(grasp_rotation)
                nearest_yaw = nearest_symmetric_yaw_degrees(
                    target_rpy[2],
                    current_yaw,
                    self.container_yaw_symmetry,
                )
                grasp_rotation = quaternion_from_rpy_degrees(
                    target_rpy[0], target_rpy[1], nearest_yaw
                )
        else:
            grasp_translation, grasp_rotation = compose_fixed_base_pose(
                marker_translation,
                self.grasp_offset,
                self.grasp_rotation,
            )
        if correction_yaw is None:
            grasp_translation = apply_base_frame_correction(
                grasp_translation,
                pick_correction,
            )
        else:
            grasp_translation = apply_marker_yaw_correction(
                grasp_translation,
                pick_correction,
                correction_yaw,
            )
        grasp_translation, pregrasp_translation = apply_vertical_pick_offsets(
            grasp_translation,
            self.pregrasp_lift,
            self.grasp_extra_depth,
            self.grasp_stop_above,
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

    def _current_tcp_yaw_degrees(self):
        """Return current TCP yaw in the fixed robot base frame."""
        pose = self._current_tcp_pose()
        if pose is None:
            return None
        return quaternion_to_rpy_degrees(pose[1])[2]

    def _current_tcp_pose(self):
        """Return current TCP translation and rotation in the base frame."""
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException:
            return None
        translation = np.array([
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        ], dtype=np.float64)
        rotation = normalize_quaternion([
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        ])
        return translation, rotation

    def _wait_for_visual_servo_targets(
        self, marker_frame, marker_history, deadline
    ):
        """Collect a fresh stable marker batch for one refinement cycle."""
        with self.history_lock:
            marker_history.clear()
            self.scan_locked_frames.discard(marker_frame)
        marker_deadline = min(
            deadline,
            time.monotonic() + self.visual_servo_marker_loss_timeout,
        )
        reason = 'no new marker samples'
        while time.monotonic() < marker_deadline:
            if self.stop_event.wait(0.05):
                return None, 'pick stopped'
            marker_pose, reason = self.stable_marker_pose(
                history=marker_history,
                yaw_only=True,
                sample_count=self.visual_servo_samples,
            )
            if marker_pose is None:
                continue
            return self.calculate_targets_from_marker_pose(
                marker_pose,
                orientation_mode=self.stack_source_orientation_mode,
            )
        return None, f'marker lost or unstable: {reason}'

    def refine_pregrasp_with_visual_feedback(
        self,
        initial_targets,
        marker_frame,
        marker_history,
        marker_id,
        correct_yaw=True,
    ):
        """Correct pregrasp XY/yaw iteratively from fresh camera feedback."""
        latest_targets = copy.deepcopy(initial_targets)
        deadline = time.monotonic() + self.visual_servo_timeout
        consecutive_successes = 0
        previous_metric = None
        increasing_count = 0

        for iteration in range(1, self.visual_servo_max_iterations + 1):
            if time.monotonic() >= deadline:
                raise RuntimeError('visual servo timed out')
            refreshed, reason = self._wait_for_visual_servo_targets(
                marker_frame, marker_history, deadline
            )
            if refreshed is None:
                raise RuntimeError(f'visual servo failed: {reason}')
            latest_targets = refreshed
            _grasp, desired_pregrasp = refreshed
            current = self._current_tcp_pose()
            if current is None:
                raise RuntimeError('visual servo cannot read current TCP TF')
            current_translation, current_rotation = current
            desired_translation = np.array([
                desired_pregrasp.pose.position.x,
                desired_pregrasp.pose.position.y,
                desired_pregrasp.pose.position.z,
            ])
            error_xy = desired_translation[:2] - current_translation[:2]
            desired_yaw = quaternion_to_rpy_degrees([
                desired_pregrasp.pose.orientation.x,
                desired_pregrasp.pose.orientation.y,
                desired_pregrasp.pose.orientation.z,
                desired_pregrasp.pose.orientation.w,
            ])[2]
            current_rpy = quaternion_to_rpy_degrees(current_rotation)
            yaw_error = wrap_degrees(desired_yaw - current_rpy[2])
            xy_error_norm = float(np.linalg.norm(error_xy))
            if xy_error_norm > self.visual_servo_max_initial_error:
                raise RuntimeError(
                    'visual servo XY error exceeds safety limit: '
                    f'{xy_error_norm * 1000.0:.1f}mm'
                )
            controlled_yaw_error = yaw_error if correct_yaw else 0.0
            metric = max(
                xy_error_norm / self.visual_servo_xy_tolerance,
                abs(controlled_yaw_error)
                / self.visual_servo_yaw_tolerance,
            )
            self.publish_status(
                f'VISUAL SERVO: ID {marker_id} iteration '
                f'{iteration}/{self.visual_servo_max_iterations}: '
                f'error_xy_mm='
                f'{np.round(error_xy * 1000.0, 2).tolist()}, '
                f'yaw_error_deg={yaw_error:.2f}'
                + (
                    ''
                    if correct_yaw
                    else ' (yaw locked for final alignment)'
                )
            )

            if visual_servo_within_tolerance(
                error_xy,
                controlled_yaw_error,
                self.visual_servo_xy_tolerance,
                self.visual_servo_yaw_tolerance,
            ):
                consecutive_successes += 1
                if (
                    consecutive_successes
                    >= self.visual_servo_required_successes
                ):
                    self.publish_status(
                        'VISUAL SERVO: converged; locking final marker pose'
                    )
                    return latest_targets
                continue

            consecutive_successes = 0
            if previous_metric is not None and metric > previous_metric:
                increasing_count += 1
            else:
                increasing_count = 0
            if increasing_count >= 2:
                raise RuntimeError(
                    'visual servo error increased twice consecutively'
                )
            previous_metric = metric

            xy_step, yaw_step = bounded_visual_servo_step(
                error_xy,
                controlled_yaw_error,
                self.visual_servo_xy_gain,
                self.visual_servo_yaw_gain,
                self.visual_servo_max_xy_step,
                self.visual_servo_max_yaw_step,
            )
            correction_translation = current_translation.copy()
            correction_translation[:2] += xy_step
            correction_rotation = quaternion_from_rpy_degrees(
                current_rpy[0],
                current_rpy[1],
                current_rpy[2] + (yaw_step if correct_yaw else 0.0),
            )
            correction = self.make_pose(
                correction_translation,
                correction_rotation,
                self.get_clock().now().to_msg(),
            )
            self.publish_status(
                'VISUAL SERVO: Cartesian correction '
                f'xy_mm={np.round(xy_step * 1000.0, 2).tolist()}, '
                f'yaw_deg={yaw_step:.2f}'
            )
            self.move_cartesian_to_pose(correction)
            if self.stop_event.wait(self.visual_servo_settle):
                raise RuntimeError('pick stopped')

        raise RuntimeError(
            'visual servo did not converge within '
            f'{self.visual_servo_max_iterations} iterations'
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

    def calculate_stack_targets(self, validate_workspace=True):
        """Build source pick and destination release targets from two markers."""
        source_targets, reason = self.calculate_targets(
            validate_workspace=validate_workspace,
            orientation_mode=self.stack_source_orientation_mode,
        )
        if source_targets is None:
            return None, f'source marker: {reason}'

        destination_pose, reason = self.stable_marker_pose(
            history=self.stack_target_history,
            yaw_only=True,
        )
        if destination_pose is None:
            return None, f'stack target marker: {reason}'
        return self.calculate_stack_targets_from_locked_poses(
            source_targets,
            destination_pose,
            validate_workspace=validate_workspace,
        )

    def calculate_stack_targets_from_locked_poses(
        self,
        source_targets,
        destination_pose,
        validate_workspace=True,
        placed_count=None,
        destination_correction=None,
    ):
        """Build release targets from scan-locked source/destination poses."""
        if destination_correction is None:
            destination_correction = self.place_correction
        if placed_count is None:
            with self.stack_level_lock:
                placed_count = self.placed_stack_count
        layer_z_offset = stack_layer_z_offset(
            self.stack_z_offset,
            self.stack_container_height,
            placed_count,
        )
        destination_yaw = quaternion_to_rpy_degrees(destination_pose[1])[2]
        (
            release_translation,
            approach_translation,
            release_rotation,
            destination_yaw_delta,
        ) = calculate_heading_aligned_stack_poses(
            destination_pose[0],
            self.grasp_offset,
            self.grasp_rpy,
            destination_yaw,
            self.reference_marker_yaw,
            self.stack_container_height,
            self.stack_approach_clearance,
            self.grasp_extra_depth,
            self.stack_xy_offset,
            self.container_yaw_symmetry,
            layer_z_offset,
        )
        release_translation = apply_marker_yaw_correction(
            release_translation,
            destination_correction,
            self.reference_marker_yaw + destination_yaw_delta,
        )
        approach_translation = apply_marker_yaw_correction(
            approach_translation,
            destination_correction,
            self.reference_marker_yaw + destination_yaw_delta,
        )
        if abs(destination_yaw_delta) > self.max_yaw_delta:
            return None, (
                'stack target yaw delta '
                f'{destination_yaw_delta:.2f}deg exceeds limit'
            )
        if validate_workspace:
            if not self.in_workspace(release_translation):
                return None, (
                    f'stack release target outside workspace: '
                    f'{release_translation}'
                )
            if not self.in_workspace(approach_translation):
                return None, (
                    f'stack approach target outside workspace: '
                    f'{approach_translation}'
                )

        stamp = self.get_clock().now().to_msg()
        release = self.make_pose(
            release_translation, release_rotation, stamp
        )
        approach = self.make_pose(
            approach_translation, release_rotation, stamp
        )
        self.stack_release_pose_publisher.publish(release)
        self.stack_approach_pose_publisher.publish(approach)
        return (
            source_targets,
            release,
            approach,
        ), f'stack layer {placed_count + 1} targets valid'

    def wait_for_new_stable_stack_targets(self):
        """Wait until source and destination markers are fresh and stable."""
        with self.history_lock:
            self.history.clear()
            self.stack_target_history.clear()
            self.scan_locked_frames.clear()
        deadline = time.monotonic() + self.stabilization_timeout
        reason = 'no new marker samples'
        last_report_time = 0.0
        last_report_reason = ''
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.2):
                return None, 'stack stopped'
            targets, reason = self.calculate_stack_targets()
            if targets is not None:
                return targets, reason
            now = time.monotonic()
            if reason != last_report_reason or now - last_report_time >= 1.0:
                self.publish_status(f'STACK: waiting for markers: {reason}')
                last_report_reason = reason
                last_report_time = now
        return None, f'markers did not stabilize: {reason}'

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
        self.publish_status('PICK: waiting for fresh stable marker')
        targets, reason = self.wait_for_new_stable_targets()
        if targets is None:
            self.publish_status(f'PICK FAILED: {reason}')
            return
        self.publish_status('PICK: fresh marker target locked')
        self.execute_pick(targets)

    def preview_stack(self, _request, response):
        targets, reason = self.calculate_stack_targets(
            validate_workspace=False
        )
        if targets is None:
            response.success = False
            response.message = reason
            return response
        source_targets, release, approach = targets
        source_grasp_yaw = quaternion_to_rpy_degrees([
            source_targets[0].pose.orientation.x,
            source_targets[0].pose.orientation.y,
            source_targets[0].pose.orientation.z,
            source_targets[0].pose.orientation.w,
        ])[2]
        release_xyz = np.round([
            release.pose.position.x,
            release.pose.position.y,
            release.pose.position.z,
        ], 4).tolist()
        approach_xyz = np.round([
            approach.pose.position.x,
            approach.pose.position.y,
            approach.pose.position.z,
        ], 4).tolist()
        response.success = True
        response.message = (
            'PREVIEW ONLY: no motion; '
            f'release_m={release_xyz}, approach_m={approach_xyz}, '
            f'source_grasp_yaw_deg={source_grasp_yaw:.2f}, '
            f'container_height_m={self.stack_container_height:.4f}'
        )
        return response

    def start_stack(self, _request, response):
        if not self.execute_motion:
            response.success = False
            response.message = 'Stack requires execute_motion:=true'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full pick and calibrated offsets are required'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_stack_after_stabilization,
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            'Stack accepted; waiting for fresh ID 0 and ID 1 markers'
        )
        return response

    def execute_stack_after_stabilization(self):
        """Lock both marker poses before starting any physical movement."""
        self.publish_status('STACK: waiting for fresh stable markers')
        targets, reason = self.wait_for_new_stable_stack_targets()
        if targets is None:
            self.publish_status(f'STACK FAILED: {reason}')
            return
        self.publish_status('STACK: source and destination targets locked')
        self.execute_stack(targets)

    def start_scan_and_transfer(self, _request, response):
        """Start legacy scanning for primary container ID 0 and A-1 ID 11."""
        if not self.execute_motion or self.motion_backend != 'moveit':
            response.success = False
            response.message = 'Scan transfer requires MoveIt execution'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full pick and calibrated offsets are required'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        with self.stack_level_lock:
            if self.placed_stack_count >= self.max_stack_levels:
                response.success = False
                response.message = (
                    f'Maximum stack level {self.max_stack_levels} reached; '
                    'call /arm2/reset_stack_level'
                )
                return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_scan_and_transfer,
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = 'Scanning for container ID 0 and A-1 ID 11 started'
        return response

    def start_destination_scan(self, _request, response):
        """Scan and persist six destination poses and an optional trailer."""
        if not self.execute_motion or self.motion_backend != 'moveit':
            response.success = False
            response.message = 'Destination scan requires MoveIt execution'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        self.stop_event.clear()
        self.destination_scan_active.set()
        self.motion_thread = threading.Thread(
            target=self.execute_destination_scan,
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            'Scanning destinations along HOME-A1-A2-A3, then returning home'
        )
        return response

    def execute_destination_scan(self):
        try:
            destination_marker_ids = tuple(range(11, 17))
            trailer_marker_ids = tuple(self.trailer_frames)
            scan_marker_ids = destination_marker_ids + trailer_marker_ids
            specs = tuple(
                (
                    (
                        f'ID {marker_id}'
                        if marker_id not in self.trailer_frames
                        else f'trailer ID {marker_id} (optional)'
                    ),
                    (
                        self.destination_frames[marker_id]
                        if marker_id not in self.trailer_frames
                        else self.trailer_frames[marker_id]
                    ),
                    self.marker_histories[marker_id],
                )
                for marker_id in scan_marker_ids
            )
            self.saved_destination_poses.clear()
            self.saved_marker_poses.clear()
            locked, reason = self._scan_destination_route(
                specs,
                required_count=len(destination_marker_ids),
            )
            if locked is None:
                self.publish_status(f'DESTINATION SCAN FAILED: {reason}')
                return
            # All required destinations are now locked in base_frame.  Freeze
            # marker collection for the entire home return so IDs 11-16 seen
            # again by the wrist camera cannot affect the first accepted
            # poses.
            self.tracking_suspended.set()
            for marker_id, pose in zip(scan_marker_ids, locked):
                if pose is not None:
                    self.saved_marker_poses[marker_id] = pose
            self.saved_destination_poses = {
                name: self.saved_marker_poses[marker_id]
                for name, marker_id in self.destination_ids.items()
            }
            with self.stack_level_lock:
                for name in self.saved_destination_stack_counts:
                    self.saved_destination_stack_counts[name] = 0
            self.publish_status(
                'DESTINATION SCAN: IDs 11-16 saved; route ended at home'
            )
        except Exception as exc:
            self.publish_status(
                f'DESTINATION SCAN FAILED during pose route: {exc}'
            )
            self._recover_home_after_failure('DESTINATION SCAN')
            return
        finally:
            self.tracking_suspended.clear()
            self.destination_scan_active.clear()
        self.publish_status(
            'DESTINATION SCAN COMPLETED: A-1-1=11, A-1-2=12, '
            'A-2-1=13, A-2-2=14, A-3-1=15, A-3-2=16; '
            'trailer IDs '
            + ', '.join(
                f'{marker_id}='
                f'{"saved" if marker_id in self.saved_marker_poses else "not seen"}'
                for marker_id in self.trailer_frames
            )
        )

    def _scan_destination_route(self, specs, required_count):
        """Visit home/A poses and lock stable destination marker poses."""
        route = (
            ('HOME', self.return_home_client, 'always'),
            ('A-1', self.go_a1_client, 'always'),
            ('A-2', self.go_a2_client, 'always'),
            ('A-3', self.go_a3_client, 'always'),
            ('A-2', self.go_a2_client, 'if_missing'),
            ('A-1', self.go_a1_client, 'if_missing'),
            ('HOME', self.return_home_client, 'if_missing'),
        )
        if any(
            client is None or not client.wait_for_service(timeout_sec=3.0)
            for _name, client, _scan_mode in route
        ):
            return None, 'destination route service is unavailable'

        locked = [None] * len(specs)
        with self.history_lock:
            self.scan_locked_frames.clear()
            for _label, _frame, history in specs:
                history.clear()

        for route_index, (pose_name, client, scan_mode) in enumerate(
            route, start=1
        ):
            if self.stop_event.is_set():
                return None, 'destination route stopped'
            required_missing = any(
                locked[index] is None for index in range(required_count)
            )
            scan_here = (
                scan_mode == 'always'
                or (scan_mode == 'if_missing' and required_missing)
            )
            if not scan_here:
                self.tracking_suspended.set()
            else:
                self.tracking_suspended.clear()
            self.publish_status(
                f'DESTINATION SCAN: route {route_index}/{len(route)} '
                f'moving to {pose_name}'
            )
            self._call_scan_service(
                client,
                f'destination route {pose_name}',
                timeout=self.motion_timeout + 5.0,
            )
            if not scan_here:
                self.publish_status(
                    f'DESTINATION SCAN: route {route_index}/{len(route)} '
                    f'at {pose_name}; all required IDs saved, '
                    'return scan skipped'
                )
                continue
            with self.history_lock:
                for index, (_label, _frame, history) in enumerate(specs):
                    if locked[index] is None:
                        history.clear()

            zone_started = time.monotonic()
            deadline = zone_started + self.scan_marker_pause + 2.0
            hard_deadline = (
                zone_started + 2.0 * self.scan_marker_pause + 4.0
            )
            stabilization_hold_announced = False
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.05):
                    return None, 'destination route stopped'
                visible_but_unlocked = any(
                    locked[index] is None
                    and self._history_sample_count(specs[index][2]) > 0
                    for index in range(required_count)
                )
                if visible_but_unlocked:
                    if not stabilization_hold_announced:
                        self.publish_status(
                            'DESTINATION SCAN: required marker visible at '
                            f'{pose_name}; holding until its pose is stable'
                        )
                        stabilization_hold_announced = True
                    deadline = min(
                        hard_deadline,
                        max(
                            deadline,
                            time.monotonic() + self.scan_marker_pause + 0.5,
                        ),
                    )
                for index, (label, frame, history) in enumerate(specs):
                    if locked[index] is not None:
                        continue
                    pose, _reason = self.stable_marker_pose(
                        history=history,
                        yaw_only=True,
                    )
                    if pose is None:
                        continue
                    locked[index] = (
                        np.array(pose[0], dtype=np.float64),
                        np.array(pose[1], dtype=np.float64),
                    )
                    with self.history_lock:
                        self.scan_locked_frames.add(frame)
                    self.publish_status(
                        f'DESTINATION SCAN: {label} saved at {pose_name}'
                    )
                if all(pose is not None for pose in locked):
                    break

        missing = [
            specs[index][0]
            for index in range(required_count)
            if locked[index] is None
        ]
        if missing:
            return None, (
                'route ended before stabilizing ' + ', '.join(missing)
            )
        return tuple(locked), 'destination route completed'

    def start_loading_transfer(self, _request, response, source_marker_id):
        """Scan one selected container and trailer ID 9 or 10, then load it."""
        if not self.execute_motion or self.motion_backend != 'moveit':
            response.success = False
            response.message = 'Trailer loading requires MoveIt execution'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full pick and calibrated offsets are required'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_loading_transfer,
            args=(source_marker_id,),
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            f'Scanning container ID {source_marker_id} and trailer ID 9/10 '
            'along HOME-A1-A2-A3, then loading the selected container'
        )
        return response

    def start_id_to_id_transfer(self, request, response):
        """Accept a source/destination ID pair supplied by another system."""
        source_id = int(request.source_id)
        destination_id = int(request.destination_id)
        valid_source_ids = set(self.source_frames)
        valid_destination_ids = (
            valid_source_ids | set(self.destination_frames)
        )
        if (
            source_id not in valid_source_ids
            or destination_id not in valid_destination_ids
        ):
            response.accepted = False
            response.message = (
                'source_id must be within 0..8; destination_id must be '
                'within 0..8 or 11..16'
            )
            return response
        if source_id == destination_id:
            response.accepted = False
            response.message = 'source_id and destination_id must be different'
            return response
        if not self.execute_motion or self.motion_backend != 'moveit':
            response.accepted = False
            response.message = 'ID transfer requires MoveIt execution'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.accepted = False
            response.message = 'Full pick and calibrated offsets are required'
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.accepted = False
            response.message = 'A robot motion is already running'
            return response
        destination_zone = next(
            (
                name for name, marker_id in self.destination_ids.items()
                if marker_id == destination_id
            ),
            None,
        )
        stack_name = (
            destination_zone
            if destination_zone is not None else f'ID-{destination_id}'
        )
        with self.stack_level_lock:
            placed_count = self.saved_destination_stack_counts[stack_name]
        if placed_count >= self.max_stack_levels:
            response.accepted = False
            response.message = (
                f'Destination ID {destination_id} already has the maximum '
                f'{self.max_stack_levels} layers'
            )
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_id_to_id_transfer,
            args=(source_id, destination_id),
            daemon=True,
        )
        self.motion_thread.start()
        response.accepted = True
        response.message = (
            f'Scanning only requested source ID {source_id} and '
            f'destination ID {destination_id}, then starting transfer'
            + (
                f' ({destination_zone})'
                if destination_zone is not None else ''
            )
        )
        return response

    def execute_id_to_id_transfer(self, source_id, destination_id):
        """Scan a source and transfer to a container or saved stack marker."""
        destination_zone = next(
            (
                name for name, marker_id in self.destination_ids.items()
                if marker_id == destination_id
            ),
            None,
        )
        specs = [(
            f'source container ID {source_id}',
            self.source_frames[source_id],
            self.marker_histories[source_id],
        )]
        if destination_zone is not None:
            specs.append((
                f'destination stack ID {destination_id} '
                f'({destination_zone})',
                self.destination_frames[destination_id],
                self.marker_histories[destination_id],
            ))
        else:
            specs.append((
                f'destination container ID {destination_id}',
                self.source_frames[destination_id],
                self.marker_histories[destination_id],
            ))
        if destination_zone is not None:
            self.destination_scan_active.set()
        try:
            scan_result, reason = self._scan_id_transfer_route(specs)
        finally:
            if destination_zone is not None:
                self.destination_scan_active.clear()
        if scan_result is None:
            self.publish_status(
                f'ID {source_id} -> ID {destination_id} FAILED: {reason}'
            )
            return
        locked, scan_zone = scan_result
        self.tracking_suspended.set()
        source_pose = copy.deepcopy(locked[0])
        destination_pose = copy.deepcopy(locked[1])
        if destination_zone is None:
            stack_name = f'ID-{destination_id}'
            destination_correction = self.id_transfer_correction
        else:
            self.saved_destination_poses[destination_zone] = copy.deepcopy(
                destination_pose
            )
            stack_name = destination_zone
            destination_correction = self.saved_destination_correction
        try:
            source_targets, reason = self.calculate_targets_from_marker_pose(
                source_pose,
                orientation_mode=self.stack_source_orientation_mode,
                pick_correction=self.id_transfer_pick_correction,
            )
            if source_targets is None:
                raise RuntimeError(f'source target: {reason}')
            with self.stack_level_lock:
                placed_count = self.saved_destination_stack_counts[stack_name]
            targets, reason = self.calculate_stack_targets_from_locked_poses(
                source_targets,
                destination_pose,
                placed_count=placed_count,
                destination_correction=destination_correction,
            )
            if targets is None:
                raise RuntimeError(f'destination target: {reason}')
            self.publish_status(
                f'ID TRANSFER: source ID {source_id} and destination '
                f'ID {destination_id}'
                + (
                    f' ({destination_zone}) saved and locked'
                    if destination_zone is not None else ' saved and locked'
                )
                + '; starting transfer'
            )
            self.execute_scanned_transfer(
                targets,
                destination_name=(
                    destination_zone
                    if destination_zone is not None else f'ID {destination_id}'
                ),
                count_stack=False,
                saved_stack_name=stack_name,
                source_marker_id=source_id,
                align_source_before_pick=True,
                source_scan_zone=scan_zone,
            )
        except Exception as exc:
            self.publish_status(
                f'ID {source_id} -> ID {destination_id} FAILED: {exc}'
            )
            self._recover_home_after_failure(
                f'ID {source_id} -> ID {destination_id}'
            )
        finally:
            self.tracking_suspended.clear()

    def _scan_id_transfer_route(self, specs):
        """Lock both ID-transfer markers along the fixed A-zone route."""
        route = (
            ('HOME', self.return_home_client),
            ('A-1', self.go_a1_client),
            ('A-2', self.go_a2_client),
            ('A-3', self.go_a3_client),
            ('A-2', self.go_a2_client),
            ('A-1', self.go_a1_client),
        )
        if any(
            client is None or not client.wait_for_service(timeout_sec=3.0)
            for _name, client in route
        ):
            return None, 'ID transfer route service is unavailable'

        locked = [None] * len(specs)
        with self.history_lock:
            for _label, frame, history in specs:
                history.clear()
                self.scan_locked_frames.discard(frame)

        for route_index, (pose_name, client) in enumerate(route, start=1):
            if self.stop_event.is_set():
                return None, 'ID transfer route stopped'
            self.tracking_suspended.clear()
            self.publish_status(
                f'ID TRANSFER SCAN: route {route_index}/{len(route)} '
                f'moving to {pose_name}'
            )
            self._call_scan_service(
                client,
                f'ID transfer route {pose_name}',
                timeout=self.motion_timeout + 5.0,
            )
            with self.history_lock:
                for index, (_label, _frame, history) in enumerate(specs):
                    if locked[index] is None:
                        history.clear()

            zone_started = time.monotonic()
            deadline = zone_started + self.scan_marker_pause + 2.0
            hard_deadline = (
                zone_started + 2.0 * self.scan_marker_pause + 3.0
            )
            grouped_hold_announced = False
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.05):
                    return None, 'ID transfer route stopped'
                all_requested_visible = all(
                    locked[index] is not None
                    or self._history_sample_count(history) > 0
                    for index, (_label, _frame, history)
                    in enumerate(specs)
                )
                if all_requested_visible:
                    if not grouped_hold_announced:
                        self.publish_status(
                            'ID TRANSFER SCAN: all requested IDs visible at '
                            f'{pose_name}; holding this zone until saved'
                        )
                        grouped_hold_announced = True
                    deadline = min(
                        hard_deadline,
                        max(
                            deadline,
                            time.monotonic() + self.scan_marker_pause + 0.5,
                        ),
                    )
                for index, (label, frame, history) in enumerate(specs):
                    if locked[index] is not None:
                        continue
                    pose, _reason = self.stable_marker_pose(
                        history=history,
                        yaw_only=True,
                    )
                    if pose is None:
                        continue
                    locked[index] = (
                        np.array(pose[0], dtype=np.float64),
                        np.array(pose[1], dtype=np.float64),
                    )
                    with self.history_lock:
                        self.scan_locked_frames.add(frame)
                    self.publish_status(
                        f'ID TRANSFER SCAN: {label} saved at {pose_name}'
                    )
                if all(pose is not None for pose in locked):
                    self.publish_status(
                        'ID TRANSFER SCAN: all requested IDs saved; '
                        'remaining route skipped'
                    )
                    return (
                        (tuple(locked), pose_name),
                        'ID transfer markers locked',
                    )

        missing = [
            label for (label, _frame, _history), pose
            in zip(specs, locked) if pose is None
        ]
        return None, (
            'HOME-A1-A2-A3-A2-A1 route ended before detecting '
            + ', '.join(missing)
        )

    def execute_loading_transfer(self, source_marker_id):
        """Scan a selected source and trailer, then execute the transfer."""
        specs = [(
            f'container ID {source_marker_id}',
            self.source_frames[source_marker_id],
            self.marker_histories[source_marker_id],
        )]
        specs.extend(
            (
                f'trailer ID {marker_id}',
                frame,
                self.marker_histories[marker_id],
            )
            for marker_id, frame in self.trailer_frames.items()
        )
        locked, reason = self._scan_trailer_route(tuple(specs))
        if locked is None:
            self.publish_status(
                f'TRAILER LOAD ID {source_marker_id} FAILED: {reason}'
            )
            return
        # Freeze both accepted base-frame poses for the complete operation.
        # Re-observing either marker from the moving wrist camera must not
        # change the pick or trailer target after this point.
        self.tracking_suspended.set()
        source_pose = copy.deepcopy(locked[0])
        selected_offset = next(
            index for index in (1, 2) if locked[index] is not None
        )
        trailer_pose = copy.deepcopy(locked[selected_offset])
        trailer_marker_id = tuple(self.trailer_frames)[selected_offset - 1]
        try:
            source_targets, reason = self.calculate_targets_from_marker_pose(
                source_pose,
                orientation_mode=self.stack_source_orientation_mode,
            )
            if source_targets is None:
                self.publish_status(
                    f'TRAILER LOAD ID {source_marker_id} FAILED: {reason}'
                )
                self._recover_home_after_failure(
                    f'TRAILER LOAD ID {source_marker_id}'
                )
                return
            with self.stack_level_lock:
                placed_count = self.saved_destination_stack_counts['TRAILER']
            if placed_count >= self.max_stack_levels:
                self.publish_status(
                    f'TRAILER LOAD ID {source_marker_id} FAILED: '
                    f'maximum {self.max_stack_levels} layers reached'
                )
                self._recover_home_after_failure(
                    f'TRAILER LOAD ID {source_marker_id}'
                )
                return
            targets, reason = self.calculate_stack_targets_from_locked_poses(
                source_targets,
                trailer_pose,
                placed_count=placed_count,
                destination_correction=self.trailer_correction,
            )
            if targets is None:
                self.publish_status(
                    f'TRAILER LOAD ID {source_marker_id} FAILED: {reason}'
                )
                self._recover_home_after_failure(
                    f'TRAILER LOAD ID {source_marker_id}'
                )
                return
            self.publish_status(
                f'TRAILER LOAD: ID {source_marker_id} and trailer ID '
                f'{trailer_marker_id} locked; '
                'tracking frozen until home return; '
                'destination IDs 11-16 preserved'
            )
            self.execute_scanned_transfer(
                targets,
                destination_name='TRAILER',
                count_stack=False,
                saved_stack_name='TRAILER',
                source_marker_id=source_marker_id,
            )
        finally:
            self.tracking_suspended.clear()

    def _scan_trailer_route(self, specs):
        """Visit HOME/A1/A2/A3 and lock one source plus either trailer."""
        route = (
            ('HOME', self.return_home_client),
            ('A-1', self.go_a1_client),
            ('A-2', self.go_a2_client),
            ('A-3', self.go_a3_client),
        )
        if any(
            client is None or not client.wait_for_service(timeout_sec=3.0)
            for _name, client in route
        ):
            return None, 'trailer scan route service is unavailable'

        locked = [None] * len(specs)
        with self.history_lock:
            for _label, frame, history in specs:
                history.clear()
                self.scan_locked_frames.discard(frame)

        for route_index, (pose_name, client) in enumerate(route, start=1):
            if self.stop_event.is_set():
                return None, 'trailer scan route stopped'
            self.publish_status(
                f'TRAILER SCAN: route {route_index}/{len(route)} '
                f'moving to {pose_name}'
            )
            self._call_scan_service(
                client,
                f'trailer scan route {pose_name}',
                timeout=self.motion_timeout + 5.0,
            )
            with self.history_lock:
                for index, (_label, _frame, history) in enumerate(specs):
                    if locked[index] is None:
                        history.clear()

            deadline = time.monotonic() + self.scan_marker_pause + 2.0
            while time.monotonic() < deadline:
                if self.stop_event.wait(0.05):
                    return None, 'trailer scan route stopped'
                for index, (label, frame, history) in enumerate(specs):
                    if locked[index] is not None:
                        continue
                    pose, _reason = self.stable_marker_pose(
                        history=history,
                        yaw_only=True,
                    )
                    if pose is None:
                        continue
                    locked[index] = (
                        np.array(pose[0], dtype=np.float64),
                        np.array(pose[1], dtype=np.float64),
                    )
                    with self.history_lock:
                        self.scan_locked_frames.add(frame)
                    self.publish_status(
                        f'TRAILER SCAN: {label} saved at {pose_name}'
                    )
                if grouped_marker_locks_satisfied(
                    locked,
                    required_indices=(0,),
                    any_of_indices=(1, 2),
                ):
                    return tuple(locked), 'trailer route markers locked'

        missing = []
        if locked[0] is None:
            missing.append(specs[0][0])
        if locked[1] is None and locked[2] is None:
            missing.append('trailer ID 9/10')
        return None, (
            'HOME-A1-A2-A3 route ended before detecting '
            + ', '.join(missing)
        )

    def start_saved_destination_transfer(
        self, _request, response, destination_name
    ):
        """Scan one container ID 0-8 and transfer it to a saved destination."""
        if not self.execute_motion or self.motion_backend != 'moveit':
            response.success = False
            response.message = 'Saved transfer requires MoveIt execution'
            return response
        if not self.allow_full_pick or not self.offsets_configured:
            response.success = False
            response.message = 'Full pick and calibrated offsets are required'
            return response
        if destination_name not in self.saved_destination_poses:
            response.success = False
            response.message = (
                f'{destination_name} is not saved; call '
                '/arm2/scan_destinations first'
            )
            return response
        with self.stack_level_lock:
            placed_count = self.saved_destination_stack_counts[
                destination_name
            ]
        if placed_count >= self.max_stack_levels:
            response.success = False
            response.message = (
                f'{destination_name} already has the maximum '
                f'{self.max_stack_levels} layers'
            )
            return response
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'A robot motion is already running'
            return response
        self.stop_event.clear()
        self.motion_thread = threading.Thread(
            target=self.execute_saved_destination_transfer,
            args=(destination_name,),
            daemon=True,
        )
        self.motion_thread.start()
        response.success = True
        response.message = (
            f'Scanning container ID 0-8 for transfer to {destination_name}'
        )
        return response

    def execute_saved_destination_transfer(self, destination_name):
        try:
            specs = tuple(
                (
                    f'container ID {marker_id}',
                    frame,
                    self.marker_histories[marker_id],
                )
                for marker_id, frame in self.source_frames.items()
            )
            locked, reason = self._scan_named_markers(
                specs,
                required_count=len(specs),
                minimum_required_locks=1,
                accept_initial_pose=True,
            )
            if locked is None:
                self.publish_status(
                    f'{destination_name} TRANSFER FAILED: {reason}'
                )
                return
            source_marker_id, source_pose = next(
                (marker_id, pose)
                for marker_id, pose in zip(self.source_frames, locked)
                if pose is not None
            )
            source_targets, reason = self.calculate_targets_from_marker_pose(
                source_pose,
                orientation_mode=self.stack_source_orientation_mode,
            )
            if source_targets is None:
                self.publish_status(
                    f'{destination_name} TRANSFER FAILED: '
                    f'container ID {source_marker_id} target: {reason}'
                )
                self._recover_home_after_failure(destination_name)
                return
            destination_pose = self.saved_destination_poses[destination_name]
            with self.stack_level_lock:
                placed_count = self.saved_destination_stack_counts[
                    destination_name
                ]
            targets, reason = self.calculate_stack_targets_from_locked_poses(
                source_targets,
                destination_pose,
                placed_count=placed_count,
                destination_correction=self.saved_destination_correction,
            )
            if targets is None:
                self.publish_status(
                    f'{destination_name} TRANSFER FAILED: {reason}'
                )
                self._recover_home_after_failure(destination_name)
                return
            self.execute_scanned_transfer(
                targets,
                destination_name=destination_name,
                count_stack=False,
                saved_stack_name=destination_name,
                source_marker_id=source_marker_id,
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(
                f'{destination_name} TRANSFER FAILED: {exc}'
            )
            self._recover_home_after_failure(destination_name)

    def _scan_named_markers(
        self,
        specs,
        required_count=None,
        minimum_required_locks=None,
        accept_initial_pose=False,
        required_indices=None,
        any_of_indices=None,
    ):
        """Sweep J1 until every named marker has a stationary locked pose."""
        use_grouped_requirements = (
            required_indices is not None or any_of_indices is not None
        )
        if use_grouped_requirements:
            if required_count is not None or minimum_required_locks is not None:
                raise ValueError(
                    'grouped marker requirements cannot use count requirements'
                )
            required_indices = tuple(required_indices or ())
            any_of_indices = tuple(any_of_indices or ())
            selected_indices = required_indices + any_of_indices
            if not selected_indices or any(
                index < 0 or index >= len(specs)
                for index in selected_indices
            ):
                raise ValueError('marker requirement index is out of range')
            if len(set(selected_indices)) != len(selected_indices):
                raise ValueError('marker requirement indices must be unique')
            if not required_indices or not any_of_indices:
                raise ValueError(
                    'grouped marker requirements need required and any-of indices'
                )
        if required_count is None:
            required_count = len(specs)
        if not 1 <= required_count <= len(specs):
            raise ValueError('required_count must select at least one spec')
        if minimum_required_locks is None:
            minimum_required_locks = required_count
        if not 1 <= minimum_required_locks <= required_count:
            raise ValueError(
                'minimum_required_locks must be within required specs'
            )

        def scan_complete():
            if use_grouped_requirements:
                return grouped_marker_locks_satisfied(
                    locked,
                    required_indices,
                    any_of_indices,
                )
            return sum(
                pose is not None for pose in locked[:required_count]
            ) >= minimum_required_locks

        locked = [None] * len(specs)
        if accept_initial_pose:
            with self.history_lock:
                for _label, frame, history in specs:
                    history.clear()
                    self.scan_locked_frames.discard(frame)
            initial_deadline = time.monotonic() + self.scan_marker_pause + 2.0
            last_reason = 'no marker samples'
            while time.monotonic() < initial_deadline:
                for index, (label, frame, history) in enumerate(specs):
                    pose, last_reason = self.stable_marker_pose(
                        history=history,
                        yaw_only=True,
                    )
                    if pose is None:
                        continue
                    locked[index] = (
                        np.array(pose[0], dtype=np.float64),
                        np.array(pose[1], dtype=np.float64),
                    )
                    with self.history_lock:
                        self.scan_locked_frames.add(frame)
                    self.publish_status(
                        f'INITIAL VIEW: {label} position saved; '
                        'skipping J1 scan'
                    )
                    return tuple(locked), 'initial marker locked'
                if self.stop_event.wait(0.05):
                    return None, 'scan stopped'
            self.publish_status(
                'INITIAL VIEW: no stable container visible; starting J1 scan '
                f'({last_reason})'
            )

        clients = (
            self.sweep_joint1_client,
            self.pause_sweep_client,
            self.resume_sweep_client,
            self.stop_robot_client,
            self.scan_state_client,
        )
        if any(
            client is None or not client.wait_for_service(timeout_sec=3.0)
            for client in clients
        ):
            return None, 'scan control service is unavailable'
        with self.history_lock:
            for _label, frame, history in specs:
                history.clear()
                self.scan_locked_frames.discard(frame)
        deadline = time.monotonic() + self.scan_timeout
        pass_number = 0
        while time.monotonic() < deadline and not scan_complete():
            pass_number += 1
            missing = ', '.join(
                label for (label, _frame, _history), pose
                in zip(specs, locked) if pose is None
            )
            self.publish_status(
                f'SCAN: J1 pass {pass_number}; missing {missing}'
            )
            sweep_future = self.sweep_joint1_client.call_async(
                Trigger.Request()
            )
            if not self._wait_for_joint1_scan_active(5.0):
                self.stop_robot_client.call_async(Trigger.Request())
                return None, 'J1 scan did not become active'
            # Discard samples collected before the sweep owned the hardware.
            # Otherwise an already-visible marker can be accepted before the
            # scan callback starts, and its early stop request can be lost.
            with self.history_lock:
                for index, (_label, _frame, history) in enumerate(specs):
                    if locked[index] is None:
                        history.clear()
            while not sweep_future.done():
                if self.stop_event.is_set():
                    self.stop_robot_client.call_async(Trigger.Request())
                    return None, 'scan stopped'
                candidate = next((
                    index for index, (_label, _frame, history)
                    in enumerate(specs)
                    if locked[index] is None
                    and self._history_sample_count(history) > 0
                ), None)
                if candidate is None:
                    time.sleep(0.05)
                    continue
                label, frame, history = specs[candidate]
                pose, reason = self._pause_and_lock_marker(label, history)
                if pose is not None:
                    locked[candidate] = pose
                    with self.history_lock:
                        self.scan_locked_frames.add(frame)
                    self.publish_status(
                        f'SCAN: {label} position saved and locked'
                    )
                else:
                    self.publish_status(
                        f'SCAN: {label} lock retry: {reason}'
                    )
                if scan_complete():
                    self._call_scan_service(
                        self.stop_robot_client,
                        'stop completed scan',
                        timeout=5.0,
                    )
                    if not self._wait_for_joint1_scan_inactive(5.0):
                        return None, 'J1 scan did not stop after marker lock'
                    # The physical sweep is stopped and its execution lock has
                    # been released.  Do not wait on the original long-running
                    # Trigger future: DDS may deliver that response late even
                    # though the hardware is already idle.
                    return tuple(locked), 'markers locked'
                self._call_scan_service(
                    self.resume_sweep_client, 'resume sweep'
                )
            if not sweep_future.done():
                self._wait_future(sweep_future, 10.0)
            result = sweep_future.result()
            if result is None or not result.success:
                message = 'no response' if result is None else result.message
                self._return_home_after_failed_scan()
                return None, f'J1 scan failed: {message}'
        if not scan_complete():
            self._return_home_after_failed_scan()
            missing = ', '.join(
                label for (label, _frame, _history), pose
                in zip(specs[:required_count], locked[:required_count])
                if pose is None
            )
            return None, (
                f'{self.scan_timeout:.0f}s scan ended before detecting '
                f'{missing}'
            )
        return tuple(locked), 'markers locked'

    def _wait_for_joint1_scan_active(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            future = self.scan_state_client.call_async(Trigger.Request())
            try:
                self._wait_future(future, 1.0)
            except RuntimeError:
                continue
            result = future.result()
            if result is not None and result.success:
                return True
            if self.stop_event.wait(0.05):
                return False
        return False

    def _wait_for_joint1_scan_inactive(self, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            future = self.scan_state_client.call_async(Trigger.Request())
            try:
                self._wait_future(future, 1.0)
            except RuntimeError:
                continue
            result = future.result()
            if result is not None and not result.success:
                return True
            if self.stop_event.wait(0.05):
                return False
        return False

    def reset_stack_level(self, _request, response):
        """Reset automatic stacking so the next placement uses layer one."""
        if self.motion_thread is not None and self.motion_thread.is_alive():
            response.success = False
            response.message = 'Cannot reset while robot motion is running'
            return response
        with self.stack_level_lock:
            self.placed_stack_count = 0
            for name in self.saved_destination_stack_counts:
                self.saved_destination_stack_counts[name] = 0
        self.publish_status(
            'STACK: all layer counters reset; next layer is 1'
        )
        response.success = True
        response.message = (
            'A-1-1 through A-3-2 stack levels reset; next placement is layer 1'
        )
        return response

    def execute_scan_and_transfer(self):
        """Scan and transfer primary container ID 0 to A-1 ID 11."""
        try:
            locked, reason = self.scan_and_lock_marker_poses()
            if locked is None:
                self.publish_status(f'SCAN TRANSFER FAILED: {reason}')
                return
            source_pose, destination_pose = locked
            source_targets, reason = self.calculate_targets_from_marker_pose(
                source_pose,
                orientation_mode=self.stack_source_orientation_mode,
            )
            if source_targets is None:
                self.publish_status(
                    f'SCAN TRANSFER FAILED: ID 0 target: {reason}'
                )
                self._recover_home_after_failure('SCAN TRANSFER')
                return
            targets, reason = self.calculate_stack_targets_from_locked_poses(
                source_targets,
                destination_pose,
            )
            if targets is None:
                self.publish_status(f'SCAN TRANSFER FAILED: {reason}')
                self._recover_home_after_failure('SCAN TRANSFER')
                return
            self.execute_scanned_transfer(targets)
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'SCAN TRANSFER FAILED: {exc}')
            self._recover_home_after_failure('SCAN TRANSFER')

    def scan_and_lock_marker_poses(self):
        """Sweep J1 and pause at whichever required marker appears."""
        clients = (
            self.sweep_joint1_client,
            self.pause_sweep_client,
            self.resume_sweep_client,
            self.stop_robot_client,
        )
        if any(
            not client.wait_for_service(timeout_sec=3.0)
            for client in clients
        ):
            return None, 'scan control service is unavailable'
        with self.history_lock:
            self.history.clear()
            self.stack_target_history.clear()
            self.scan_locked_frames.clear()
        locked = [None, None]
        histories = (self.history, self.stack_target_history)
        labels = ('ID 0', 'ID 11')
        scan_deadline = time.monotonic() + self.scan_timeout
        scan_error = None
        pass_number = 0
        while (
            time.monotonic() < scan_deadline
            and not all(value is not None for value in locked)
        ):
            pass_number += 1
            self.publish_status(
                f'SCAN: J1 pass {pass_number} for missing ID 0/ID 11'
            )
            sweep_future = self.sweep_joint1_client.call_async(
                Trigger.Request()
            )
            while not sweep_future.done():
                if self.stop_event.is_set():
                    self.stop_robot_client.call_async(Trigger.Request())
                    return None, 'scan stopped'
                candidate = next(
                    (
                        index
                        for index, history in enumerate(histories)
                        if locked[index] is None
                        and self._history_sample_count(history) > 0
                    ),
                    None,
                )
                if candidate is None:
                    time.sleep(0.05)
                    continue
                pose, reason = self._pause_and_lock_marker(
                    labels[candidate],
                    histories[candidate],
                )
                if pose is not None:
                    locked[candidate] = pose
                    with self.history_lock:
                        self.scan_locked_frames.add(
                            (
                                self.marker_frame,
                                self.stack_target_frame,
                            )[candidate]
                        )
                    self.publish_status(
                        f'SCAN: {labels[candidate]} position saved and locked'
                    )
                else:
                    self.publish_status(
                        f'SCAN: {labels[candidate]} lock retry: {reason}'
                    )
                if all(value is not None for value in locked):
                    self.publish_status(
                        'SCAN: ID 0 and ID 11 saved; stopping scan'
                    )
                    self.stop_robot_client.call_async(Trigger.Request())
                    break
                self._call_scan_service(
                    self.resume_sweep_client, 'resume sweep'
                )
            if not sweep_future.done():
                self._wait_future(sweep_future, 10.0)
            result = sweep_future.result()
            if result is None or not result.success:
                message = 'no response' if result is None else result.message
                scan_error = f'J1 scan failed: {message}'
                break
        if scan_error is not None:
            self._return_home_after_failed_scan()
            return None, scan_error
        if not all(value is not None for value in locked):
            self._return_home_after_failed_scan()
            missing = ', '.join(
                label for label, value in zip(labels, locked)
                if value is None
            )
            return None, (
                f'{self.scan_timeout:.0f}s scan ended before detecting '
                f'{missing}'
            )
        return tuple(locked), 'markers locked'

    def _return_home_after_failed_scan(self):
        """Return home when scanning cannot proceed to a transfer."""
        self._recover_home_after_failure('SCAN')

    def _recover_home_after_failure(self, label):
        """Stop, wait five seconds, then make a best-effort home return."""
        self._stop_active_motion()
        self.publish_status(
            f'{label}: failure recovery; returning home in 5 seconds'
        )
        time.sleep(5.0)
        self.stop_event.clear()
        try:
            self.command_return_home()
        except Exception as exc:
            self.publish_status(
                f'{label}: automatic failure home return failed: {exc}'
            )
            return
        self.publish_status(
            f'{label}: automatic failure home return completed'
        )

    def _history_sample_count(self, history):
        with self.history_lock:
            return len(history)

    def _pause_and_lock_marker(self, label, history):
        self._call_scan_service(
            self.pause_sweep_client, 'pause sweep', timeout=5.0
        )
        # The bridge checks its pause event every 0.1 s and stops serial
        # motion there. Wait before clearing samples so the 0.5 s capture
        # window contains only stationary observations.
        if self.stop_event.wait(0.2):
            return None, 'scan stopped'
        with self.history_lock:
            history.clear()
        self.publish_status(
            f'SCAN: {label} detected; holding '
            f'{self.scan_marker_pause:.1f}s'
        )
        deadline = time.monotonic() + self.scan_marker_pause + 2.0
        minimum_hold_end = time.monotonic() + self.scan_marker_pause
        reason = 'no stable samples'
        while time.monotonic() < deadline:
            if self.stop_event.wait(0.05):
                return None, 'scan stopped'
            if time.monotonic() < minimum_hold_end:
                continue
            pose, reason = self.stable_marker_pose(
                history=history,
                yaw_only=True,
            )
            if pose is not None:
                return (
                    np.array(pose[0], dtype=np.float64),
                    np.array(pose[1], dtype=np.float64),
                ), 'stable'
        return None, reason

    def _call_scan_service(self, client, label, timeout=3.0):
        future = client.call_async(Trigger.Request())
        self._wait_future(future, timeout)
        result = future.result()
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            raise RuntimeError(f'{label} failed: {message}')

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
            self.publish_pick_target('PICK', initial_targets[0])
            self._perform_pick(initial_targets)
            self.publish_status('PICK: returning to home')
            self.command_return_home()
            self.publish_status('PICK: completed')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'PICK FAILED: {exc}')
            self._recover_home_after_failure('PICK')
        finally:
            self.motion_lock.release()

    def _perform_pick(
        self,
        initial_targets,
        allow_yaw_fallback=True,
        allow_segmented_descent=False,
        marker_frame=None,
        marker_history=None,
        marker_id=0,
        search_higher_pregrasp=False,
    ):
        grasp, pregrasp = map(copy.deepcopy, initial_targets)
        self.publish_status('PICK: opening gripper')
        self.command_gripper(open_gripper=True)
        self.publish_status('PICK: moving to pregrasp')

        def move_to_pick_pregrasp(base_pregrasp):
            nonlocal pregrasp
            height_offsets = (
                (0.0, 0.02, 0.04, 0.06)
                if search_higher_pregrasp else (0.0,)
            )
            failures = []
            for height_offset in height_offsets:
                candidate = copy.deepcopy(base_pregrasp)
                candidate.pose.position.z += height_offset
                translation = np.array([
                    candidate.pose.position.x,
                    candidate.pose.position.y,
                    candidate.pose.position.z,
                ])
                if not self.in_workspace(translation):
                    failures.append(
                        f'+{height_offset * 100.0:.0f}cm outside workspace'
                    )
                    continue
                pregrasp = candidate
                try:
                    if self.prefer_z_last_motion:
                        self.move_to_pose_z_last(
                            pregrasp,
                            keep_current_orientation=(
                                self.pregrasp_test_keep_orientation
                            ),
                            require_preferred_pick_branch=True,
                            descent_preflight_target=(
                                grasp if search_higher_pregrasp else None
                            ),
                        )
                    else:
                        self.move_to_pose(
                            pregrasp,
                            keep_current_orientation=(
                                self.pregrasp_test_keep_orientation
                            ),
                            require_preferred_pick_branch=True,
                            descent_preflight_target=(
                                grasp if search_higher_pregrasp else None
                            ),
                        )
                    if height_offset > 0.0:
                        self.publish_status(
                            'PICK: selected higher pregrasp '
                            f'+{height_offset * 100.0:.0f}cm'
                        )
                    return
                except RuntimeError as exc:
                    if self.stop_event.is_set():
                        raise
                    failures.append(
                        f'+{height_offset * 100.0:.0f}cm: {exc}'
                    )
            raise RuntimeError('; '.join(failures))

        nominal_grasp = copy.deepcopy(grasp)
        nominal_pregrasp = copy.deepcopy(pregrasp)
        yaw_offsets = [0.0]
        if search_higher_pregrasp:
            # Near the edge of the workspace, an exact marker yaw can force
            # the vertical descent through an IK branch discontinuity.  Test
            # small, symmetric grasp-yaw deviations before the 180-degree
            # equivalent and execute only a candidate whose complete descent
            # passed preflight.
            yaw_offsets.extend((15.0, -15.0, 30.0, -30.0))
        if abs(self.container_yaw_symmetry - 180.0) <= 1e-6:
            yaw_offsets.append(180.0)

        yaw_failures = []
        selected = False
        for yaw_offset in yaw_offsets:
            grasp = copy.deepcopy(nominal_grasp)
            candidate_pregrasp = copy.deepcopy(nominal_pregrasp)
            for pose in (grasp, candidate_pregrasp):
                rpy = quaternion_to_rpy_degrees([
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ])
                rotation = quaternion_from_rpy_degrees(
                    rpy[0], rpy[1], wrap_degrees(rpy[2] + yaw_offset)
                )
                (
                    pose.pose.orientation.x,
                    pose.pose.orientation.y,
                    pose.pose.orientation.z,
                    pose.pose.orientation.w,
                ) = map(float, rotation)
            if yaw_offset != 0.0:
                candidate_yaw = quaternion_to_rpy_degrees([
                    candidate_pregrasp.pose.orientation.x,
                    candidate_pregrasp.pose.orientation.y,
                    candidate_pregrasp.pose.orientation.z,
                    candidate_pregrasp.pose.orientation.w,
                ])[2]
                equivalent = (
                    ' equivalent' if abs(yaw_offset) == 180.0 else ''
                )
                self.publish_status(
                    'PICK: retrying pregrasp/descent preflight with '
                    f'yaw offset {yaw_offset:+.0f}deg{equivalent} '
                    f'(target={candidate_yaw:.2f}deg)'
                )
            try:
                move_to_pick_pregrasp(candidate_pregrasp)
                selected = True
                if yaw_offset != 0.0:
                    self.publish_status(
                        'PICK: selected preflight-safe grasp yaw offset '
                        f'{yaw_offset:+.0f}deg'
                    )
                break
            except RuntimeError as exc:
                if self.stop_event.is_set():
                    raise
                yaw_failures.append(f'yaw {yaw_offset:+.0f}deg: {exc}')
        if not selected and search_higher_pregrasp:
            # A vertical line can cross an IK singularity even though the
            # grasp itself is reachable.  As a final safe fallback, approach
            # the last few centimetres on a shallow diagonal.  The same
            # bottom-up IK and full Cartesian preflight are still required,
            # so this does not relax the joint-jump protection.
            oblique_offsets = (
                (0.010, 0.0),
                (-0.010, 0.0),
                (0.0, 0.010),
                (0.0, -0.010),
            )
            grasp = copy.deepcopy(nominal_grasp)
            for dx, dy in oblique_offsets:
                candidate_pregrasp = copy.deepcopy(nominal_grasp)
                candidate_pregrasp.pose.position.x += dx
                candidate_pregrasp.pose.position.y += dy
                candidate_pregrasp.pose.position.z += 0.025
                translation = np.array([
                    candidate_pregrasp.pose.position.x,
                    candidate_pregrasp.pose.position.y,
                    candidate_pregrasp.pose.position.z,
                ])
                if not self.in_workspace(translation):
                    yaw_failures.append(
                        'oblique '
                        f'dx={dx * 1000.0:+.0f}mm, '
                        f'dy={dy * 1000.0:+.0f}mm outside workspace'
                    )
                    continue
                self.publish_status(
                    'PICK: testing short oblique approach '
                    f'dx={dx * 1000.0:+.0f}mm, '
                    f'dy={dy * 1000.0:+.0f}mm, dz=+25mm'
                )
                try:
                    if self.prefer_z_last_motion:
                        self.move_to_pose_z_last(
                            candidate_pregrasp,
                            keep_current_orientation=(
                                self.pregrasp_test_keep_orientation
                            ),
                            require_preferred_pick_branch=True,
                            descent_preflight_target=grasp,
                        )
                    else:
                        self.move_to_pose(
                            candidate_pregrasp,
                            keep_current_orientation=(
                                self.pregrasp_test_keep_orientation
                            ),
                            require_preferred_pick_branch=True,
                            descent_preflight_target=grasp,
                        )
                    pregrasp = candidate_pregrasp
                    selected = True
                    self.publish_status(
                        'PICK: selected preflight-safe short oblique '
                        f'approach dx={dx * 1000.0:+.0f}mm, '
                        f'dy={dy * 1000.0:+.0f}mm'
                    )
                    break
                except RuntimeError as exc:
                    if self.stop_event.is_set():
                        raise
                    yaw_failures.append(
                        'oblique '
                        f'dx={dx * 1000.0:+.0f}mm, '
                        f'dy={dy * 1000.0:+.0f}mm: {exc}'
                    )
        if not selected:
            raise RuntimeError(
                'no pregrasp/grasp yaw or short oblique candidate passed '
                'complete descent preflight: ' + '; '.join(yaw_failures)
            )
        if (
            self.visual_servo_enabled
            and marker_frame is not None
            and marker_history is not None
        ):
            # The wrist camera loses the top marker after final grasp-yaw
            # alignment. Refine XY while retaining the visible wrist attitude,
            # then use the last observed marker yaw for one final alignment.
            grasp, pregrasp = self.refine_pregrasp_with_visual_feedback(
                (grasp, pregrasp),
                marker_frame,
                marker_history,
                marker_id,
                correct_yaw=False,
            )
        if self.pregrasp_test_keep_orientation:
            self.publish_status('PICK: aligning at pregrasp')
            self.move_to_pose(
                pregrasp,
                require_preferred_pick_branch=True,
            )
        if (
            not self.visual_servo_enabled
            and self.refresh_marker_before_descent
        ):
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
        self.avoid_known_bad_pick_branch(grasp)
        try:
            self.move_cartesian_to_pose(
                grasp,
                allow_segmented=allow_segmented_descent,
                prefer_j2_branch_fallback=allow_segmented_descent,
            )
        except CartesianPlanningError as initial_error:
            if (
                initial_error.executed_segments == 0
                and 'ik_only=1.000' in str(initial_error)
                and 'no_joint_jump=' in str(initial_error)
                and self.try_opposite_ik_branch(
                    grasp, 'PICK vertical descent'
                )
            ):
                self.publish_status(
                    'PICK: retrying vertical descent on opposite J2 branch'
                )
                self.move_cartesian_to_pose(
                    grasp,
                    allow_segmented=allow_segmented_descent,
                    prefer_j2_branch_fallback=allow_segmented_descent,
                )
                initial_error = None
            if initial_error is None:
                pass
            else:
                remaining_distance = self._cartesian_request_distance(grasp)
                can_finish_with_pose_goal = (
                    allow_segmented_descent
                    and initial_error.executed_segments > 0
                    and remaining_distance is not None
                    and remaining_distance
                    <= self.stack_pose_goal_finish_max_distance
                )
                if can_finish_with_pose_goal:
                    self.publish_status(
                        'PICK: finishing locked grasp pose without marker: '
                        f'remaining={remaining_distance * 1000.0:.1f}mm'
                    )
                    self.move_to_pose(grasp)
                elif not allow_yaw_fallback:
                    raise initial_error
                else:
                    grasp = self.move_with_yaw_fallbacks(
                        grasp, pregrasp, initial_error
                    )
        self.publish_status('PICK: closing gripper')
        self.command_gripper(open_gripper=False)
        self.publish_status('PICK: finding vertical lift path')
        self.move_adaptive_cartesian_lift(grasp)
        return grasp

    def avoid_known_bad_pick_branch(self, grasp):
        """Leave a measured bad J2/J3 branch before grasp descent."""
        if self.motion_backend != 'moveit':
            return
        with self.joint_state_lock:
            joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')
        j2_deg = math.degrees(float(joints[1]))
        j3_deg = math.degrees(float(joints[2]))
        is_bad_branch = (
            abs(j2_deg - self.bad_pick_branch_j2)
            <= self.bad_pick_branch_tolerance
            and abs(j3_deg - self.bad_pick_branch_j3)
            <= self.bad_pick_branch_tolerance
        )
        if not is_bad_branch:
            return
        self.publish_status(
            'PICK: known bad pregrasp branch rejected: '
            f'J2={j2_deg:.1f}deg, J3={j3_deg:.1f}deg; '
            'switching J2 and J3 together before descent'
        )
        if self._try_opposite_j2_j3_branch('PICK known bad branch'):
            return
        raise RuntimeError(
            'known bad pick branch detected and no safe IK solution with '
            'both J2 and J3 reversed was found'
        )

    def publish_pick_target(self, label, grasp, marker_id=0):
        yaw = quaternion_to_rpy_degrees([
            grasp.pose.orientation.x,
            grasp.pose.orientation.y,
            grasp.pose.orientation.z,
            grasp.pose.orientation.w,
        ])[2]
        self.publish_status(
            f'{label}: ID {marker_id} grasp target '
            f'x={grasp.pose.position.x:.4f}, '
            f'y={grasp.pose.position.y:.4f}, '
            f'z={grasp.pose.position.z:.4f}, yaw={yaw:.2f}deg'
        )

    def execute_stack(self, targets):
        if not self.motion_lock.acquire(blocking=False):
            return
        self.tracking_suspended.set()
        try:
            self.j2_fallback_used = False
            self.j3_fallback_used = False
            self.ik_seed_fallback_index = 0
            self.ik_seed_fallback_solutions = set()
            source_targets, release, approach = targets
            self.publish_status(
                'STACK: using locked initial ID 0 and ID 1 poses'
            )
            source_targets = copy.deepcopy(source_targets)
            self.publish_pick_target('STACK', source_targets[0])
            picked_grasp = self._perform_pick(
                source_targets,
                allow_yaw_fallback=False,
                allow_segmented_descent=True,
            )
            self.raise_to_common_clearance_before_j1(release)
            self.move_joint1_toward_destination(picked_grasp, release)
            transit_approach = (
                self.move_to_destination_radius_at_clearance(release)
            )
            destination_yaw = quaternion_to_rpy_degrees([
                release.pose.orientation.x,
                release.pose.orientation.y,
                release.pose.orientation.z,
                release.pose.orientation.w,
            ])[2]
            self.publish_status(
                'STACK: moving above ID 1 and aligning heading: '
                f'tcp_yaw={destination_yaw:.2f}deg'
            )
            approach = self.move_to_reachable_stack_release(
                release,
                approach.pose.orientation,
                'STACK release',
                alignment_pose=transit_approach,
            )
            try:
                self.verify_release_pose(release, 'STACK release')
            except RuntimeError:
                self.publish_status(
                    'STACK: release verification failed; retreating '
                    'without opening gripper'
                )
                self.move_segmented_cartesian_with_pose_finish(
                    approach, 'STACK verification retreat'
                )
                raise
            self.publish_status('STACK: opening gripper')
            self.command_gripper(open_gripper=True)
            self.publish_status('STACK: retreating vertically')
            self.move_segmented_cartesian_with_pose_finish(
                approach, 'STACK retreat'
            )
            self.publish_status('STACK: returning to home')
            self.command_return_home()
            self.publish_status('STACK: completed ID 0 onto ID 1')
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'STACK FAILED: {exc}')
            self._recover_home_after_failure('STACK')
        finally:
            self.tracking_suspended.clear()
            self.motion_lock.release()

    def execute_scanned_transfer(
        self,
        targets,
        destination_name='A-1',
        count_stack=True,
        saved_stack_name=None,
        source_marker_id=0,
        align_source_before_pick=False,
        source_scan_zone=None,
    ):
        """Move a scan-locked container to one locked destination."""
        if not self.motion_lock.acquire(blocking=False):
            return
        try:
            self.j2_fallback_used = False
            self.j3_fallback_used = False
            self.ik_seed_fallback_index = 0
            self.ik_seed_fallback_solutions = set()
            source_targets, release, approach = targets
            self.publish_status('TRANSFER: moving to saved container pose')
            self.publish_pick_target(
                'TRANSFER', source_targets[0], marker_id=source_marker_id
            )
            if align_source_before_pick:
                self.move_joint1_toward_source(
                    source_targets[0], source_scan_zone
                )
            picked_grasp = self._perform_pick(
                copy.deepcopy(source_targets),
                allow_yaw_fallback=False,
                allow_segmented_descent=True,
                marker_frame=self.source_frames[source_marker_id],
                marker_history=self.marker_histories[source_marker_id],
                marker_id=source_marker_id,
                search_higher_pregrasp=align_source_before_pick,
            )
            self.raise_to_common_clearance_before_j1(release)
            self.move_joint1_toward_destination(
                picked_grasp,
                release,
                destination_name=destination_name,
            )
            transit_approach = (
                self.move_to_destination_radius_at_clearance(release)
            )
            self.tracking_suspended.set()
            destination_yaw = quaternion_to_rpy_degrees([
                release.pose.orientation.x,
                release.pose.orientation.y,
                release.pose.orientation.z,
                release.pose.orientation.w,
            ])[2]
            self.publish_status(
                f'TRANSFER: container picked; moving to {destination_name}: '
                f'tcp_yaw={destination_yaw:.2f}deg'
            )
            approach = self.move_to_reachable_stack_release(
                release,
                approach.pose.orientation,
                f'{destination_name} release',
                alignment_pose=transit_approach,
            )
            try:
                self.verify_release_pose(
                    release, f'{destination_name} release'
                )
            except RuntimeError:
                self.publish_status(
                    f'TRANSFER: {destination_name} release verification '
                    'failed; retreating without opening gripper'
                )
                self.move_segmented_cartesian_with_pose_finish(
                    approach, f'{destination_name} verification retreat'
                )
                raise
            self.publish_status(
                f'TRANSFER: releasing container at {destination_name}'
            )
            self.command_gripper(open_gripper=True)
            self.publish_status(
                f'TRANSFER: retreating from {destination_name}'
            )
            self.move_segmented_cartesian_with_pose_finish(
                approach, f'{destination_name} retreat'
            )
            if count_stack:
                with self.stack_level_lock:
                    self.placed_stack_count += 1
                    completed_layer = self.placed_stack_count
                self.publish_status(
                    f'TRANSFER: stack layer {completed_layer} placed'
                )
            elif saved_stack_name is not None:
                with self.stack_level_lock:
                    self.saved_destination_stack_counts[
                        saved_stack_name
                    ] += 1
                    completed_layer = self.saved_destination_stack_counts[
                        saved_stack_name
                    ]
                self.publish_status(
                    f'TRANSFER: {saved_stack_name} layer '
                    f'{completed_layer}/{self.max_stack_levels} placed'
                )
            self.publish_status('TRANSFER: returning to final home')
            self.command_return_home()
            self.publish_status(
                f'TRANSFER: container to {destination_name} completed'
            )
        except Exception as exc:
            self.stop_event.set()
            self._stop_active_motion()
            self.publish_status(f'TRANSFER FAILED: {exc}')
            self._recover_home_after_failure('TRANSFER')
        finally:
            self.tracking_suspended.clear()
            self.motion_lock.release()

    def raise_to_common_clearance_before_j1(self, release):
        """Reach the source/destination common safe Z before rotating J1."""
        try:
            current = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_z = float(current.transform.translation.z)
        destination_z = (
            float(release.pose.position.z)
            + self.stack_approach_clearance
        )
        common_z = max(current_z, destination_z)
        if current_z >= common_z - 0.003:
            return
        vertical = copy.deepcopy(release)
        vertical.pose.position.x = float(current.transform.translation.x)
        vertical.pose.position.y = float(current.transform.translation.y)
        vertical.pose.position.z = common_z
        vertical.pose.orientation = copy.deepcopy(
            current.transform.rotation
        )
        self.publish_status(
            'TRANSFER: raising to common destination clearance before J1: '
            f'extra={(common_z - current_z) * 1000.0:.1f}mm'
        )
        try:
            self.move_segmented_cartesian_with_pose_finish(
                vertical, 'TRANSFER pre-J1 clearance rise'
            )
            return
        except CartesianPlanningError as initial_error:
            if initial_error.executed_segments > 0:
                raise
            initial_failure = str(initial_error)

        current_rpy = quaternion_to_rpy_degrees([
            current.transform.rotation.x,
            current.transform.rotation.y,
            current.transform.rotation.z,
            current.transform.rotation.w,
        ])
        failures = [f'fixed orientation: {initial_failure}']
        for roll_adjustment, pitch_adjustment in (
            (0.0, 5.0), (0.0, -5.0),
            (5.0, 0.0), (-5.0, 0.0),
            (0.0, 10.0), (0.0, -10.0),
            (10.0, 0.0), (-10.0, 0.0),
            (5.0, 5.0), (5.0, -5.0),
            (-5.0, 5.0), (-5.0, -5.0),
        ):
            candidate = copy.deepcopy(vertical)
            rotation = quaternion_from_rpy_degrees(
                current_rpy[0] + roll_adjustment,
                current_rpy[1] + pitch_adjustment,
                current_rpy[2],
            )
            (
                candidate.pose.orientation.x,
                candidate.pose.orientation.y,
                candidate.pose.orientation.z,
                candidate.pose.orientation.w,
            ) = map(float, rotation)
            self.publish_status(
                'TRANSFER: retrying clearance rise with bounded tilt: '
                f'roll={roll_adjustment:+.0f}deg, '
                f'pitch={pitch_adjustment:+.0f}deg'
            )
            try:
                self.move_segmented_cartesian_with_pose_finish(
                    candidate, 'TRANSFER bounded-tilt clearance rise'
                )
                return
            except CartesianPlanningError as exc:
                if exc.executed_segments > 0:
                    raise
                failures.append(
                    f'roll {roll_adjustment:+.0f}/'
                    f'pitch {pitch_adjustment:+.0f}: {exc}'
                )
        if self.try_opposite_ik_branch(
            release, 'TRANSFER pre-J1 clearance rise'
        ):
            return self.raise_to_common_clearance_before_j1(release)
        raise RuntimeError(
            'No safe pre-J1 clearance rise within 10deg tilt: '
            + '; '.join(failures)
        )

    def move_to_destination_radius_at_clearance(self, release):
        """Match destination XY at safe Z without rotating the wrist."""
        try:
            current = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        destination_z = (
            float(release.pose.position.z)
            + self.stack_approach_clearance
        )
        transit_z = max(
            float(current.transform.translation.z), destination_z
        )
        orientation = copy.deepcopy(current.transform.rotation)

        if float(current.transform.translation.z) < transit_z - 0.003:
            vertical = copy.deepcopy(release)
            vertical.pose.position.x = float(
                current.transform.translation.x
            )
            vertical.pose.position.y = float(
                current.transform.translation.y
            )
            vertical.pose.position.z = transit_z
            vertical.pose.orientation = copy.deepcopy(orientation)
            self.publish_status(
                'TRANSFER: restoring configured clearance after J1: '
                f'{self.stack_approach_clearance * 1000.0:.0f}mm'
            )
            self.move_segmented_cartesian_with_pose_finish(
                vertical, 'TRANSFER clearance rise'
            )

        radial = copy.deepcopy(release)
        radial.pose.position.z = transit_z
        radial.pose.orientation = copy.deepcopy(orientation)
        radial_translation = np.array([
            radial.pose.position.x,
            radial.pose.position.y,
            radial.pose.position.z,
        ])
        if not self.in_workspace(radial_translation):
            raise RuntimeError(
                'destination radial waypoint outside workspace: '
                f'{np.round(radial_translation, 4).tolist()}'
            )
        self.publish_status(
            'TRANSFER: matching destination XY radius at safe height '
            'while holding wrist orientation'
        )
        current_rpy = quaternion_to_rpy_degrees([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        radial_failures = []
        radial_aligned = False
        yaw_candidates = (
            0.0, 5.0, -5.0, 10.0, -10.0, 15.0, -15.0,
            30.0, -30.0, 45.0, -45.0, 90.0, -90.0, 180.0,
        )
        orientation_candidates = [
            (yaw, 0.0, 0.0) for yaw in yaw_candidates
        ]
        orientation_candidates.extend(
            (yaw, roll, pitch)
            for yaw in (0.0, 45.0, 90.0)
            for roll, pitch in (
                (0.0, -5.0), (0.0, 5.0),
                (0.0, -10.0), (0.0, 10.0),
                (-5.0, 0.0), (5.0, 0.0),
            )
        )
        for yaw_adjustment, roll_adjustment, pitch_adjustment in (
            orientation_candidates
        ):
            candidate = copy.deepcopy(radial)
            candidate_rotation = quaternion_from_rpy_degrees(
                current_rpy[0] + roll_adjustment,
                current_rpy[1] + pitch_adjustment,
                wrap_degrees(current_rpy[2] + yaw_adjustment),
            )
            (
                candidate.pose.orientation.x,
                candidate.pose.orientation.y,
                candidate.pose.orientation.z,
                candidate.pose.orientation.w,
            ) = map(float, candidate_rotation)
            try:
                # This is a loaded horizontal transit. Never enter a partial
                # XY move that can strand the arm between destination zones.
                self.move_cartesian_to_pose(
                    candidate,
                    allow_segmented=False,
                )
                radial = candidate
                radial_aligned = True
                self.publish_status(
                    'TRANSFER: destination radius matched with wrist '
                    f'yaw={yaw_adjustment:+.0f}deg, '
                    f'roll={roll_adjustment:+.0f}deg, '
                    f'pitch={pitch_adjustment:+.0f}deg'
                )
                break
            except CartesianPlanningError as exc:
                if exc.executed_segments > 0:
                    raise
                radial_failures.append(
                    f'yaw {yaw_adjustment:+.0f}/'
                    f'roll {roll_adjustment:+.0f}/'
                    f'pitch {pitch_adjustment:+.0f}deg: {exc}'
                )
        if not radial_aligned:
            if self.try_opposite_ik_branch(
                release, 'TRANSFER radial alignment'
            ):
                return self.move_to_destination_radius_at_clearance(
                    release
                )
            raise RuntimeError(
                'No Cartesian radial path at safe height with bounded wrist '
                'yaw changes: ' + '; '.join(radial_failures)
            )

        return radial

    def move_to_reachable_stack_approach(
        self,
        release,
        orientation,
        excluded_yaw_offsets=None,
        alignment_pose=None,
        descent_orientation=None,
    ):
        """Find a reachable clearance and yaw without changing target XYZ."""
        failures = []
        excluded_yaw_offsets = set(excluded_yaw_offsets or ())
        base_rpy = quaternion_to_rpy_degrees([
            orientation.x,
            orientation.y,
            orientation.z,
            orientation.w,
        ])
        try:
            current = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_yaw = quaternion_to_rpy_degrees([
            current.transform.rotation.x,
            current.transform.rotation.y,
            current.transform.rotation.z,
            current.transform.rotation.w,
        ])[2]
        yaw_offsets = sorted(
            self.stack_yaw_fallback_offsets,
            key=lambda offset: abs(wrap_degrees(
                base_rpy[2] + offset - current_yaw
            )),
        )
        self.publish_status(
            'STACK: yaw candidates ordered for minimum wrist rotation: '
            + ', '.join(f'{offset:+.0f}deg' for offset in yaw_offsets)
        )
        if alignment_pose is None:
            approach_candidates = [
                (
                    clearance,
                    None,
                )
                for clearance in lift_distance_candidates(
                    self.stack_approach_clearance,
                    self.stack_minimum_approach_clearance,
                    self.stack_approach_search_step,
                )
            ]
        else:
            clearance = (
                float(alignment_pose.pose.position.z)
                - float(release.pose.position.z)
            )
            approach_candidates = [(clearance, alignment_pose)]
        for clearance, fixed_approach in approach_candidates:
            for yaw_offset in yaw_offsets:
                if yaw_offset in excluded_yaw_offsets:
                    continue
                candidate_rotation = quaternion_from_rpy_degrees(
                    base_rpy[0],
                    base_rpy[1],
                    wrap_degrees(base_rpy[2] + yaw_offset),
                )
                approach = copy.deepcopy(
                    release if fixed_approach is None else fixed_approach
                )
                if fixed_approach is None:
                    approach.pose.position.z += clearance
                (
                    approach.pose.orientation.x,
                    approach.pose.orientation.y,
                    approach.pose.orientation.z,
                    approach.pose.orientation.w,
                ) = map(float, candidate_rotation)
                translation = np.array([
                    approach.pose.position.x,
                    approach.pose.position.y,
                    approach.pose.position.z,
                ])
                if not self.in_workspace(translation):
                    failures.append(
                        f'yaw {yaw_offset:+.0f}deg/{clearance:.3f}m: '
                        'outside workspace'
                    )
                    continue
                self.publish_status(
                    'STACK: trying approach '
                    f'yaw offset {yaw_offset:+.0f} deg, '
                    f'clearance {clearance * 100.0:.1f} cm '
                    'with prioritized J2/J3 branches'
                )
                try:
                    if self.motion_backend == 'moveit':
                        exact_approach = copy.deepcopy(approach)
                        exact_approach.pose.orientation = copy.deepcopy(
                            descent_orientation or approach.pose.orientation
                        )
                        solution, exact_approach = (
                            self._preflight_stack_approach_and_descent(
                                approach,
                                exact_approach,
                                release,
                                'STACK approach',
                            )
                        )
                        self._execute_planned_joint_goal(solution)
                        approach = exact_approach
                    elif self.prefer_z_last_motion:
                        self.move_to_pose_z_last(approach)
                    else:
                        self.move_to_pose(approach)
                    release.pose.orientation = copy.deepcopy(
                        approach.pose.orientation
                    )
                    self.stack_approach_pose_publisher.publish(approach)
                    self.publish_status(
                        'STACK: selected approach '
                        f'yaw offset {yaw_offset:+.0f} deg, '
                        f'clearance {clearance * 100.0:.1f} cm'
                    )
                    return approach, yaw_offset
                except RuntimeError as exc:
                    error = str(exc)
                    retryable = (
                        'code=99999' in error
                        or 'code=-4' in error
                        or 'IK branch passed plan-only validation' in error
                        or 'passed descent preflight' in error
                    )
                    if not retryable:
                        raise
                    if 'code=-4' in error:
                        self.publish_status(
                            'STACK: controller rejected this yaw branch; '
                            'trying the next yaw candidate'
                        )
                    failures.append(
                        f'yaw {yaw_offset:+.0f}deg/{clearance:.3f}m: {error}'
                    )
        if self.motion_backend == 'moveit':
            # A destination may be reachable while a purely vertical release
            # crosses an IK discontinuity.  Keep the exact release XYZ and
            # test a short, shallow final approach from four lateral sides.
            # The full diagonal is collision/joint-jump checked before the
            # loaded arm moves to the candidate.
            for yaw_offset in yaw_offsets:
                if yaw_offset in excluded_yaw_offsets:
                    continue
                candidate_rotation = quaternion_from_rpy_degrees(
                    base_rpy[0],
                    base_rpy[1],
                    wrap_degrees(base_rpy[2] + yaw_offset),
                )
                for dx, dy in (
                    (0.010, 0.0),
                    (-0.010, 0.0),
                    (0.0, 0.010),
                    (0.0, -0.010),
                ):
                    approach = copy.deepcopy(release)
                    approach.pose.position.x += dx
                    approach.pose.position.y += dy
                    approach.pose.position.z += 0.025
                    (
                        approach.pose.orientation.x,
                        approach.pose.orientation.y,
                        approach.pose.orientation.z,
                        approach.pose.orientation.w,
                    ) = map(float, candidate_rotation)
                    translation = np.array([
                        approach.pose.position.x,
                        approach.pose.position.y,
                        approach.pose.position.z,
                    ])
                    if not self.in_workspace(translation):
                        failures.append(
                            f'oblique yaw {yaw_offset:+.0f}deg '
                            f'dx={dx * 1000.0:+.0f}mm '
                            f'dy={dy * 1000.0:+.0f}mm: outside workspace'
                        )
                        continue
                    self.publish_status(
                        'STACK: testing short oblique release approach '
                        f'yaw={yaw_offset:+.0f}deg, '
                        f'dx={dx * 1000.0:+.0f}mm, '
                        f'dy={dy * 1000.0:+.0f}mm, dz=+25mm'
                    )
                    exact_approach = copy.deepcopy(approach)
                    exact_approach.pose.orientation = copy.deepcopy(
                        descent_orientation or approach.pose.orientation
                    )
                    try:
                        solution, exact_approach = (
                            self._preflight_stack_approach_and_descent(
                                approach,
                                exact_approach,
                                release,
                                'STACK short oblique approach',
                            )
                        )
                        self._execute_planned_joint_goal(solution)
                        release.pose.orientation = copy.deepcopy(
                            exact_approach.pose.orientation
                        )
                        self.stack_approach_pose_publisher.publish(
                            exact_approach
                        )
                        self.publish_status(
                            'STACK: selected preflight-safe short oblique '
                            f'approach yaw={yaw_offset:+.0f}deg, '
                            f'dx={dx * 1000.0:+.0f}mm, '
                            f'dy={dy * 1000.0:+.0f}mm'
                        )
                        return exact_approach, yaw_offset
                    except RuntimeError as exc:
                        if self.stop_event.is_set():
                            raise
                        failures.append(
                            f'oblique yaw {yaw_offset:+.0f}deg '
                            f'dx={dx * 1000.0:+.0f}mm '
                            f'dy={dy * 1000.0:+.0f}mm: {exc}'
                        )
        raise RuntimeError(
            'No reachable vertical or short oblique stack approach: '
            + '; '.join(failures)
        )

    def _request_pose_ik(self, target, seed):
        """Return one collision-free IK solution seeded by six joints."""
        return self._request_seeded_pose_ik(target, seed, self.ik_timeout)

    def _request_motion_pose_ik(self, target, seed):
        """Return collision-free IK using the full motion timeout."""
        return self._request_seeded_pose_ik(
            target, seed, self.motion_ik_timeout
        )

    def _request_seeded_pose_ik(self, target, seed, timeout):
        """Return one collision-free IK solution from an explicit seed."""
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self.moveit_group
        ik_request.ik_link_name = self.moveit_ee_link
        ik_request.avoid_collisions = True
        ik_request.timeout = duration_message(timeout)
        ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        ik_request.robot_state.joint_state.position = [
            float(value) for value in seed
        ]
        ik_request.pose_stamped = copy.deepcopy(target)
        ik_request.pose_stamped.header.frame_id = self.base_frame
        ik_request.pose_stamped.header.stamp = Time().to_msg()
        future = self.position_ik_client.call_async(request)
        self._wait_future(future, 5.0)
        response = future.result()
        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            return None
        solution_map = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if any(name not in solution_map for name in JOINT_NAMES):
            return None
        return np.array(
            [solution_map[name] for name in JOINT_NAMES],
            dtype=np.float64,
        )

    def _preflight_vertical_descent(self, start_joints, release):
        """Validate descent across expected physical joint tracking error."""
        test_states = [('nominal', np.array(start_joints, dtype=np.float64))]
        margin = self.stack_preflight_joint_margin
        if margin > 0.0:
            # Vertical placement reachability is dominated by the shoulder
            # and elbow tracking error.  Other joints retain the controller's
            # normal goal tolerance and must not reject an otherwise valid
            # descent merely because the hardware cannot settle to sub-degree
            # accuracy on every axis.
            for joint_index in (1, 2):
                for sign in (-1.0, 1.0):
                    perturbed = np.array(start_joints, dtype=np.float64)
                    perturbed[joint_index] += sign * margin
                    lower, upper = JOINT_LIMITS_DEG[joint_index]
                    perturbed[joint_index] = min(
                        max(perturbed[joint_index], math.radians(lower)),
                        math.radians(upper),
                    )
                    test_states.append((
                        f'J{joint_index + 1}{sign:+.0f}', perturbed
                    ))

        minimum_fraction = 1.0
        minimum_label = 'nominal'
        for state_label, state in test_states:
            request = GetCartesianPath.Request()
            request.header = copy.deepcopy(release.header)
            request.header.frame_id = self.base_frame
            request.header.stamp = Time().to_msg()
            request.start_state.is_diff = True
            request.start_state.joint_state.name = list(JOINT_NAMES)
            request.start_state.joint_state.position = [
                float(value) for value in state
            ]
            request.group_name = self.moveit_group
            request.link_name = self.moveit_ee_link
            request.waypoints = [copy.deepcopy(release.pose)]
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
            if (
                response is None
                or response.error_code.val != MoveItErrorCodes.SUCCESS
            ):
                return False, f'{state_label}=planner error'
            if response.fraction < minimum_fraction:
                minimum_fraction = response.fraction
                minimum_label = state_label
            if response.fraction < self.cartesian_min_fraction:
                return False, (
                    f'{state_label} margin fraction={response.fraction:.3f}'
                )
            if not response.solution.joint_trajectory.points:
                return False, f'{state_label}=empty Cartesian path'
            j6_travel = trajectory_joint_travel_degrees(
                state[5], response.solution, JOINT_NAMES[5]
            )
            if j6_travel > self.max_j6_trajectory_travel:
                return False, (
                    f'{state_label} J6 path travel={j6_travel:.1f}deg '
                    f'exceeds {self.max_j6_trajectory_travel:.1f}deg'
                )
        return True, (
            f'robust fraction={minimum_fraction:.3f} at {minimum_label}'
        )

    def _preflight_stack_approach_and_descent(
        self,
        branch_approach,
        exact_approach,
        release,
        context,
    ):
        """Select a branch and 180-degree-equivalent release orientation."""
        if not self.position_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/compute_ik is unavailable')
        if not self.cartesian_path_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(
                'MoveIt /arm2/compute_cartesian_path service is unavailable'
            )
        with self.joint_state_lock:
            current_joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            current_joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')

        # For placement, prefer the elbow-out J3-positive families so the
        # third link stays away from the gripper/container during descent.
        # Elbow-in J3-negative solutions remain available as fallbacks.
        branch_patterns = (
            ('place preferred elbow-out (J2-/J3+)', -1.0, 1.0),
            ('place elbow-out (J2+/J3+)', 1.0, 1.0),
            ('place fallback elbow-in (J2-/J3-)', -1.0, -1.0),
            ('place fallback elbow-in (J2+/J3-)', 1.0, -1.0),
        )
        exact_rpy = quaternion_to_rpy_degrees([
            exact_approach.pose.orientation.x,
            exact_approach.pose.orientation.y,
            exact_approach.pose.orientation.z,
            exact_approach.pose.orientation.w,
        ])
        equivalent_approaches = []
        for symmetry_offset in (0.0, 180.0):
            candidate = copy.deepcopy(exact_approach)
            rotation = quaternion_from_rpy_degrees(
                exact_rpy[0],
                exact_rpy[1],
                wrap_degrees(exact_rpy[2] + symmetry_offset),
            )
            (
                candidate.pose.orientation.x,
                candidate.pose.orientation.y,
                candidate.pose.orientation.z,
                candidate.pose.orientation.w,
            ) = map(float, rotation)
            equivalent_approaches.append((symmetry_offset, candidate))
        failures = []
        for label, j2_sign, j3_sign in branch_patterns:
            for magnitude in (30.0, 60.0, 90.0):
                seed = current_joints.copy()
                for joint_index, sign in ((1, j2_sign), (2, j3_sign)):
                    lower, upper = JOINT_LIMITS_DEG[joint_index]
                    seed[joint_index] = math.radians(min(
                        max(sign * magnitude, lower + 1.0), upper - 1.0
                    ))
                branch_solution = self._request_pose_ik(
                    branch_approach, seed
                )
                if branch_solution is None:
                    continue
                j2_deg = math.degrees(float(branch_solution[1]))
                j3_deg = math.degrees(float(branch_solution[2]))
                if j2_deg * j2_sign < 5.0 or j3_deg * j3_sign < 5.0:
                    continue
                exact_solutions = []
                for symmetry_offset, candidate in equivalent_approaches:
                    exact_solution = self._request_pose_ik(
                        candidate, branch_solution
                    )
                    if exact_solution is None:
                        failures.append(
                            f'{label}: yaw{symmetry_offset:+.0f} IK failed'
                        )
                        continue
                    j6_travel = abs(math.degrees(float(
                        exact_solution[5] - current_joints[5]
                    )))
                    exact_solutions.append((
                        j6_travel,
                        symmetry_offset,
                        candidate,
                        exact_solution,
                    ))
                exact_solutions.sort(key=lambda item: item[0])
                for (
                    j6_travel,
                    symmetry_offset,
                    candidate,
                    exact_solution,
                ) in exact_solutions:
                    if j6_travel > 170.0:
                        failures.append(
                            f'{label}: yaw{symmetry_offset:+.0f} direct '
                            f'J6 travel {j6_travel:.1f}deg'
                        )
                        continue
                    try:
                        self._execute_planned_joint_goal(
                            exact_solution, plan_only=True
                        )
                    except RuntimeError as exc:
                        failures.append(
                            f'{label}: yaw{symmetry_offset:+.0f} {exc}'
                        )
                        continue
                    candidate_release = copy.deepcopy(release)
                    candidate_release.pose.orientation = copy.deepcopy(
                        candidate.pose.orientation
                    )
                    descent_ok, descent_detail = (
                        self._preflight_vertical_descent(
                            exact_solution, candidate_release
                        )
                    )
                    if not descent_ok:
                        failures.append(
                            f'{label}: yaw{symmetry_offset:+.0f} descent '
                            f'{descent_detail}'
                        )
                        continue
                    self.publish_status(
                        f'{context}: preflight passed {label}, '
                        f'equivalent yaw {symmetry_offset:+.0f}deg: '
                        f'J2={math.degrees(float(exact_solution[1])):.1f}deg, '
                        f'J3={math.degrees(float(exact_solution[2])):.1f}deg, '
                        f'J6 travel={j6_travel:.1f}deg, {descent_detail}'
                    )
                    return exact_solution, candidate
            failures.append(f'{label}: no complete approach/descent')
        raise RuntimeError(
            f'no {context} IK branch passed descent preflight: '
            + '; '.join(failures)
        )

    def _move_to_pose_with_j2_j3_branch_priority(
        self,
        target,
        context,
        descent_preflight_target=None,
    ):
        """Plan a target pose using explicit J2/J3 branch priority."""
        if not self.position_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/compute_ik is unavailable')
        with self.joint_state_lock:
            current_joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            current_joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')

        # Keep J3 negative first for picking: on this arm that folds the third
        # link away from the gripper/container.  Exhaust both J2 families in
        # that separated geometry before considering a J3-positive fallback.
        # Multiple magnitudes help the numerical IK solver reach the requested
        # family without weakening the sign check on its returned solution.
        branch_patterns = (
            ('preferred separated (J2+/J3-)', 1.0, -1.0),
            ('separated J2 fallback (J2-/J3-)', -1.0, -1.0),
            ('close-side fallback (J2+/J3+)', 1.0, 1.0),
            ('last close-side fallback (J2-/J3+)', -1.0, 1.0),
        )
        seed_magnitudes = (30.0, 60.0, 90.0)
        failures = []
        for label, j2_sign, j3_sign in branch_patterns:
            for magnitude in seed_magnitudes:
                seed = current_joints.copy()
                for joint_index, sign in ((1, j2_sign), (2, j3_sign)):
                    lower, upper = JOINT_LIMITS_DEG[joint_index]
                    seed_deg = min(
                        max(sign * magnitude, lower + 1.0),
                        upper - 1.0,
                    )
                    seed[joint_index] = math.radians(seed_deg)

                # When validating a pick descent, solve the final grasp first
                # and use that exact branch as the seed for the pregrasp.  A
                # pregrasp-first solve can look reachable yet cross an IK
                # discontinuity immediately after descent begins.
                if descent_preflight_target is not None:
                    grasp_solution = self._request_motion_pose_ik(
                        descent_preflight_target, seed
                    )
                    if grasp_solution is None:
                        continue
                    grasp_j2_deg = math.degrees(float(grasp_solution[1]))
                    grasp_j3_deg = math.degrees(float(grasp_solution[2]))
                    if (
                        grasp_j2_deg * j2_sign < 5.0
                        or grasp_j3_deg * j3_sign < 5.0
                    ):
                        continue
                    solution = self._request_motion_pose_ik(
                        target, grasp_solution
                    )
                else:
                    grasp_solution = None
                    solution = self._request_motion_pose_ik(target, seed)
                if solution is None:
                    continue
                solution_j2_deg = math.degrees(float(solution[1]))
                solution_j3_deg = math.degrees(float(solution[2]))
                if (
                    solution_j2_deg * j2_sign < 5.0
                    or solution_j3_deg * j3_sign < 5.0
                ):
                    continue
                if (
                    abs(solution_j2_deg - self.bad_pick_branch_j2)
                    <= self.bad_pick_branch_tolerance
                    and abs(solution_j3_deg - self.bad_pick_branch_j3)
                    <= self.bad_pick_branch_tolerance
                ):
                    failures.append(
                        f'{label}: rejected known close J3/gripper '
                        f'pregrasp J2={solution_j2_deg:.1f}deg, '
                        f'J3={solution_j3_deg:.1f}deg'
                    )
                    continue
                try:
                    self._execute_planned_joint_goal(
                        solution,
                        plan_only=True,
                    )
                except RuntimeError as exc:
                    failures.append(f'{label}: {exc}')
                    continue
                if descent_preflight_target is not None:
                    branch_drift = max(
                        abs(solution_j2_deg - grasp_j2_deg),
                        abs(solution_j3_deg - grasp_j3_deg),
                    )
                    descent_ok, descent_detail = (
                        self._preflight_vertical_descent(
                            solution, descent_preflight_target
                        )
                    )
                    if not descent_ok:
                        failures.append(
                            f'{label}: descent {descent_detail}'
                        )
                        continue
                    descent_detail = (
                        f'{descent_detail}, paired branch drift='
                        f'{branch_drift:.1f}deg'
                    )
                else:
                    descent_detail = None
                self.publish_status(
                    f'{context}: selected planned J2/J3 branch '
                    f'{label}: J2={solution_j2_deg:.1f}deg, '
                    f'J3={solution_j3_deg:.1f}deg'
                    + (
                        f', descent {descent_detail}'
                        if descent_detail is not None else ''
                    )
                )
                self._execute_planned_joint_goal(solution)
                return
            failures.append(f'{label}: no valid planned IK solution')
        raise RuntimeError(
            f'no {context} IK branch passed plan-only validation: '
            + '; '.join(failures)
        )

    def move_to_reachable_stack_release(
        self,
        release,
        orientation,
        label,
        alignment_pose=None,
    ):
        """Use fallback yaw only for approach, then place at exact yaw."""
        desired_release_orientation = copy.deepcopy(
            release.pose.orientation
        )
        excluded_yaw_offsets = set()
        descent_failures = []
        while len(excluded_yaw_offsets) < len(
            self.stack_yaw_fallback_offsets
        ):
            approach, yaw_offset = self.move_to_reachable_stack_approach(
                release,
                orientation,
                excluded_yaw_offsets=excluded_yaw_offsets,
                alignment_pose=alignment_pose,
                descent_orientation=desired_release_orientation,
            )
            exact_approach = copy.deepcopy(approach)
            exact_approach.pose.orientation = copy.deepcopy(
                release.pose.orientation
            )
            self.publish_status(
                f'{label}: approach yaw {yaw_offset:+.0f} deg selected; '
                'aligning exact destination yaw before descent'
            )
            try:
                self.move_to_pose(exact_approach)
            except RuntimeError as exc:
                descent_failures.append(
                    f'approach yaw {yaw_offset:+.0f}deg exact-yaw '
                    f'alignment: {exc}'
                )
                excluded_yaw_offsets.add(yaw_offset)
                self.publish_status(
                    f'{label}: exact destination yaw is not reachable from '
                    f'approach yaw {yaw_offset:+.0f} deg; trying next branch'
                )
                continue
            # Keep the 180-degree-equivalent orientation selected by
            # preflight. Restoring desired_release_orientation here makes the
            # real descent differ from the validated path and can force J3/J6
            # onto the opposite IK branch.
            release.pose.orientation = copy.deepcopy(
                exact_approach.pose.orientation
            )
            approach = exact_approach
            self.publish_status(
                f'{label}: testing vertical descent at exact destination yaw'
            )
            try:
                self.move_segmented_cartesian_with_pose_finish(
                    release,
                    label,
                    prefer_j2_branch_fallback=True,
                )
                return approach
            except CartesianPlanningError as exc:
                if (
                    exc.executed_segments == 0
                    and 'ik_only=1.000' in str(exc)
                    and 'no_joint_jump=' in str(exc)
                    and self.try_opposite_ik_branch(
                        release, f'{label} vertical descent'
                    )
                ):
                    self.publish_status(
                        f'{label}: retrying vertical descent on opposite '
                        'J2 branch'
                    )
                    continue
                descent_failures.append(
                    f'yaw {yaw_offset:+.0f}deg: {exc}'
                )
                excluded_yaw_offsets.add(yaw_offset)
                self.publish_status(
                    f'{label}: vertical descent failed with '
                    f'yaw offset {yaw_offset:+.0f} deg; '
                    'retreating and trying the next yaw'
                )
                if exc.executed_segments > 0:
                    self.move_segmented_cartesian_with_pose_finish(
                        approach, f'{label} retry retreat'
                    )
        raise RuntimeError(
            f'No yaw supports the complete vertical descent for {label}: '
            + '; '.join(descent_failures)
        )

    def move_segmented_cartesian_with_pose_finish(
        self,
        target,
        label,
        prefer_j2_branch_fallback=False,
    ):
        """Move in safe Cartesian segments, then finish a short remainder."""
        try:
            self.move_cartesian_to_pose(
                target,
                allow_segmented=True,
                prefer_j2_branch_fallback=prefer_j2_branch_fallback,
            )
        except CartesianPlanningError as exc:
            remaining_distance = self._cartesian_request_distance(target)
            can_finish_with_pose_goal = (
                exc.executed_segments > 0
                and remaining_distance is not None
                and remaining_distance
                <= self.stack_pose_goal_finish_max_distance
            )
            if not can_finish_with_pose_goal:
                raise
            self.publish_status(
                f'{label}: finishing short Cartesian remainder with pose goal: '
                f'remaining={remaining_distance * 1000.0:.1f}mm'
            )
            self.move_to_pose(target)

    def verify_release_pose(self, target, label):
        """Refuse to release unless the measured TCP reached the target."""
        try:
            current = self.buffer.lookup_transform(
                target.header.frame_id,
                self.moveit_ee_link,
                Time(),
            )
        except TransformException as exc:
            raise RuntimeError(
                f'{label}: current TCP transform unavailable: {exc}'
            ) from exc
        dx = float(current.transform.translation.x) - float(
            target.pose.position.x
        )
        dy = float(current.transform.translation.y) - float(
            target.pose.position.y
        )
        dz = abs(
            float(current.transform.translation.z)
            - float(target.pose.position.z)
        )
        xy_error = math.hypot(dx, dy)
        actual_yaw = quaternion_to_rpy_degrees([
            current.transform.rotation.x,
            current.transform.rotation.y,
            current.transform.rotation.z,
            current.transform.rotation.w,
        ])[2]
        target_yaw = quaternion_to_rpy_degrees([
            target.pose.orientation.x,
            target.pose.orientation.y,
            target.pose.orientation.z,
            target.pose.orientation.w,
        ])[2]
        yaw_error = abs(wrap_degrees(actual_yaw - target_yaw))
        self.publish_status(
            f'{label}: release verification xy={xy_error * 1000.0:.1f}mm, '
            f'z={dz * 1000.0:.1f}mm, yaw={yaw_error:.1f}deg'
        )
        if (
            xy_error > self.release_verify_xy_tolerance
            or dz > self.release_verify_z_tolerance
            or yaw_error > self.release_verify_yaw_tolerance
        ):
            raise RuntimeError(
                f'{label}: release pose outside tolerance: '
                f'xy={xy_error * 1000.0:.1f}mm '
                f'(max {self.release_verify_xy_tolerance * 1000.0:.1f}mm), '
                f'z={dz * 1000.0:.1f}mm '
                f'(max {self.release_verify_z_tolerance * 1000.0:.1f}mm), '
                f'yaw={yaw_error:.1f}deg '
                f'(max {self.release_verify_yaw_tolerance:.1f}deg)'
            )

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

    def move_to_pose(
        self,
        pose,
        keep_current_orientation=False,
        require_preferred_pick_branch=False,
        descent_preflight_target=None,
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
            if require_preferred_pick_branch:
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
                    target.pose.orientation = copy.deepcopy(
                        current.transform.rotation
                    )
                self._move_to_pose_with_j2_j3_branch_priority(
                    target,
                    'PICK pregrasp',
                    descent_preflight_target=descent_preflight_target,
                )
                return
            self._move_to_pose_moveit(
                pose,
                keep_current_orientation,
                False,
            )
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

    def move_to_pose_z_last(
        self,
        target,
        keep_current_orientation=False,
        require_preferred_pick_branch=False,
        descent_preflight_target=None,
    ):
        """Delay only descent; upward targets use the normal direct move."""
        try:
            current = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_z = float(current.transform.translation.z)
        target_z = float(target.pose.position.z)
        if target_z >= current_z - 0.003:
            self.move_to_pose(
                target,
                keep_current_orientation,
                require_preferred_pick_branch,
                descent_preflight_target,
            )
            return
        intermediate = copy.deepcopy(target)
        intermediate.pose.position.z = current_z
        if keep_current_orientation:
            intermediate.pose.orientation = copy.deepcopy(
                current.transform.rotation
            )
        translation = np.array([
            intermediate.pose.position.x,
            intermediate.pose.position.y,
            intermediate.pose.position.z,
        ])
        if not self.in_workspace(translation):
            self.get_logger().warning(
                'Z-last intermediate is outside workspace; using direct move'
            )
            self.move_to_pose(
                target,
                keep_current_orientation,
                require_preferred_pick_branch,
                descent_preflight_target,
            )
            return
        self.publish_status(
            'MOTION: moving XY/orientation first at current Z '
            '(favoring base rotation)'
        )
        try:
            self.move_to_pose(
                intermediate,
                keep_current_orientation=keep_current_orientation,
                require_preferred_pick_branch=(
                    require_preferred_pick_branch
                ),
                descent_preflight_target=descent_preflight_target,
            )
        except RuntimeError as exc:
            if self.stop_event.is_set():
                raise
            self.get_logger().warning(
                f'Z-last intermediate failed; using direct move: {exc}'
            )
            self.move_to_pose(
                target,
                keep_current_orientation,
                require_preferred_pick_branch,
                descent_preflight_target,
            )
            return
        descent = copy.deepcopy(target)
        if keep_current_orientation:
            descent.pose.orientation = copy.deepcopy(
                intermediate.pose.orientation
            )
        self.publish_status('MOTION: descending Z last at target XY')
        self.move_segmented_cartesian_with_pose_finish(
            descent, 'MOTION Z-last descent'
        )

    def _move_to_pose_moveit(
        self,
        pose,
        keep_current_orientation,
        require_preferred_pick_branch=False,
    ):
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
                self._execute_moveit_pose_goal(
                    target,
                    require_preferred_pick_branch,
                )
                return
            except RuntimeError as exc:
                if self.stop_event.is_set():
                    raise
                self.get_logger().warning(
                    'MoveIt failed with current TCP orientation; retrying '
                    f'the taught grasp orientation: {exc}'
                )
        self._execute_moveit_pose_goal(
            pose,
            require_preferred_pick_branch,
        )

    def _execute_moveit_pose_goal(
        self,
        target,
        require_preferred_pick_branch=False,
    ):
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/move_action server is unavailable')

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
        request.goal_constraints = [self._moveit_pose_constraints(
            target,
            require_preferred_pick_branch,
        )]

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

    def _joint_state_callback(self, message):
        """Cache a complete, name-ordered arm joint state."""
        positions = dict(zip(message.name, message.position))
        if any(name not in positions for name in JOINT_NAMES):
            return
        with self.joint_state_lock:
            self.latest_joint_positions = np.array(
                [positions[name] for name in JOINT_NAMES],
                dtype=np.float64,
            )
            self.latest_joint_state_time = time.monotonic()

    def move_joint1_toward_destination(
        self,
        source_pose,
        destination_pose,
        destination_name=None,
    ):
        """Rotate J1 first so the lifted arm faces its destination zone."""
        if self.motion_backend != 'moveit':
            raise RuntimeError('J1-priority transfer requires MoveIt')
        source_bearing = math.atan2(
            source_pose.pose.position.y,
            source_pose.pose.position.x,
        )
        destination_bearing = math.atan2(
            destination_pose.pose.position.y,
            destination_pose.pose.position.x,
        )
        bearing_delta = math.atan2(
            math.sin(destination_bearing - source_bearing),
            math.cos(destination_bearing - source_bearing),
        )
        with self.joint_state_lock:
            positions = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            positions is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')

        destination_zone = None
        if destination_name:
            parts = str(destination_name).split('-')
            if len(parts) >= 2 and parts[0] == 'A':
                destination_zone = f'A-{parts[1]}'
        if destination_zone in self.destination_zone_j1:
            target_j1 = self.destination_zone_j1[destination_zone]
            heading_description = (
                f'zone={destination_zone}, '
                f'target={math.degrees(target_j1):.1f}deg'
            )
        else:
            target_j1 = float(positions[0] + bearing_delta)
            heading_description = (
                f'bearing delta={math.degrees(bearing_delta):.1f}deg'
            )
        lower = math.radians(JOINT_LIMITS_DEG[0][0])
        upper = math.radians(JOINT_LIMITS_DEG[0][1])
        candidates = [
            target_j1 - 2.0 * math.pi,
            target_j1,
            target_j1 + 2.0 * math.pi,
        ]
        valid = [value for value in candidates if lower <= value <= upper]
        if not valid:
            raise RuntimeError(
                'J1 destination bearing is outside joint limits: '
                f'{math.degrees(target_j1):.1f}deg'
            )
        positions[0] = min(
            valid,
            key=lambda value: abs(value - positions[0]),
        )
        self.publish_status(
            'TRANSFER: facing destination before place IK while holding '
            f'J2-J6: {heading_description}'
        )
        self._execute_moveit_joint_goal(positions)

    def move_joint1_toward_source(self, source_pose, scan_zone=None):
        """Face the source zone while preserving the safe scan posture."""
        if self.motion_backend != 'moveit':
            raise RuntimeError('J1-priority pick requires MoveIt')
        if scan_zone in self.destination_zone_j1:
            target_j1 = self.destination_zone_j1[scan_zone]
            target_description = f'zone={scan_zone}'
        else:
            target_j1 = math.atan2(
                source_pose.pose.position.y,
                source_pose.pose.position.x,
            )
            target_description = 'marker bearing'
        with self.joint_state_lock:
            positions = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            positions is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')

        lower = math.radians(JOINT_LIMITS_DEG[0][0])
        upper = math.radians(JOINT_LIMITS_DEG[0][1])
        candidates = (
            target_j1 - 2.0 * math.pi,
            target_j1,
            target_j1 + 2.0 * math.pi,
        )
        valid = [value for value in candidates if lower <= value <= upper]
        if not valid:
            raise RuntimeError(
                'J1 source bearing is outside joint limits: '
                f'{math.degrees(target_j1):.1f}deg'
            )
        current_j1 = float(positions[0])
        positions[0] = min(valid, key=lambda value: abs(value - current_j1))
        self.publish_status(
            'TRANSFER: holding source zone before pick IK with J2-J6 fixed: '
            f'{target_description}, '
            f'current={math.degrees(current_j1):.1f}deg, '
            f'target={math.degrees(float(positions[0])):.1f}deg'
        )
        self._execute_moveit_joint_goal(positions)

    def _execute_moveit_joint_goal(self, positions):
        """Execute a trajectory whose J2-J6 positions remain constant."""
        if not self.follow_joint_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError('arm trajectory controller is unavailable')
        with self.joint_state_lock:
            start = self.latest_joint_positions.copy()
        delta = abs(float(positions[0] - start[0]))
        duration_sec = max(1.0, math.degrees(delta) / 20.0)
        start_point = JointTrajectoryPoint()
        start_point.positions = [float(value) for value in start]
        start_point.time_from_start = Duration(sec=0, nanosec=100000000)
        target_point = JointTrajectoryPoint()
        target_point.positions = [float(value) for value in positions]
        whole_seconds = int(duration_sec)
        target_point.time_from_start = Duration(
            sec=whole_seconds,
            nanosec=int((duration_sec - whole_seconds) * 1e9),
        )
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = list(JOINT_NAMES)
        goal.trajectory.points = [start_point, target_point]

        goal_future = self.follow_joint_trajectory_client.send_goal_async(goal)
        self._wait_future(goal_future, 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            raise RuntimeError('controller rejected the J1-only trajectory')
        with self.moveit_goal_lock:
            self.current_moveit_goal = goal_handle
        try:
            result_future = goal_handle.get_result_async()
            self._wait_future(
                result_future,
                duration_sec + self.motion_timeout + 10.0,
                goal_handle,
            )
            wrapped_result = result_future.result()
            if wrapped_result is None:
                raise RuntimeError('controller returned no J1 result')
            result = wrapped_result.result
            if result.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(
                    'J1-only trajectory failed: '
                    f'code={result.error_code}, '
                    f'message={result.error_string}'
                )
            if self.stop_event.wait(self.moveit_state_settle):
                raise RuntimeError('pick stopped')
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _try_opposite_j2_j3_branch(self, context):
        """Keep TCP pose while reversing J3 and moving J2 across/near zero."""
        try:
            current_pose = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_tcp_z = float(current_pose.transform.translation.z)
        minimum_height = max(
            self.j2_fallback_min_tcp_height,
            self.j3_fallback_min_tcp_height,
        )
        if current_tcp_z < minimum_height:
            self.publish_status(
                f'{context}: opposite J2+J3 fallback blocked; TCP Z '
                f'{current_tcp_z * 1000.0:.1f}mm is below '
                f'{minimum_height * 1000.0:.1f}mm'
            )
            return False
        with self.joint_state_lock:
            current_joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            current_joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')
        if not self.position_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/compute_ik is unavailable')

        current_j2_deg = math.degrees(float(current_joints[1]))
        current_j3_deg = math.degrees(float(current_joints[2]))
        seed = current_joints.copy()
        seed[1] = math.radians(
            -max(30.0, abs(current_j2_deg))
            if current_j2_deg >= 0.0
            else max(30.0, abs(current_j2_deg))
        )
        seed[2] = math.radians(
            -max(30.0, abs(current_j3_deg))
            if current_j3_deg >= 0.0
            else max(30.0, abs(current_j3_deg))
        )
        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self.moveit_group
        ik_request.ik_link_name = self.moveit_ee_link
        ik_request.avoid_collisions = True
        ik_request.timeout = duration_message(self.motion_ik_timeout)
        ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        ik_request.robot_state.joint_state.position = [
            float(value) for value in seed
        ]
        ik_request.pose_stamped.header.frame_id = self.base_frame
        ik_request.pose_stamped.header.stamp = Time().to_msg()
        ik_request.pose_stamped.pose.position.x = float(
            current_pose.transform.translation.x
        )
        ik_request.pose_stamped.pose.position.y = float(
            current_pose.transform.translation.y
        )
        ik_request.pose_stamped.pose.position.z = float(
            current_pose.transform.translation.z
        )
        ik_request.pose_stamped.pose.orientation = copy.deepcopy(
            current_pose.transform.rotation
        )
        future = self.position_ik_client.call_async(request)
        self._wait_future(future, 5.0)
        response = future.result()
        self.j2_fallback_used = True
        self.j3_fallback_used = True
        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            self.publish_status(
                f'{context}: no opposite J2+J3 IK solution found'
            )
            return False
        solution_map = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if any(name not in solution_map for name in JOINT_NAMES):
            raise RuntimeError(
                'opposite J2+J3 IK returned incomplete joints'
            )
        solution = np.array(
            [solution_map[name] for name in JOINT_NAMES],
            dtype=np.float64,
        )
        solution_j2_deg = math.degrees(float(solution[1]))
        solution_j3_deg = math.degrees(float(solution[2]))
        j3_opposite = (
            abs(current_j3_deg) >= 5.0
            and abs(solution_j3_deg) >= 5.0
            and current_j3_deg * solution_j3_deg < 0.0
        )
        j2_opposite_or_neutral = (
            abs(current_j2_deg) >= 5.0
            and (
                abs(solution_j2_deg) < 5.0
                or current_j2_deg * solution_j2_deg < 0.0
            )
        )
        if not (j3_opposite and j2_opposite_or_neutral):
            self.publish_status(
                f'{context}: IK did not reverse J3 with opposite/neutral J2: '
                f'J2 {current_j2_deg:.1f}->{solution_j2_deg:.1f}deg, '
                f'J3 {current_j3_deg:.1f}->{solution_j3_deg:.1f}deg'
            )
            return False
        j2_result = (
            'neutral' if abs(solution_j2_deg) < 5.0 else 'opposite'
        )
        self.publish_status(
            f'{context}: switching J3 with {j2_result} J2 at safe height: '
            f'J2 {current_j2_deg:.1f}->{solution_j2_deg:.1f}deg, '
            f'J3 {current_j3_deg:.1f}->{solution_j3_deg:.1f}deg'
        )
        self._execute_planned_joint_goal(solution)
        return True

    def try_opposite_ik_branch(self, release, context):
        """Try unused opposite-sign J3, then J2 IK branches."""
        if self.motion_backend != 'moveit':
            return False
        if not self.j3_fallback_used and self._try_opposite_joint_branch(
            2,
            'J3',
            'j3_fallback_used',
            self.j3_fallback_min_tcp_height,
            context,
        ):
            return True
        if not self.j2_fallback_used and self._try_opposite_joint_branch(
            1,
            'J2',
            'j2_fallback_used',
            self.j2_fallback_min_tcp_height,
            context,
        ):
            return True
        return self._try_multi_seed_ik_branch(context)

    def _try_multi_seed_ik_branch(self, context):
        """Try additional J3-priority seeds without persistent storage."""
        seed_patterns = (
            (2, 20.0), (2, -20.0),
            (2, 40.0), (2, -40.0),
            (2, 60.0), (2, -60.0),
            (1, 20.0), (1, -20.0),
            (1, 40.0), (1, -40.0),
        )
        try:
            current_pose = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_tcp_z = float(current_pose.transform.translation.z)
        minimum_height = max(
            self.j2_fallback_min_tcp_height,
            self.j3_fallback_min_tcp_height,
        )
        if current_tcp_z < minimum_height:
            self.publish_status(
                f'{context}: multi-seed IK fallback blocked; TCP Z '
                f'{current_tcp_z * 1000.0:.1f}mm is below '
                f'{minimum_height * 1000.0:.1f}mm'
            )
            return False
        with self.joint_state_lock:
            current_joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            current_joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')
        if not self.position_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/compute_ik is unavailable')

        while self.ik_seed_fallback_index < len(seed_patterns):
            joint_index, offset_deg = seed_patterns[
                self.ik_seed_fallback_index
            ]
            self.ik_seed_fallback_index += 1
            joint_label = f'J{joint_index + 1}'
            seed = current_joints.copy()
            seed_deg = math.degrees(float(seed[joint_index])) + offset_deg
            lower, upper = JOINT_LIMITS_DEG[joint_index]
            seed_deg = min(max(seed_deg, lower + 1.0), upper - 1.0)
            seed[joint_index] = math.radians(seed_deg)

            request = GetPositionIK.Request()
            ik_request = request.ik_request
            ik_request.group_name = self.moveit_group
            ik_request.ik_link_name = self.moveit_ee_link
            ik_request.avoid_collisions = True
            ik_request.timeout = duration_message(self.motion_ik_timeout)
            ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
            ik_request.robot_state.joint_state.position = [
                float(value) for value in seed
            ]
            ik_request.pose_stamped.header.frame_id = self.base_frame
            ik_request.pose_stamped.header.stamp = Time().to_msg()
            ik_request.pose_stamped.pose.position.x = float(
                current_pose.transform.translation.x
            )
            ik_request.pose_stamped.pose.position.y = float(
                current_pose.transform.translation.y
            )
            ik_request.pose_stamped.pose.position.z = float(
                current_pose.transform.translation.z
            )
            ik_request.pose_stamped.pose.orientation = copy.deepcopy(
                current_pose.transform.rotation
            )
            future = self.position_ik_client.call_async(request)
            self._wait_future(future, 5.0)
            response = future.result()
            if (
                response is None
                or response.error_code.val != MoveItErrorCodes.SUCCESS
            ):
                continue
            solution_map = dict(zip(
                response.solution.joint_state.name,
                response.solution.joint_state.position,
            ))
            if any(name not in solution_map for name in JOINT_NAMES):
                continue
            solution = np.array(
                [solution_map[name] for name in JOINT_NAMES],
                dtype=np.float64,
            )
            branch_change_deg = max(
                abs(math.degrees(float(solution[index] - current_joints[index])))
                for index in (1, 2)
            )
            signature = tuple(
                round(math.degrees(float(solution[index])), 1)
                for index in (1, 2, 5)
            )
            if (
                branch_change_deg < 10.0
                or signature in self.ik_seed_fallback_solutions
            ):
                continue
            self.ik_seed_fallback_solutions.add(signature)
            self.publish_status(
                f'{context}: trying dynamic IK seed {joint_label} '
                f'{offset_deg:+.0f}deg; solution J2/J3/J6='
                f'{signature}'
            )
            self._execute_planned_joint_goal(solution)
            return True
        self.publish_status(
            f'{context}: all dynamic J3/J2 IK seeds exhausted'
        )
        return False

    def _try_opposite_joint_branch(
        self,
        joint_index,
        joint_label,
        used_attribute,
        minimum_tcp_height,
        context,
    ):
        """Switch once to an opposite-sign IK branch at safe height."""
        if getattr(self, used_attribute):
            return False
        try:
            current_pose = self.buffer.lookup_transform(
                self.base_frame, self.moveit_ee_link, Time()
            )
        except TransformException as exc:
            raise RuntimeError(
                f'current TCP transform unavailable: {exc}'
            ) from exc
        current_tcp_z = float(current_pose.transform.translation.z)
        if current_tcp_z < minimum_tcp_height:
            self.publish_status(
                f'{context}: opposite-{joint_label} fallback blocked; TCP Z '
                f'{current_tcp_z * 1000.0:.1f}mm is below '
                f'{minimum_tcp_height * 1000.0:.1f}mm'
            )
            return False
        with self.joint_state_lock:
            current_joints = (
                None if self.latest_joint_positions is None
                else self.latest_joint_positions.copy()
            )
            state_time = self.latest_joint_state_time
        if (
            current_joints is None
            or state_time is None
            or time.monotonic() - state_time > 1.0
        ):
            raise RuntimeError('fresh arm joint state is unavailable')
        if not self.position_ik_client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/compute_ik is unavailable')

        request = GetPositionIK.Request()
        ik_request = request.ik_request
        ik_request.group_name = self.moveit_group
        ik_request.ik_link_name = self.moveit_ee_link
        ik_request.avoid_collisions = True
        ik_request.timeout = duration_message(self.motion_ik_timeout)
        ik_request.robot_state.joint_state.name = list(JOINT_NAMES)
        seed = current_joints.copy()
        current_joint_deg = math.degrees(
            float(current_joints[joint_index])
        )
        seed_joint_deg = (
            -max(30.0, abs(current_joint_deg))
            if current_joint_deg >= 0.0
            else max(30.0, abs(current_joint_deg))
        )
        seed[joint_index] = math.radians(seed_joint_deg)
        ik_request.robot_state.joint_state.position = [
            float(value) for value in seed
        ]
        ik_request.pose_stamped.header.frame_id = self.base_frame
        ik_request.pose_stamped.header.stamp = Time().to_msg()
        ik_request.pose_stamped.pose.position.x = float(
            current_pose.transform.translation.x
        )
        ik_request.pose_stamped.pose.position.y = float(
            current_pose.transform.translation.y
        )
        ik_request.pose_stamped.pose.position.z = float(
            current_pose.transform.translation.z
        )
        ik_request.pose_stamped.pose.orientation = copy.deepcopy(
            current_pose.transform.rotation
        )
        future = self.position_ik_client.call_async(request)
        self._wait_future(future, 5.0)
        response = future.result()
        if (
            response is None
            or response.error_code.val != MoveItErrorCodes.SUCCESS
        ):
            self.publish_status(
                f'{context}: no opposite-{joint_label} IK solution found'
            )
            setattr(self, used_attribute, True)
            return False
        solution_map = dict(zip(
            response.solution.joint_state.name,
            response.solution.joint_state.position,
        ))
        if any(name not in solution_map for name in JOINT_NAMES):
            raise RuntimeError(
                f'opposite-{joint_label} IK returned incomplete joints'
            )
        solution = np.array(
            [solution_map[name] for name in JOINT_NAMES],
            dtype=np.float64,
        )
        solution_joint_deg = math.degrees(
            float(solution[joint_index])
        )
        opposite = (
            abs(current_joint_deg) >= 5.0
            and abs(solution_joint_deg) >= 5.0
            and current_joint_deg * solution_joint_deg < 0.0
        )
        setattr(self, used_attribute, True)
        if not opposite:
            self.publish_status(
                f'{context}: IK solver returned the same {joint_label} '
                f'branch ({current_joint_deg:.1f}deg -> '
                f'{solution_joint_deg:.1f}deg)'
            )
            return False
        self.publish_status(
            f'{context}: switching to opposite {joint_label} branch at '
            f'safe height: {current_joint_deg:.1f}deg -> '
            f'{solution_joint_deg:.1f}deg'
        )
        self._execute_planned_joint_goal(solution)
        return True

    def _execute_planned_joint_goal(self, positions, plan_only=False):
        """Collision-check, and optionally execute, a MoveIt joint goal."""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            raise RuntimeError('MoveIt /arm2/move_action server is unavailable')
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
        constraints = Constraints()
        tolerance = math.radians(0.5)
        for name, position in zip(JOINT_NAMES, positions):
            joint = JointConstraint()
            joint.joint_name = name
            joint.position = float(position)
            joint.tolerance_above = tolerance
            joint.tolerance_below = tolerance
            joint.weight = 1.0
            constraints.joint_constraints.append(joint)
        request.goal_constraints = [constraints]
        goal.planning_options.plan_only = bool(plan_only)
        goal.planning_options.replan = True
        goal.planning_options.replan_attempts = 2
        goal.planning_options.replan_delay = 0.2
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True
        goal_future = self.move_group_client.send_goal_async(goal)
        self._wait_future(goal_future, self.moveit_planning_time + 5.0)
        goal_handle = goal_future.result()
        if goal_handle is None or not goal_handle.accepted:
            mode = 'plan-only' if plan_only else 'execution'
            raise RuntimeError(f'MoveIt rejected joint-goal {mode}')
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
                raise RuntimeError('MoveIt returned no joint-goal result')
            result = wrapped_result.result
            if result.error_code.val != MoveItErrorCodes.SUCCESS:
                mode = 'plan-only' if plan_only else 'execution'
                raise RuntimeError(
                    f'MoveIt joint-goal {mode} failed: '
                    f'code={result.error_code.val}'
                )
        finally:
            with self.moveit_goal_lock:
                self.current_moveit_goal = None

    def _moveit_pose_constraints(
        self,
        pose,
        require_preferred_pick_branch=False,
    ):
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
        if require_preferred_pick_branch:
            # The measured reliable grasp family uses J2 positive and J3
            # negative. Encode those signs in the goal itself so MoveIt does
            # not execute a known-bad pregrasp and try to recover afterward.
            for joint_index, allowed_lower, allowed_upper in (
                (1, 5.0, JOINT_LIMITS_DEG[1][1]),
                (2, JOINT_LIMITS_DEG[2][0], -5.0),
            ):
                midpoint_deg = (allowed_lower + allowed_upper) / 2.0
                half_range_deg = (allowed_upper - allowed_lower) / 2.0
                joint = JointConstraint()
                joint.joint_name = JOINT_NAMES[joint_index]
                joint.position = math.radians(midpoint_deg)
                joint.tolerance_above = math.radians(half_range_deg)
                joint.tolerance_below = math.radians(half_range_deg)
                joint.weight = 1.0
                constraints.joint_constraints.append(joint)
        return constraints

    def move_cartesian_to_pose(
        self,
        target,
        allow_segmented=False,
        segment_count=0,
        prefer_j2_branch_fallback=False,
    ):
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
                'MoveIt /arm2/compute_cartesian_path service is unavailable'
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
        with self.joint_state_lock:
            current_j6 = (
                None if self.latest_joint_positions is None
                else float(self.latest_joint_positions[5])
            )
        if current_j6 is None:
            raise CartesianPlanningError(
                'Cannot validate Cartesian J6 travel without joint state'
            )
        j6_travel = trajectory_joint_travel_degrees(
            current_j6, response.solution, JOINT_NAMES[5]
        )
        if j6_travel > self.max_j6_trajectory_travel:
            raise CartesianPlanningError(
                f'Cartesian J6 path travel {j6_travel:.1f}deg exceeds '
                f'{self.max_j6_trajectory_travel:.1f}deg limit',
                executed_segments=segment_count,
            )
        acceptable = cartesian_path_acceptable(
            response.fraction,
            requested_distance,
            self.cartesian_min_fraction,
            self.cartesian_absolute_min_fraction,
            self.cartesian_max_shortfall,
        )
        if not acceptable:
            can_execute_segment = (
                allow_segmented
                and bool(response.solution.joint_trajectory.points)
                and cartesian_segment_executable(
                    response.fraction,
                    requested_distance,
                    self.stack_segmented_descent_min_fraction,
                    0.005,
                    segment_count,
                    self.stack_segmented_descent_max_segments,
                )
            )
            if can_execute_segment:
                if prefer_j2_branch_fallback and segment_count == 0:
                    diagnostics = self._diagnose_cartesian_limits(request)
                    if (
                        'ik_only=1.000' in diagnostics
                        and 'no_joint_jump=' in diagnostics
                    ):
                        shortfall_mm = (
                            requested_distance
                            * (1.0 - response.fraction)
                            * 1000.0
                        )
                        raise CartesianPlanningError(
                            'Cartesian path rejected before partial descent: '
                            f'fraction={response.fraction:.3f}, '
                            f'shortfall={shortfall_mm:.1f}mm; '
                            f'{diagnostics}',
                            executed_segments=segment_count,
                        )
                progress_mm = (
                    requested_distance * response.fraction * 1000.0
                )
                remaining_mm = (
                    requested_distance * (1.0 - response.fraction) * 1000.0
                )
                self.publish_status(
                    'PICK: executing safe segmented descent '
                    f'{segment_count + 1}/'
                    f'{self.stack_segmented_descent_max_segments}: '
                    f'fraction={response.fraction:.3f}, '
                    f'progress={progress_mm:.1f}mm, '
                    f'remaining={remaining_mm:.1f}mm'
                )
                self._execute_cartesian_trajectory(response.solution)
                try:
                    self.move_cartesian_to_pose(
                        target,
                        allow_segmented=True,
                        segment_count=segment_count + 1,
                        prefer_j2_branch_fallback=(
                            prefer_j2_branch_fallback
                        ),
                    )
                except CartesianPlanningError as exc:
                    exc.executed_segments = max(
                        exc.executed_segments, segment_count + 1
                    )
                    raise
                return
            diagnostics = self._diagnose_cartesian_limits(request)
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
                f'{diagnostics}',
                executed_segments=segment_count,
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

        self._execute_cartesian_trajectory(response.solution)

    def _execute_cartesian_trajectory(self, trajectory):
        if not self.execute_trajectory_client.wait_for_server(
            timeout_sec=5.0
        ):
            raise RuntimeError(
                'MoveIt /arm2/execute_trajectory action is unavailable'
            )
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = trajectory
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

    def _diagnose_cartesian_limits(self, safe_request):
        """Plan unsafe diagnostic variants without executing any of them."""
        variants = (
            ('no_collision', False, self.cartesian_joint_jump),
            ('no_joint_jump', True, 0.0),
            ('ik_only', False, 0.0),
        )
        results = []
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
        return 'diagnostic plan-only fractions: ' + ', '.join(results)

    def move_adaptive_cartesian_lift(self, grasp):
        """Raise TCP to a base-frame absolute Z, independent of pick Z."""
        failures = []
        for target_z in lift_distance_candidates(
            self.lift_after_pick,
            self.minimum_lift_after_pick,
            self.lift_search_step,
        ):
            lift = copy.deepcopy(grasp)
            lift.pose.position.z = target_z
            if float(grasp.pose.position.z) >= target_z:
                self.publish_status(
                    f'PICK: TCP already at or above absolute Z '
                    f'{target_z * 1000.0:.0f}mm'
                )
                return
            lift_error = None
            while True:
                try:
                    self.move_cartesian_to_pose(lift, allow_segmented=True)
                    self.publish_status(
                        f'PICK: raised container to absolute TCP Z '
                        f'{target_z * 1000.0:.0f}mm'
                    )
                    return
                except CartesianPlanningError as exc:
                    lift_error = exc
                    branch_discontinuity = (
                        'ik_only=1.000' in str(exc)
                        and 'no_joint_jump=' in str(exc)
                    )
                    if branch_discontinuity and self.try_opposite_ik_branch(
                        lift, 'PICK vertical lift'
                    ):
                        self.publish_status(
                            'PICK: retrying vertical lift after opposite '
                            'joint-branch switch'
                        )
                        continue
                    break
            if lift_error is not None:
                remaining_distance = self._cartesian_request_distance(lift)
                try:
                    current = self.buffer.lookup_transform(
                        self.base_frame, self.moveit_ee_link, Time()
                    )
                    achieved_z = float(current.transform.translation.z)
                except TransformException:
                    achieved_z = None
                if (
                    lift_error.executed_segments > 0
                    and achieved_z is not None
                    and achieved_z >= self.minimum_lift_after_pick
                ):
                    self.publish_status(
                        'PICK: accepting safe partial vertical lift: '
                        f'TCP Z={achieved_z * 1000.0:.1f}mm, '
                        f'minimum={self.minimum_lift_after_pick * 1000.0:.0f}mm, '
                        f'remaining={remaining_distance * 1000.0:.1f}mm; '
                        'destination clearance stage will continue the rise'
                    )
                    return
                can_finish_with_pose_goal = (
                    lift_error.executed_segments > 0
                    and remaining_distance is not None
                    and remaining_distance
                    <= self.stack_pose_goal_finish_max_distance
                )
                if can_finish_with_pose_goal:
                    self.publish_status(
                        'PICK: finishing short remaining vertical lift with '
                        'pose goal: '
                        f'remaining={remaining_distance * 1000.0:.1f}mm'
                    )
                    try:
                        self.move_to_pose(lift)
                        self.publish_status(
                            f'PICK: raised container to absolute TCP Z '
                            f'{target_z * 1000.0:.0f}mm'
                        )
                        return
                    except RuntimeError as finish_exc:
                        failures.append(
                            f'Z={target_z:.3f}m pose finish: {finish_exc}'
                        )
                        continue
                failures.append(f'Z={target_z:.3f}m: {lift_error}')
                self.get_logger().warning(
                    f'Absolute lift Z {target_z * 1000.0:.0f}mm is not '
                    'feasible; trying a lower safe Z'
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
        raise RuntimeError('ROS request timed out')

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

    def command_return_home(self):
        """Request the serial-owning bridge to return to joint-space home."""
        if self.motion_backend != 'moveit':
            raise RuntimeError(
                'automatic home return requires the MoveIt trajectory bridge'
            )
        if not self.return_home_client.wait_for_service(timeout_sec=3.0):
            raise RuntimeError('JetCobot return-home service is unavailable')
        future = self.return_home_client.call_async(Trigger.Request())
        self._wait_future(future, self.motion_timeout + 5.0)
        result = future.result()
        if result is None or not result.success:
            message = 'no response' if result is None else result.message
            raise RuntimeError(f'return home failed: {message}')

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
