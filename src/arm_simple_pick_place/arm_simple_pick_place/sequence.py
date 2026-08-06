"""Pure motion-sequence construction and pose validation helpers."""

from dataclasses import dataclass
import math


@dataclass(frozen=True)
class MarkerPose:
    """One marker pose frozen in the robot base frame."""

    x_m: float
    y_m: float
    z_m: float
    yaw_deg: float


@dataclass(frozen=True)
class MotionStep:
    """One backend-independent manipulation command."""

    action: str
    pose: tuple[float, float, float, float, float, float] | None = None


@dataclass(frozen=True)
class Heights:
    """Explicit Z values used by the deliberately simple sequence."""

    approach_z_m: float
    pick_z_offset_m: float
    pick_lift_z_m: float
    place_z_offset_m: float
    retreat_z_m: float


def wrap_degrees(value):
    """Wrap an angle to [-180, 180)."""
    return (float(value) + 180.0) % 360.0 - 180.0


def tool_pose(marker, z_m, yaw_offset_deg=0.0):
    """Return raw marker XY with fixed R/P and configured marker yaw."""
    return (
        float(marker.x_m),
        float(marker.y_m),
        float(z_m),
        -180.0,
        0.0,
        wrap_degrees(marker.yaw_deg + yaw_offset_deg),
    )


def pick_steps(marker, heights, yaw_offset_deg=0.0):
    """Build the exact pick sequence after marker discovery."""
    return (
        MotionStep('gripper_open'),
        MotionStep(
            'move',
            tool_pose(marker, heights.approach_z_m, yaw_offset_deg),
        ),
        MotionStep(
            'move',
            tool_pose(
                marker,
                marker.z_m + heights.pick_z_offset_m,
                yaw_offset_deg,
            ),
        ),
        MotionStep('gripper_close'),
        MotionStep(
            'move',
            tool_pose(marker, heights.pick_lift_z_m, yaw_offset_deg),
        ),
    )


def place_steps(marker, heights, yaw_offset_deg=0.0):
    """Build the exact place sequence after marker discovery."""
    return (
        MotionStep(
            'move',
            tool_pose(marker, heights.approach_z_m, yaw_offset_deg),
        ),
        MotionStep(
            'move',
            tool_pose(
                marker,
                marker.z_m + heights.place_z_offset_m,
                yaw_offset_deg,
            ),
        ),
        MotionStep('gripper_open'),
        MotionStep(
            'move',
            tool_pose(marker, heights.retreat_z_m, yaw_offset_deg),
        ),
    )


def observation_roles(operation):
    """Return the ordered ArUco observations required by a service."""
    if operation == 'pick':
        return ('pick',)
    if operation == 'place':
        return ('place',)
    if operation == 'pick_and_place':
        return ('pick', 'place')
    raise ValueError(f'unknown operation: {operation}')


def remaining_observation_roles(required, found):
    """Return required marker roles that have not been frozen yet."""
    return tuple(role for role in required if role not in found)


def pose_errors(actual, target):
    """Return XYZ errors in metres and RPY errors in degrees."""
    if len(actual) != 6 or len(target) != 6:
        raise ValueError('actual and target poses must contain six values')
    xyz = tuple(abs(float(a) - float(b)) for a, b in zip(
        actual[:3], target[:3]
    ))
    rpy = tuple(abs(wrap_degrees(float(a) - float(b))) for a, b in zip(
        actual[3:], target[3:]
    ))
    return xyz, rpy


def pose_within_tolerance(
    actual, target, position_tolerance_m, angle_tolerance_deg
):
    """Check the requested ±position and ±angle limits."""
    if position_tolerance_m <= 0.0 or angle_tolerance_deg <= 0.0:
        raise ValueError('pose tolerances must be positive')
    xyz, rpy = pose_errors(actual, target)
    return (
        all(math.isfinite(value) for value in xyz + rpy)
        and max(xyz) <= position_tolerance_m
        and max(rpy) <= angle_tolerance_deg
    )
