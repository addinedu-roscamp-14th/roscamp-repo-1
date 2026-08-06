"""Tests for container pick transform calculations."""

import threading
from types import SimpleNamespace

from arm.container_pick_coordinator import (
    apply_vertical_pick_offsets,
    cartesian_path_acceptable,
    compose_fixed_base_pose,
    compose_pose,
    compose_symmetric_yaw_follow_poses,
    compose_yaw_follow_pose,
    ContainerPickCoordinator,
    inverted_l_workspace_contains,
    joint_trajectory_metrics,
    lift_distance_candidates,
    quaternion_from_rpy_degrees,
    quaternion_to_rpy_degrees,
)

import numpy as np

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def test_direct_pose_move_uses_send_coords_without_preflight_ik():
    """Direct Cartesian motion is delegated to the JetCobot controller."""
    coordinator = object.__new__(ContainerPickCoordinator)
    coordinator.stop_event = threading.Event()
    coordinator.motion_backend = 'direct'
    coordinator.serial_lock = threading.Lock()
    coordinator.speed = 5
    coordinator.motion_timeout = 1.0
    coordinator.direct_pose_verification = True
    coordinator.direct_xy_tolerance = 0.005
    coordinator.direct_position_tolerance = 0.015
    coordinator.direct_angle_tolerance = 6.0
    coordinator.in_workspace = lambda translation: True
    coordinator.publish_status = lambda text: None

    class Robot:
        def __init__(self):
            self.command = None

        def send_coords(self, coords, speed, mode):
            self.command = (coords, speed, mode)

        def get_coords(self):
            return [100.0, -150.0, 200.0, 0.0, 0.0, 0.0]

    coordinator.robot = Robot()
    pose = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=-0.15, z=0.2),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    ))

    coordinator.move_to_pose(pose)

    assert coordinator.robot.command == (
        [100.0, -150.0, 200.0, 0.0, 0.0, 0.0],
        5,
        0,
    )


def test_direct_pose_can_ignore_error_after_motion_stops():
    """End-to-end testing can log pose error without blocking the sequence."""
    coordinator = object.__new__(ContainerPickCoordinator)
    coordinator.stop_event = threading.Event()
    coordinator.motion_backend = 'direct'
    coordinator.serial_lock = threading.Lock()
    coordinator.speed = 5
    coordinator.motion_timeout = 1.0
    coordinator.direct_pose_verification = False
    coordinator.in_workspace = lambda translation: True
    coordinator.publish_status = lambda text: None

    class Robot:
        def __init__(self):
            self.moving = iter((1, 0, 0, 0))

        def send_coords(self, coords, speed, mode):
            pass

        def is_moving(self):
            return next(self.moving)

        def get_coords(self):
            return [80.0, -130.0, 170.0, 3.0, 2.0, 1.0]

    coordinator.robot = Robot()
    pose = SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=0.1, y=-0.15, z=0.2),
        orientation=SimpleNamespace(x=0.0, y=0.0, z=0.0, w=1.0),
    ))

    coordinator.move_to_pose(pose)


def test_compose_pose_rotates_marker_offset():
    """Marker yaw rotates its local grasp offset into the base frame."""
    rotation = quaternion_from_rpy_degrees(0.0, 0.0, 90.0)
    translation, result_rotation = compose_pose(
        [0.1, 0.2, 0.3],
        rotation,
        [0.05, 0.0, 0.0],
        quaternion_from_rpy_degrees(0.0, 0.0, 0.0),
    )
    assert np.allclose(translation, [0.1, 0.25, 0.3], atol=1e-9)
    assert np.allclose(result_rotation, rotation, atol=1e-9)


def test_rpy_quaternion_round_trip():
    """A configured grasp orientation survives quaternion conversion."""
    expected = [25.0, -30.0, 70.0]
    quaternion = quaternion_from_rpy_degrees(*expected)
    actual = quaternion_to_rpy_degrees(quaternion)
    assert np.allclose(actual, expected, atol=1e-9)


def test_measured_grasp_offset_reproduces_taught_tcp_pose():
    """The measured marker offset recreates the manually taught TCP pose."""
    translation, rotation = compose_pose(
        [0.0702611714, -0.1812154682, 0.0539621953],
        [-0.0109490729, -0.0413679949, 0.9891935085, -0.1402319851],
        [0.018108, 0.009738, -0.041746],
        quaternion_from_rpy_degrees(-166.852, 11.738, -66.238),
    )
    expected_rotation = np.array([0.431, 0.895, -0.106, 0.030])
    expected_rotation /= np.linalg.norm(expected_rotation)

    assert np.allclose(translation, [0.056, -0.192, 0.011], atol=2e-6)
    assert abs(float(np.dot(rotation, expected_rotation))) > 0.999999


def test_fixed_base_grasp_ignores_unstable_marker_rotation():
    """An aligned container uses marker XYZ and a fixed taught TCP attitude."""
    expected_rotation = quaternion_from_rpy_degrees(-170.530, 8.370, 129.248)
    translation, rotation = compose_fixed_base_pose(
        [0.070261, -0.181215, 0.053962],
        [-0.014261, -0.010785, -0.042962],
        expected_rotation,
    )

    assert np.allclose(translation, [0.056, -0.192, 0.011], atol=1e-9)
    assert np.allclose(rotation, expected_rotation, atol=1e-9)


def test_extra_depth_does_not_lower_pregrasp():
    """Extra grasp depth changes final Z without changing visual pregrasp."""
    grasp, pregrasp = apply_vertical_pick_offsets(
        [0.05, -0.18, 0.011], pregrasp_lift=0.08, extra_depth=0.005
    )

    assert np.allclose(grasp, [0.05, -0.18, 0.006])
    assert np.allclose(pregrasp, [0.05, -0.18, 0.091])


def test_yaw_follow_rotates_grasp_and_xy_offset_only():
    """Marker yaw rotates XY and gripper yaw while retaining roll/pitch."""
    translation, rotation, yaw_delta = compose_yaw_follow_pose(
        [0.10, 0.20, 0.05],
        [0.02, 0.0, -0.04],
        [-170.0, 8.0, 120.0],
        marker_yaw_degrees=60.0,
        reference_marker_yaw_degrees=30.0,
    )
    rpy = quaternion_to_rpy_degrees(rotation)

    assert yaw_delta == 30.0
    assert np.allclose(
        translation,
        [0.10 + 0.02 * np.cos(np.deg2rad(30.0)),
         0.20 + 0.02 * np.sin(np.deg2rad(30.0)),
         0.01],
    )
    assert np.allclose(rpy, [-170.0, 8.0, 150.0], atol=1e-9)


def test_symmetric_yaw_keeps_position_and_adds_180_degree_candidate():
    """Both gripper branches share the full-yaw-rotated grasp position."""
    translation, rotations, yaws, yaw_delta = (
        compose_symmetric_yaw_follow_poses(
            [0.10, 0.20, 0.05],
            [0.02, 0.0, -0.04],
            [-170.0, 8.0, 120.0],
            marker_yaw_degrees=170.0,
            reference_marker_yaw_degrees=30.0,
        )
    )

    assert yaw_delta == 140.0
    assert np.allclose(
        translation,
        [0.10 + 0.02 * np.cos(np.deg2rad(140.0)),
         0.20 + 0.02 * np.sin(np.deg2rad(140.0)),
         0.01],
    )
    assert np.allclose(yaws, [-100.0, 80.0])
    assert np.allclose(
        [quaternion_to_rpy_degrees(rotation)[2] for rotation in rotations],
        yaws,
        atol=1e-9,
    )


def test_joint_trajectory_metrics_prioritize_endpoint_then_path():
    """Trajectory metrics measure net joint travel and total route length."""
    points = [
        SimpleNamespace(positions=[0.0, 0.0]),
        SimpleNamespace(positions=[0.5, -0.25]),
        SimpleNamespace(positions=[0.25, -0.5]),
    ]

    endpoint_travel, path_length = joint_trajectory_metrics(points)

    assert endpoint_travel == 0.75
    assert path_length == 1.25


def test_lift_candidates_descend_to_exact_minimum():
    """Adaptive lift searches downward and always tests its minimum."""
    assert np.allclose(
        lift_distance_candidates(0.18, 0.05, 0.02),
        [0.18, 0.16, 0.14, 0.12, 0.10, 0.08, 0.06, 0.05],
    )


def test_cartesian_shortfall_is_bounded_in_metres():
    """A lower fraction is accepted only for a small absolute residual."""
    assert cartesian_path_acceptable(0.955, 0.08, 0.97, 0.90, 0.005)
    assert not cartesian_path_acceptable(0.955, 0.18, 0.97, 0.90, 0.005)
    assert not cartesian_path_acceptable(0.89, 0.01, 0.97, 0.90, 0.005)


def test_inverted_l_workspace_excludes_upper_left_quadrant():
    """The configured bottom/right L rejects only its upper-left cutout."""
    bounds = (
        np.array([-0.28, -0.28]),
        np.array([0.28, 0.0]),
        np.array([0.0, -0.28]),
        np.array([0.28, 0.28]),
    )
    assert inverted_l_workspace_contains([0.20, 0.20], *bounds)
    assert inverted_l_workspace_contains([-0.20, -0.20], *bounds)
    assert inverted_l_workspace_contains([0.20, -0.20], *bounds)
    assert not inverted_l_workspace_contains([-0.20, 0.20], *bounds)


def test_collision_diagnostic_reports_first_contact_pair():
    """Unsafe IK samples expose the first colliding MoveIt body pair."""
    contact = SimpleNamespace(
        contact_body_1='3_Link',
        contact_body_2='gripper_link',
        position=SimpleNamespace(x=0.1, y=-0.2, z=0.03),
        depth=0.0015,
    )

    class Future:
        def __init__(self, response):
            self._response = response

        def result(self):
            return self._response

    class Client:
        @staticmethod
        def wait_for_service(timeout_sec):
            return timeout_sec > 0.0

        @staticmethod
        def call_async(request):
            colliding = request.robot_state.joint_state.position[0] >= 0.6
            return Future(SimpleNamespace(
                valid=not colliding,
                contacts=[contact] if colliding else [],
            ))

    trajectory = JointTrajectory()
    trajectory.joint_names = ['1_Joint']
    trajectory.points = [
        JointTrajectoryPoint(positions=[position])
        for position in (0.0, 0.3, 0.6, 0.9)
    ]
    coordinator = object.__new__(ContainerPickCoordinator)
    coordinator.state_validity_client = Client()
    coordinator.moveit_group = 'arm_group'
    coordinator._wait_future = lambda future, timeout: None

    detail = coordinator._diagnose_collision_contacts(trajectory, 0.5)

    assert 'first invalid state=3/4' in detail
    assert '3_Link<->gripper_link' in detail
    assert 'depth=1.50mm' in detail
