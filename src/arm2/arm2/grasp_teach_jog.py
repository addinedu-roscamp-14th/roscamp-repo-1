"""Interactive MoveIt TCP jogging and one-key container grasp teaching."""

from collections import deque
import copy
import math
from pathlib import Path
import select
import sys
import termios
import time
import tty

from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, MoveItErrorCodes, OrientationConstraint
from moveit_msgs.msg import PositionConstraint
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.time import Time
from shape_msgs.msg import SolidPrimitive
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from .teach_container_grasp import (
    mean_quaternion,
    quaternion_to_rpy_degrees,
)


def quaternion_from_rpy_degrees(roll, pitch, yaw):
    """Convert degree RPY to an XYZW quaternion."""
    roll, pitch, yaw = [math.radians(value) / 2.0 for value in (roll, pitch, yaw)]
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    ]


class GraspTeachJog(Node):
    """Jog the live physical arm through MoveIt and save its taught grasp."""

    def __init__(self):
        super().__init__('grasp_teach_jog')
        self.base_frame = 'base_link'
        self.tcp_frame = 'TCP'
        self.marker_frame = 'arm2/container_marker'
        self.translation_step = 1.0
        self.rotation_step = 1.0
        self.marker_history = deque(maxlen=100)
        self.buffer = Buffer(cache_time=rclpy.duration.Duration(seconds=30.0))
        self.listener = TransformListener(self.buffer, self)
        self.move_client = ActionClient(self, MoveGroup, '/move_action')
        self.create_timer(0.1, self._capture_marker)

    def _capture_marker(self):
        try:
            transform = self.buffer.lookup_transform(
                self.base_frame, self.marker_frame, Time()
            ).transform
        except TransformException:
            return
        stamp = (
            transform.translation.x,
            transform.translation.y,
            transform.translation.z,
            transform.rotation.x,
            transform.rotation.y,
            transform.rotation.z,
            transform.rotation.w,
        )
        if not self.marker_history or stamp != self.marker_history[-1]:
            self.marker_history.append(stamp)

    def current_pose(self):
        """Read the measured TCP pose from TF."""
        transform = self.buffer.lookup_transform(
            self.base_frame, self.tcp_frame, Time()
        ).transform
        pose = PoseStamped()
        pose.header.frame_id = self.base_frame
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = transform.translation.x
        pose.pose.position.y = transform.translation.y
        pose.pose.position.z = transform.translation.z
        pose.pose.orientation = copy.deepcopy(transform.rotation)
        return pose

    def move_axis(self, axis, delta):
        """Move one Cartesian/RPY axis by a small increment."""
        try:
            target = self.current_pose()
        except TransformException as exc:
            self.get_logger().error(f'TCP TF unavailable: {exc}')
            return
        if axis < 3:
            values = [target.pose.position.x, target.pose.position.y, target.pose.position.z]
            values[axis] += delta / 1000.0
            target.pose.position.x, target.pose.position.y, target.pose.position.z = values
        else:
            rpy = quaternion_to_rpy_degrees([
                target.pose.orientation.x, target.pose.orientation.y,
                target.pose.orientation.z, target.pose.orientation.w,
            ])
            rpy[axis - 3] += delta
            quaternion = quaternion_from_rpy_degrees(*rpy)
            (target.pose.orientation.x, target.pose.orientation.y,
             target.pose.orientation.z, target.pose.orientation.w) = quaternion
        self.send_pose(target)

    def send_pose(self, target):
        """Plan and execute one precise pose increment on the real controller."""
        if not self.move_client.wait_for_server(timeout_sec=3.0):
            self.get_logger().error('/move_action is unavailable')
            return
        constraints = Constraints()
        position = PositionConstraint()
        position.header = target.header
        position.link_name = self.tcp_frame
        sphere = SolidPrimitive()
        sphere.type = SolidPrimitive.SPHERE
        sphere.dimensions = [0.002]
        region_pose = copy.deepcopy(target.pose)
        region_pose.orientation.x = 0.0
        region_pose.orientation.y = 0.0
        region_pose.orientation.z = 0.0
        region_pose.orientation.w = 1.0
        position.constraint_region.primitives = [sphere]
        position.constraint_region.primitive_poses = [region_pose]
        position.weight = 1.0
        orientation = OrientationConstraint()
        orientation.header = target.header
        orientation.link_name = self.tcp_frame
        orientation.orientation = target.pose.orientation
        tolerance = math.radians(5.0)
        orientation.absolute_x_axis_tolerance = tolerance
        orientation.absolute_y_axis_tolerance = tolerance
        orientation.absolute_z_axis_tolerance = tolerance
        orientation.weight = 1.0
        constraints.position_constraints = [position]
        constraints.orientation_constraints = [orientation]

        goal = MoveGroup.Goal()
        goal.request.group_name = 'arm_group'
        goal.request.num_planning_attempts = 3
        goal.request.allowed_planning_time = 3.0
        goal.request.max_velocity_scaling_factor = 0.15
        goal.request.max_acceleration_scaling_factor = 0.15
        goal.request.start_state.is_diff = True
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False
        goal.planning_options.replan = False
        goal.planning_options.planning_scene_diff.is_diff = True
        goal.planning_options.planning_scene_diff.robot_state.is_diff = True

        future = self.move_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        handle = future.result()
        if handle is None or not handle.accepted:
            self.get_logger().warning('Jog goal was rejected')
            return
        result_future = handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=20.0)
        wrapped = result_future.result()
        if wrapped is None:
            self.get_logger().warning('Jog timed out')
            return
        code = wrapped.result.error_code.val
        if code != MoveItErrorCodes.SUCCESS:
            self.get_logger().warning(f'Jog failed: MoveIt code={code}')
            return
        pose = self.current_pose().pose
        self.get_logger().info(
            'TCP xyz_mm=['
            f'{pose.position.x * 1000:.1f}, {pose.position.y * 1000:.1f}, '
            f'{pose.position.z * 1000:.1f}]'
        )

    def save_teach(self):
        """Save the current final grasp using recent stable marker samples."""
        if len(self.marker_history) < 10:
            self.get_logger().error(
                'Need at least 10 visible ArUco samples before teaching'
            )
            return
        values = np.asarray(list(self.marker_history)[-30:], dtype=float)
        marker_xyz = np.median(values[:, :3], axis=0)
        marker_std = float(np.max(np.std(values[:, :3], axis=0)))
        if marker_std > 0.003:
            self.get_logger().error(
                f'ArUco is unstable: max std={marker_std * 1000:.2f}mm'
            )
            return
        marker_q = mean_quaternion(values[:, 3:])
        tcp = self.current_pose().pose
        tcp_xyz = np.array([tcp.position.x, tcp.position.y, tcp.position.z])
        tcp_q = [tcp.orientation.x, tcp.orientation.y,
                 tcp.orientation.z, tcp.orientation.w]
        base_path = Path('config/arm2/arm2_container_pick.yaml')
        parameters = yaml.safe_load(base_path.read_text(encoding='utf-8'))[
            '/arm2/container_pick_coordinator']['ros__parameters']
        container_xyz = (
            marker_xyz
            + np.asarray(parameters['marker_translation_correction_xyz_m'])
            + np.asarray(parameters['container_offset_xyz_m'])
        )
        grasp_offset = tcp_xyz - container_xyz
        extra_depth = float(parameters['grasp_extra_depth_m'])
        grasp_offset[2] += extra_depth
        reconstructed_final = container_xyz + grasp_offset
        reconstructed_final[2] -= extra_depth
        residual_mm = float(np.max(np.abs(
            reconstructed_final - tcp_xyz
        ))) * 1000.0
        if residual_mm > 0.5:
            self.get_logger().error(
                'Teach calculation residual is too large: '
                f'{residual_mm:.3f}mm; calibration was not saved'
            )
            return
        marker_yaw = quaternion_to_rpy_degrees(marker_q)[2]
        tcp_rpy = quaternion_to_rpy_degrees(tcp_q)
        output = {
            '/arm2/container_pick_coordinator': {
                'ros__parameters': {
                    'grasp_offset_xyz_m': [round(float(v), 6) for v in grasp_offset],
                    'grasp_offset_rpy_deg': [round(float(v), 3) for v in tcp_rpy],
                    'reference_marker_yaw_deg': round(float(marker_yaw), 3),
                    'offsets_configured': True,
                }
            }
        }
        output_path = Path('config/arm2/arm2_container_grasp_teach.yaml')
        output_path.write_text(yaml.safe_dump(output, sort_keys=False), encoding='utf-8')
        self.get_logger().info(
            f'TEACH SAVED: {output_path.resolve()}, '
            f'reconstruction_residual={residual_mm:.3f}mm'
        )

    def print_help(self):
        print('\nTCP 미세조작 (통합 launch를 켠 상태에서 사용)')
        print('  W/S: X +/-    A/D: Y +/-    R/F: Z +/-')
        print('  U/O: Roll +/- I/K: Pitch +/- J/L: Yaw +/-')
        print('  1: 1mm/1deg  2: 3mm/3deg  3: 5mm/5deg')
        print('  P: 현재 실제 파지 자세 저장   H: 도움말   Q: 종료\n')

    def run(self):
        """Run the interactive keyboard loop."""
        if not sys.stdin.isatty():
            raise RuntimeError('grasp_teach_jog requires an interactive terminal')
        self.print_help()
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            while rclpy.ok():
                rclpy.spin_once(self, timeout_sec=0.02)
                if not select.select([sys.stdin], [], [], 0.02)[0]:
                    continue
                key = sys.stdin.read(1).lower()
                moves = {
                    'w': (0, self.translation_step), 's': (0, -self.translation_step),
                    'a': (1, self.translation_step), 'd': (1, -self.translation_step),
                    'r': (2, self.translation_step), 'f': (2, -self.translation_step),
                    'u': (3, self.rotation_step), 'o': (3, -self.rotation_step),
                    'i': (4, self.rotation_step), 'k': (4, -self.rotation_step),
                    'j': (5, self.rotation_step), 'l': (5, -self.rotation_step),
                }
                if key in moves:
                    self.move_axis(*moves[key])
                elif key in ('1', '2', '3'):
                    self.translation_step = {'1': 1.0, '2': 3.0, '3': 5.0}[key]
                    self.rotation_step = self.translation_step
                    self.get_logger().info(f'Step={self.translation_step:g}mm/deg')
                elif key == 'p':
                    self.save_teach()
                elif key == 'h':
                    self.print_help()
                elif key == 'q':
                    return
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)


def main(args=None):
    """Start the interactive MoveIt grasp teaching tool."""
    rclpy.init(args=args)
    node = GraspTeachJog()
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
