#!/usr/bin/env python3

import json
import os
from pathlib import Path
import select
import sys
import termios
import tty

from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path as PathMsg
import numpy as np
from porter_interfaces.action import DispatchNavigation
from porter_interfaces.msg import PixelNavigationCommand
import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import Trigger
import yaml


def read_pgm_size(pgm_path):
    with open(pgm_path, 'rb') as stream:
        if stream.readline().strip() not in (b'P2', b'P5'):
            raise ValueError(f'Unsupported PGM format: {pgm_path}')

        tokens = []
        while len(tokens) < 2:
            line = stream.readline()
            if not line:
                break
            tokens.extend(line.split(b'#', 1)[0].split())

    if len(tokens) < 2:
        raise ValueError(f'Invalid PGM header: {pgm_path}')
    return int(tokens[0]), int(tokens[1])


def validate_calibration_map(calibration, calibration_yaml_path):
    map_metadata = calibration.get('map')
    if not isinstance(map_metadata, dict):
        raise ValueError('Calibration yaml does not contain map metadata')

    configured_map_yaml = map_metadata.get('yaml')
    if not configured_map_yaml:
        raise ValueError('Calibration yaml does not contain map.yaml')

    map_yaml_path = Path(configured_map_yaml)
    if not map_yaml_path.is_absolute():
        map_yaml_path = calibration_yaml_path.parent / map_yaml_path
    map_yaml_path = map_yaml_path.resolve()
    if not map_yaml_path.exists():
        raise FileNotFoundError(f'Calibrated map yaml not found: {map_yaml_path}')

    with open(map_yaml_path, 'r') as stream:
        current_map = yaml.safe_load(stream) or {}

    image_path = Path(str(current_map.get('image', '')))
    if not image_path.is_absolute():
        image_path = map_yaml_path.parent / image_path
    image_path = image_path.resolve()
    if not image_path.exists():
        raise FileNotFoundError(f'Current map image not found: {image_path}')

    current_width, current_height = read_pgm_size(image_path)
    expected_width = int(map_metadata.get('width', -1))
    expected_height = int(map_metadata.get('height', -1))
    expected_resolution = float(map_metadata.get('resolution', float('nan')))
    expected_origin = np.asarray(map_metadata.get('origin', []), dtype=np.float64)
    current_resolution = float(current_map.get('resolution', float('nan')))
    current_origin = np.asarray(current_map.get('origin', []), dtype=np.float64)

    mismatches = []
    if (expected_width, expected_height) != (current_width, current_height):
        mismatches.append(
            f'size calibrated={expected_width}x{expected_height}, '
            f'current={current_width}x{current_height}'
        )
    if not np.isclose(expected_resolution, current_resolution, atol=1e-9):
        mismatches.append(
            f'resolution calibrated={expected_resolution}, '
            f'current={current_resolution}'
        )
    if (
        expected_origin.shape != current_origin.shape
        or not np.allclose(expected_origin, current_origin, atol=1e-9)
    ):
        mismatches.append(
            f'origin calibrated={expected_origin.tolist()}, '
            f'current={current_origin.tolist()}'
        )

    if mismatches:
        raise ValueError(
            'Camera-map calibration does not match the current SLAM map: '
            + '; '.join(mismatches)
            + '. Run direct_calibrator again with the current map before '
            'publishing navigation goals.'
        )


def waiting_point_behind_target(target_x, target_y, yaw, distance):
    """Return a staging point behind a final pose in the map frame."""
    return (
        float(target_x - np.cos(yaw) * distance),
        float(target_y - np.sin(yaw) * distance),
    )


class CameraToMapBridge(Node):
    def __init__(self):
        super().__init__('camera_to_map_bridge')

        self.declare_parameter('calibration_yaml', 'config/central/camera_map_calibration.yaml')
        self.declare_parameter('input_pixel_topic', '/central/target_pixel')
        self.declare_parameter(
            'fleet_pixel_command_topic',
            '/central/fleet/pixel_navigation_command',
        )
        self.declare_parameter(
            'dispatch_action',
            '/central/dispatch_navigation',
        )
        self.declare_parameter('output_pose_topic', '/central/target_map_pose')
        self.declare_parameter('output_json_topic', '/central/target_map_json')
        self.declare_parameter('target_id', 'target')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('minimum_direction_distance', 0.02)
        self.declare_parameter('validate_calibration_map', True)
        self.declare_parameter('b1_camera_left_offset_m', 0.15)
        # Signed camera-vertical offset: positive is image-down, negative is
        # image-up.  The measured B-1 stop is 5cm above the former +3cm pose.
        self.declare_parameter('b1_camera_down_offset_m', -0.02)
        self.declare_parameter('b1_waiting_distance_m', 0.25)
        # A-1/A-2/A-3 cargo bins share one fixed, pre-measured map-frame stop
        # pose (measured with RViz "2D Pose Estimate") instead of a pixel ->
        # map conversion, since the loading spot in front of the shelf is the
        # same regardless of which bin is being worked.
        # Fixed A-zone stop measured at rectified camera pixel (157, 262).
        self.declare_parameter('a_zone_map_x', 0.16812885)
        self.declare_parameter('a_zone_map_y', 0.06234431)
        self.declare_parameter('a_zone_map_yaw_deg', 90.0)
        self.declare_parameter('a_zone_stop_back_offset_m', 0.10)
        self.declare_parameter('a_zone_waiting_distance_m', 0.20)
        self.declare_parameter(
            'a_zone_waiting_camera_down_offset_m', 0.05
        )
        self.declare_parameter('waypoint_mode', False)
        self.declare_parameter('enable_spacebar_commit', True)
        self.declare_parameter(
            'output_waypoints_topic', '/central/target_map_waypoints'
        )
        self.declare_parameter(
            'output_waypoints_preview_topic',
            '/central/target_map_waypoints_preview',
        )
        self.declare_parameter(
            'commit_waypoints_service', '/central/commit_waypoints'
        )
        self.declare_parameter(
            'clear_waypoints_service', '/central/clear_waypoints'
        )

        self.calibration_yaml_path = self.resolve_path(
            str(self.get_parameter('calibration_yaml').value)
        )
        self.input_pixel_topic = str(self.get_parameter('input_pixel_topic').value)
        self.fleet_pixel_command_topic = str(
            self.get_parameter('fleet_pixel_command_topic').value
        )
        self.dispatch_action = str(
            self.get_parameter('dispatch_action').value
        )
        self.output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self.output_json_topic = str(self.get_parameter('output_json_topic').value)
        self.target_id = str(self.get_parameter('target_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.minimum_direction_distance = float(
            self.get_parameter('minimum_direction_distance').value
        )
        self.validate_calibration_map = bool(
            self.get_parameter('validate_calibration_map').value
        )
        self.b1_camera_left_offset_m = float(
            self.get_parameter('b1_camera_left_offset_m').value
        )
        if self.b1_camera_left_offset_m < 0.0:
            raise ValueError('b1_camera_left_offset_m must not be negative')
        self.b1_camera_down_offset_m = float(
            self.get_parameter('b1_camera_down_offset_m').value
        )
        self.b1_waiting_distance_m = float(
            self.get_parameter('b1_waiting_distance_m').value
        )
        if self.b1_waiting_distance_m < 0.0:
            raise ValueError('b1_waiting_distance_m must not be negative')
        self.a_zone_map_x = float(self.get_parameter('a_zone_map_x').value)
        self.a_zone_map_y = float(self.get_parameter('a_zone_map_y').value)
        self.a_zone_map_yaw = float(
            np.radians(float(self.get_parameter('a_zone_map_yaw_deg').value))
        )
        self.a_zone_stop_back_offset_m = float(
            self.get_parameter('a_zone_stop_back_offset_m').value
        )
        if self.a_zone_stop_back_offset_m < 0.0:
            raise ValueError('a_zone_stop_back_offset_m must not be negative')
        self.a_zone_waiting_distance_m = float(
            self.get_parameter('a_zone_waiting_distance_m').value
        )
        if self.a_zone_waiting_distance_m < 0.0:
            raise ValueError('a_zone_waiting_distance_m must not be negative')
        self.a_zone_waiting_camera_down_offset_m = float(
            self.get_parameter(
                'a_zone_waiting_camera_down_offset_m'
            ).value
        )
        if self.a_zone_waiting_camera_down_offset_m < 0.0:
            raise ValueError(
                'a_zone_waiting_camera_down_offset_m must not be negative'
            )
        self.waypoint_mode = bool(self.get_parameter('waypoint_mode').value)
        self.enable_spacebar_commit = bool(
            self.get_parameter('enable_spacebar_commit').value
        )
        self.output_waypoints_topic = str(
            self.get_parameter('output_waypoints_topic').value
        )
        self.output_waypoints_preview_topic = str(
            self.get_parameter('output_waypoints_preview_topic').value
        )
        self.commit_waypoints_service = str(
            self.get_parameter('commit_waypoints_service').value
        )
        self.clear_waypoints_service = str(
            self.get_parameter('clear_waypoints_service').value
        )
        self.pending_target = None
        self.waypoint_points = []
        self.stdin_fd = None
        self.stdin_settings = None

        self.homography = self.load_homography(self.calibration_yaml_path)

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.json_pub = self.create_publisher(String, self.output_json_topic, 10)
        self.waypoints_pub = self.create_publisher(
            PathMsg, self.output_waypoints_topic, 10
        )
        self.waypoints_preview_pub = self.create_publisher(
            PathMsg, self.output_waypoints_preview_topic, 10
        )
        self.create_service(
            Trigger, self.commit_waypoints_service, self.on_commit_waypoints
        )
        self.create_service(
            Trigger, self.clear_waypoints_service, self.on_clear_waypoints
        )
        self.create_subscription(
            PointStamped,
            self.input_pixel_topic,
            self.on_pixel_point,
            10,
        )
        self.create_subscription(
            PixelNavigationCommand,
            self.fleet_pixel_command_topic,
            self.on_fleet_pixel_command,
            10,
        )
        self.dispatch_client = ActionClient(
            self,
            DispatchNavigation,
            self.dispatch_action,
        )
        self.get_logger().info(f'Loaded calibration: {self.calibration_yaml_path}')
        self.get_logger().info(f'Subscribing pixel points: {self.input_pixel_topic}')
        self.get_logger().info(f'Publishing map PoseStamped: {self.output_pose_topic}')
        self.get_logger().info(f'Publishing map JSON: {self.output_json_topic}')
        if self.waypoint_mode:
            self.get_logger().info(
                'Waypoint mode enabled. Click intermediate positions once, then '
                'click the final position and final heading direction; '
                f'commit with service {self.commit_waypoints_service}'
            )
            if self.enable_spacebar_commit:
                self.setup_keyboard_commit()
        else:
            self.get_logger().info(
                'Click 1: target position, click 2: target heading direction'
            )

    def on_fleet_pixel_command(self, message):
        target_pixel = [
            float(message.target_pixel.x),
            float(message.target_pixel.y),
        ]
        heading_pixel = [
            float(message.heading_pixel.x),
            float(message.heading_pixel.y),
        ]
        is_a = message.zone_id == 'A' or message.mode == 'parking_a'
        is_b1 = False
        if is_a:
            # A-1/A-2/A-3 share one fixed, pre-measured map pose: skip the
            # pixel -> map conversion entirely (target/heading pixels only
            # exist to satisfy the pixel-goal HTTP schema upstream).
            yaw = self.a_zone_map_yaw
            target_x = float(
                self.a_zone_map_x
                - np.cos(yaw) * self.a_zone_stop_back_offset_m
            )
            target_y = float(
                self.a_zone_map_y
                - np.sin(yaw) * self.a_zone_stop_back_offset_m
            )
            heading_x = target_x + float(np.cos(yaw))
            heading_y = target_y + float(np.sin(yaw))
        else:
            target_x, target_y = self.camera_pixel_to_map_xy(target_pixel)
            heading_x, heading_y = self.camera_pixel_to_map_xy(heading_pixel)
            is_b1 = message.zone_id == 'B-1' or message.mode == 'parking_b1'
            if is_b1:
                left_x, left_y = self.camera_left_map_offset(
                    target_pixel, self.b1_camera_left_offset_m
                )
                down_x, down_y = self.camera_down_map_offset(
                    target_pixel, self.b1_camera_down_offset_m
                )
                target_x += left_x + down_x
                target_y += left_y + down_y
                heading_x += left_x + down_x
                heading_y += left_y + down_y

            delta_x = heading_x - target_x
            delta_y = heading_y - target_y
            distance = float(np.hypot(delta_x, delta_y))
            if distance < self.minimum_direction_distance:
                self.get_logger().warning(
                    f'Rejected fleet command {message.command_id}: '
                    f'heading distance {distance:.3f}m is too short'
                )
                return
            yaw = float(np.arctan2(delta_y, delta_x))
        source = PointStamped()
        source.header = message.header
        pose = self.build_pose_msg(source, target_x, target_y, yaw)
        self.pose_pub.publish(pose)
        self.json_pub.publish(self.build_json_msg(
            source,
            target_pixel,
            heading_pixel,
            target_x,
            target_y,
            heading_x,
            heading_y,
            yaw,
        ))

        if not self.dispatch_client.server_is_ready():
            self.get_logger().error(
                f'Fleet dispatcher unavailable: {self.dispatch_action}'
            )
            return
        goal = DispatchNavigation.Goal()
        goal.command_id = message.command_id
        goal.predecessor_command_id = message.predecessor_command_id
        goal.requested_vehicle_id = message.requested_vehicle_id
        goal.zone_id = 'B-1' if is_b1 else ('A' if is_a else message.zone_id)
        goal.zone_visually_empty = message.zone_visually_empty
        goal.queue_if_busy = message.queue_if_busy
        waiting_distance = (
            self.b1_waiting_distance_m
            if is_b1
            else self.a_zone_waiting_distance_m if is_a else 0.0
        )
        goal.use_waiting_pose = waiting_distance > 0.0
        if goal.use_waiting_pose:
            waiting_x, waiting_y = waiting_point_behind_target(
                target_x,
                target_y,
                yaw,
                waiting_distance,
            )
            if is_a and self.a_zone_waiting_camera_down_offset_m > 0.0:
                down_x, down_y = self.camera_down_map_offset(
                    target_pixel,
                    self.a_zone_waiting_camera_down_offset_m,
                )
                waiting_x += down_x
                waiting_y += down_y
            goal.waiting_pose = self.build_pose_msg(
                source,
                waiting_x,
                waiting_y,
                yaw,
            )
        goal.poses = [pose]
        future = self.dispatch_client.send_goal_async(
            goal,
            feedback_callback=self._on_dispatch_feedback,
        )
        future.add_done_callback(self._on_dispatch_response)
        self.get_logger().info(
            f'Dispatching {message.command_id}: '
            f'vehicle={message.requested_vehicle_id or "AUTO"}, '
            f'zone={goal.zone_id or "-"}, '
            f'map=({target_x:.3f}, {target_y:.3f}), '
            f'waiting_distance={waiting_distance:.2f}m, '
            f'a_waiting_camera_down='
            f'{self.a_zone_waiting_camera_down_offset_m if is_a else 0.0:.2f}m'
        )

    def _on_dispatch_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.get_logger().info(
            f'Fleet feedback: vehicle={feedback.assigned_vehicle_id}, '
            f'state={feedback.state}'
        )

    def _on_dispatch_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'Fleet dispatch request failed: {exc}')
            return
        if not goal_handle.accepted:
            self.get_logger().error('Fleet dispatcher rejected command')
            return
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_dispatch_result)

    def _on_dispatch_result(self, future):
        try:
            result = future.result().result
            message = (
                f'Fleet result: vehicle={result.assigned_vehicle_id}, '
                f'success={result.success}, message={result.message}'
            )
            if result.success:
                self.get_logger().info(message)
            else:
                self.get_logger().error(message)
        except Exception as exc:
            self.get_logger().error(f'Failed to receive fleet result: {exc}')

    def resolve_path(self, configured_path):
        path = Path(configured_path)
        if path.is_absolute():
            return path

        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parents[3] / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

        return candidates[0]

    def load_homography(self, calibration_yaml_path):
        if not calibration_yaml_path.exists():
            raise FileNotFoundError(f'Calibration yaml not found: {calibration_yaml_path}')

        with open(calibration_yaml_path, 'r') as stream:
            calibration = yaml.safe_load(stream)

        if self.validate_calibration_map:
            validate_calibration_map(calibration, calibration_yaml_path)

        try:
            matrix = calibration['homography']['camera_pixel_to_map_xy']
        except KeyError as exc:
            raise ValueError(
                'Calibration yaml does not contain homography.camera_pixel_to_map_xy'
            ) from exc

        return np.array(matrix, dtype=np.float64)

    def on_pixel_point(self, msg):
        camera_pixel = [float(msg.point.x), float(msg.point.y)]
        map_x, map_y = self.camera_pixel_to_map_xy(camera_pixel)

        if self.waypoint_mode:
            self.waypoint_points.append({
                'source_msg': msg,
                'camera_pixel': camera_pixel,
                'map_xy': [map_x, map_y],
            })
            self.publish_waypoints_preview()
            self.get_logger().info(
                f'Added waypoint click {len(self.waypoint_points)}: '
                f'camera_pixel={self.round_list(camera_pixel, 3)}, '
                f'map_xy={[round(map_x, 6), round(map_y, 6)]}'
            )
            return

        if self.pending_target is None:
            self.pending_target = {
                'source_msg': msg,
                'camera_pixel': camera_pixel,
                'map_xy': [map_x, map_y],
            }
            self.get_logger().info(
                f'Target position saved: camera_pixel={self.round_list(camera_pixel, 3)}, '
                f'map_xy={[round(map_x, 6), round(map_y, 6)]}. '
                'Click the direction point.'
            )
            return

        target = self.pending_target
        target_x, target_y = target['map_xy']
        if self.is_b1_parking_message(target['source_msg']):
            left_x, left_y = self.camera_left_map_offset(
                target['camera_pixel'],
                self.b1_camera_left_offset_m,
            )
            down_x, down_y = self.camera_down_map_offset(
                target['camera_pixel'],
                self.b1_camera_down_offset_m,
            )
            offset_x = left_x + down_x
            offset_y = left_y + down_y
            target_x += offset_x
            target_y += offset_y
            map_x += offset_x
            map_y += offset_y
            self.get_logger().info(
                'Applied B-1 parking offsets: '
                f'camera_left={self.b1_camera_left_offset_m:.3f} m, '
                f'camera_down={self.b1_camera_down_offset_m:.3f} m, '
                f'map_delta={[round(offset_x, 6), round(offset_y, 6)]}'
            )
        delta_x = map_x - target_x
        delta_y = map_y - target_y
        direction_distance = float(np.hypot(delta_x, delta_y))
        if direction_distance < self.minimum_direction_distance:
            self.get_logger().warning(
                f'Direction point is too close ({direction_distance:.3f} m). '
                'Click a farther direction point.'
            )
            return

        yaw = float(np.arctan2(delta_y, delta_x))
        heading_deg = float(np.rad2deg(yaw))
        pose_msg = self.build_pose_msg(msg, target_x, target_y, yaw)
        json_msg = self.build_json_msg(
            msg,
            target['camera_pixel'],
            camera_pixel,
            target_x,
            target_y,
            map_x,
            map_y,
            yaw,
        )

        self.pending_target = None

        self.pose_pub.publish(pose_msg)
        self.json_pub.publish(json_msg)
        self.get_logger().info(
            f'Published target map_xy={[round(target_x, 6), round(target_y, 6)]}, '
            f'heading={heading_deg:.1f} deg'
        )

    def on_commit_waypoints(self, _request, response):
        response.success, response.message = self.commit_waypoints()
        return response

    def commit_waypoints(self):
        if not self.waypoint_mode:
            return False, 'waypoint_mode is disabled'

        if len(self.waypoint_points) < 2:
            return False, (
                'At least a final position and final heading click are required'
            )

        try:
            poses = self.build_waypoint_poses()
        except ValueError as exc:
            return False, str(exc)

        path_msg = self.build_path_msg(poses)
        count = len(path_msg.poses)
        self.waypoints_pub.publish(path_msg)
        self.json_pub.publish(self.build_waypoints_json_msg(path_msg))
        self.waypoint_points.clear()
        self.publish_waypoints_preview()

        self.get_logger().info(
            f'Published {count} waypoint(s): {self.output_waypoints_topic}'
        )
        return True, f'Published {count} waypoint(s)'

    def setup_keyboard_commit(self):
        if not sys.stdin.isatty():
            self.get_logger().warning(
                'Spacebar commit disabled because stdin is not a terminal. '
                f'Use service {self.commit_waypoints_service}'
            )
            return

        try:
            self.stdin_fd = sys.stdin.fileno()
            self.stdin_settings = termios.tcgetattr(self.stdin_fd)
            tty.setcbreak(self.stdin_fd)
            self.create_timer(0.05, self.poll_keyboard)
            self.get_logger().info(
                'SPACE: publish all waypoint clicks | service command also available'
            )
        except (OSError, termios.error) as exc:
            self.stdin_fd = None
            self.stdin_settings = None
            self.get_logger().warning(
                f'Failed to enable spacebar commit: {exc}. '
                f'Use service {self.commit_waypoints_service}'
            )

    def poll_keyboard(self):
        if self.stdin_fd is None:
            return

        readable, _, _ = select.select([self.stdin_fd], [], [], 0.0)
        if not readable:
            return

        key = os.read(self.stdin_fd, 1)
        if key != b' ':
            return

        success, message = self.commit_waypoints()
        if not success:
            self.get_logger().warning(f'SPACE commit rejected: {message}')

    def restore_keyboard(self):
        if self.stdin_fd is None or self.stdin_settings is None:
            return
        try:
            termios.tcsetattr(
                self.stdin_fd, termios.TCSADRAIN, self.stdin_settings
            )
        except (OSError, termios.error):
            pass
        self.stdin_fd = None
        self.stdin_settings = None

    def destroy_node(self):
        self.restore_keyboard()
        return super().destroy_node()

    def on_clear_waypoints(self, _request, response):
        click_count = len(self.waypoint_points)
        self.pending_target = None
        self.waypoint_points.clear()
        self.publish_waypoints_preview()
        response.success = True
        response.message = f'Cleared {click_count} waypoint click(s)'
        self.get_logger().info(response.message)
        return response

    def publish_waypoints_preview(self):
        preview_poses = [
            self.build_pose_msg(
                point['source_msg'],
                point['map_xy'][0],
                point['map_xy'][1],
                0.0,
            )
            for point in self.waypoint_points
        ]
        self.waypoints_preview_pub.publish(self.build_path_msg(preview_poses))

    def build_waypoint_poses(self):
        position_points = self.waypoint_points[:-1]
        final_direction = self.waypoint_points[-1]
        poses = []

        for index, point in enumerate(position_points):
            if index + 1 < len(position_points):
                direction_point = position_points[index + 1]
                point_kind = f'waypoint {index + 1} and {index + 2}'
            else:
                direction_point = final_direction
                point_kind = 'final position and final heading point'

            delta_x = direction_point['map_xy'][0] - point['map_xy'][0]
            delta_y = direction_point['map_xy'][1] - point['map_xy'][1]
            distance = float(np.hypot(delta_x, delta_y))
            if distance < self.minimum_direction_distance:
                raise ValueError(
                    f'Distance between {point_kind} is too short '
                    f'({distance:.3f} m)'
                )

            yaw = float(np.arctan2(delta_y, delta_x))
            poses.append(self.build_pose_msg(
                point['source_msg'],
                point['map_xy'][0],
                point['map_xy'][1],
                yaw,
            ))

        return poses

    def build_path_msg(self, poses):
        path_msg = PathMsg()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = self.frame_id
        path_msg.poses = list(poses)
        for pose in path_msg.poses:
            pose.header.frame_id = self.frame_id
        return path_msg

    def build_waypoints_json_msg(self, path_msg):
        waypoints = []
        for index, pose_msg in enumerate(path_msg.poses, start=1):
            orientation = pose_msg.pose.orientation
            yaw = 2.0 * np.arctan2(orientation.z, orientation.w)
            waypoints.append({
                'index': index,
                'x': round(pose_msg.pose.position.x, 6),
                'y': round(pose_msg.pose.position.y, 6),
                'yaw': round(float(yaw), 6),
                'heading_deg': round(float(np.rad2deg(yaw)), 3),
            })

        msg = String()
        msg.data = json.dumps({
            'command': 'navigate_through_poses',
            'frame_id': self.frame_id,
            'target_id': self.target_id,
            'waypoints': waypoints,
        }, ensure_ascii=False)
        return msg

    def camera_pixel_to_map_xy(self, camera_pixel):
        u, v = camera_pixel
        denominator = (
            self.homography[2, 0] * u +
            self.homography[2, 1] * v +
            self.homography[2, 2]
        )
        if abs(denominator) < 1e-12:
            raise ZeroDivisionError('Homography projection denominator is near zero')

        map_x = (
            self.homography[0, 0] * u +
            self.homography[0, 1] * v +
            self.homography[0, 2]
        ) / denominator
        map_y = (
            self.homography[1, 0] * u +
            self.homography[1, 1] * v +
            self.homography[1, 2]
        ) / denominator
        return float(map_x), float(map_y)

    def camera_left_map_offset(self, camera_pixel, distance_m):
        if distance_m == 0.0:
            return 0.0, 0.0

        start_x, start_y = self.camera_pixel_to_map_xy(camera_pixel)
        left_x, left_y = self.camera_pixel_to_map_xy(
            [camera_pixel[0] - 10.0, camera_pixel[1]]
        )
        delta_x = left_x - start_x
        delta_y = left_y - start_y
        norm = float(np.hypot(delta_x, delta_y))
        if norm < 1e-12:
            raise ValueError(
                'Cannot determine camera-left direction from homography'
            )
        scale = distance_m / norm
        return delta_x * scale, delta_y * scale

    def camera_down_map_offset(self, camera_pixel, distance_m):
        if distance_m == 0.0:
            return 0.0, 0.0

        start_x, start_y = self.camera_pixel_to_map_xy(camera_pixel)
        down_x, down_y = self.camera_pixel_to_map_xy(
            [camera_pixel[0], camera_pixel[1] + 10.0]
        )
        delta_x = down_x - start_x
        delta_y = down_y - start_y
        norm = float(np.hypot(delta_x, delta_y))
        if norm < 1e-12:
            raise ValueError(
                'Cannot determine camera-down direction from homography'
            )
        scale = distance_m / norm
        return delta_x * scale, delta_y * scale

    @staticmethod
    def is_b1_parking_message(message):
        return str(message.header.frame_id).endswith('/parking_b1')

    def build_pose_msg(self, source_msg, map_x, map_y, yaw):
        pose_msg = PoseStamped()
        pose_msg.header.stamp = source_msg.header.stamp
        if pose_msg.header.stamp.sec == 0 and pose_msg.header.stamp.nanosec == 0:
            pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = self.frame_id
        pose_msg.pose.position.x = map_x
        pose_msg.pose.position.y = map_y
        pose_msg.pose.position.z = 0.0

        half_yaw = yaw * 0.5
        pose_msg.pose.orientation.z = float(np.sin(half_yaw))
        pose_msg.pose.orientation.w = float(np.cos(half_yaw))
        return pose_msg

    def build_json_msg(
        self,
        source_msg,
        target_camera_pixel,
        direction_camera_pixel,
        map_x,
        map_y,
        direction_map_x,
        direction_map_y,
        yaw,
    ):
        payload = {
            'frame_id': self.frame_id,
            'target_id': self.target_id,
            'source_frame_id': source_msg.header.frame_id,
            'stamp': {
                'sec': int(source_msg.header.stamp.sec),
                'nanosec': int(source_msg.header.stamp.nanosec),
            },
            'target_camera_pixel': {
                'u': round(target_camera_pixel[0], 3),
                'v': round(target_camera_pixel[1], 3),
            },
            'direction_camera_pixel': {
                'u': round(direction_camera_pixel[0], 3),
                'v': round(direction_camera_pixel[1], 3),
            },
            'direction_map_point': {
                'x': round(direction_map_x, 6),
                'y': round(direction_map_y, 6),
            },
            'map_pose': {
                'x': round(map_x, 6),
                'y': round(map_y, 6),
                'z': 0.0,
                'yaw': round(yaw, 6),
                'heading_deg': round(float(np.rad2deg(yaw)), 3),
            },
        }

        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        return msg

    def round_list(self, values, digits):
        return [round(float(value), digits) for value in values]


def main(args=None):
    rclpy.init(args=args)
    node = CameraToMapBridge()

    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
