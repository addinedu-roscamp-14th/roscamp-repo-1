#!/usr/bin/env python3

import json
from pathlib import Path

from geometry_msgs.msg import PointStamped, PoseStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String
import yaml


class CameraToMapBridge(Node):
    def __init__(self):
        super().__init__('camera_to_map_bridge')

        self.declare_parameter('calibration_yaml', 'config/central/camera_map_calibration.yaml')
        self.declare_parameter('input_pixel_topic', '/central/target_pixel')
        self.declare_parameter('output_pose_topic', '/central/target_map_pose')
        self.declare_parameter('output_json_topic', '/central/target_map_json')
        self.declare_parameter('target_id', 'target')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('minimum_direction_distance', 0.02)

        self.calibration_yaml_path = self.resolve_path(
            str(self.get_parameter('calibration_yaml').value)
        )
        self.input_pixel_topic = str(self.get_parameter('input_pixel_topic').value)
        self.output_pose_topic = str(self.get_parameter('output_pose_topic').value)
        self.output_json_topic = str(self.get_parameter('output_json_topic').value)
        self.target_id = str(self.get_parameter('target_id').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.minimum_direction_distance = float(
            self.get_parameter('minimum_direction_distance').value
        )
        self.pending_target = None

        self.homography = self.load_homography(self.calibration_yaml_path)

        self.pose_pub = self.create_publisher(PoseStamped, self.output_pose_topic, 10)
        self.json_pub = self.create_publisher(String, self.output_json_topic, 10)
        self.create_subscription(
            PointStamped,
            self.input_pixel_topic,
            self.on_pixel_point,
            10,
        )
        self.get_logger().info(f'Loaded calibration: {self.calibration_yaml_path}')
        self.get_logger().info(f'Subscribing pixel points: {self.input_pixel_topic}')
        self.get_logger().info(f'Publishing map PoseStamped: {self.output_pose_topic}')
        self.get_logger().info(f'Publishing map JSON: {self.output_json_topic}')
        self.get_logger().info(
            'Click 1: target position, click 2: target heading direction'
        )

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

        self.pose_pub.publish(pose_msg)
        self.json_pub.publish(json_msg)
        self.pending_target = None

        self.get_logger().info(
            f'Published target map_xy={[round(target_x, 6), round(target_y, 6)]}, '
            f'heading={heading_deg:.1f} deg'
        )

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
    except ExternalShutdownException:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
