#!/usr/bin/env python3

from pathlib import Path

import cv2
from geometry_msgs.msg import PointStamped
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class CalibrationCollector(Node):
    def __init__(self):
        super().__init__('calibration_collector')

        self.declare_parameter('map_yaml', 'config/SLAM/current_map.yaml')
        self.declare_parameter('camera_click_topic', '/central/yolo/image_annotated_mouse_left')
        self.declare_parameter('map_click_topic', '/central/calib/pgm_point')
        self.declare_parameter('output_yaml', 'config/central/camera_map_calibration.yaml')

        self.map_yaml_path = self.resolve_path(str(self.get_parameter('map_yaml').value))
        self.camera_click_topic = str(self.get_parameter('camera_click_topic').value)
        self.map_click_topic = str(self.get_parameter('map_click_topic').value)
        self.output_yaml_path = self.resolve_path(str(self.get_parameter('output_yaml').value))

        self.map_config = self.load_map_config(self.map_yaml_path)
        self.map_width, self.map_height = self.load_map_size(self.map_yaml_path, self.map_config)
        self.resolution = float(self.map_config['resolution'])
        self.origin = [float(value) for value in self.map_config['origin']]

        self.pending_camera_pixel = None
        self.pending_pgm_pixel = None
        self.points = []
        self.camera_click_topics = set()
        self.auto_mouse_subscriptions = {}

        if self.camera_click_topic:
            self.add_camera_click_subscription(self.camera_click_topic)
        self.create_subscription(
            PointStamped,
            self.map_click_topic,
            self.on_map_click,
            10,
        )
        self.discovery_timer = self.create_timer(1.0, self.discover_mouse_topics)
        self.status_timer = self.create_timer(5.0, self.log_status)

        self.get_logger().info(
            f'Listening camera clicks: {self.camera_click_topic} '
            'and auto-discovered */mouse_left topics'
        )
        self.get_logger().info(f'Listening PGM map clicks: {self.map_click_topic}')
        self.get_logger().info(
            'Click one point in the PGM map image and the matching point in the camera image. '
            'Order does not matter.'
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

    def load_map_config(self, map_yaml_path):
        if not map_yaml_path.exists():
            raise FileNotFoundError(f'Map yaml not found: {map_yaml_path}')

        with open(map_yaml_path, 'r') as stream:
            return yaml.safe_load(stream)

    def load_map_size(self, map_yaml_path, map_config):
        image_path = Path(map_config['image'])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path

        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f'Map image not found or unreadable: {image_path}')

        height, width = image.shape[:2]
        return width, height

    def add_camera_click_subscription(self, topic_name):
        if topic_name in self.camera_click_topics:
            return

        subscription = self.create_subscription(
            PointStamped,
            topic_name,
            lambda msg, source_topic=topic_name: self.on_camera_click(msg, source_topic),
            10,
        )
        self.camera_click_topics.add(topic_name)
        self.auto_mouse_subscriptions[topic_name] = subscription
        self.get_logger().info(f'Camera click subscription active: {topic_name}')

    def discover_mouse_topics(self):
        for topic_name, topic_types in self.get_topic_names_and_types():
            if not self.is_camera_mouse_topic(topic_name, topic_types):
                continue
            self.add_camera_click_subscription(topic_name)

    def is_camera_mouse_topic(self, topic_name, topic_types):
        if topic_name == self.map_click_topic:
            return False
        if 'mouse_left' not in topic_name:
            return False
        if 'geometry_msgs/msg/PointStamped' not in topic_types:
            return False
        return True

    def on_camera_click(self, msg, source_topic=''):
        self.pending_camera_pixel = self.point_to_pixel(msg)
        suffix = f' from {source_topic}' if source_topic else ''
        self.get_logger().info(f'Camera click{suffix}: {self.pending_camera_pixel}')
        self.try_add_pair()

    def on_map_click(self, msg):
        self.pending_pgm_pixel = self.point_to_pixel(msg)
        self.get_logger().info(f'PGM click: {self.pending_pgm_pixel}')
        self.try_add_pair()

    def point_to_pixel(self, msg):
        if hasattr(msg, 'point'):
            point = msg.point
        else:
            point = msg
        return [float(point.x), float(point.y)]

    def log_status(self):
        pending = []
        if self.pending_pgm_pixel is not None:
            pending.append('PGM')
        if self.pending_camera_pixel is not None:
            pending.append('camera')

        pending_text = ', '.join(pending) if pending else 'none'
        self.get_logger().info(
            f'Calibration status: pairs={len(self.points)}, pending={pending_text}, '
            f'camera_topics={sorted(self.camera_click_topics)}, '
            f'output={self.output_yaml_path}'
        )

    def try_add_pair(self):
        if self.pending_camera_pixel is None or self.pending_pgm_pixel is None:
            return

        map_xy = self.pgm_pixel_to_map_xy(self.pending_pgm_pixel)
        pair = {
            'index': len(self.points) + 1,
            'camera_pixel': self.round_list(self.pending_camera_pixel, 3),
            'pgm_pixel': self.round_list(self.pending_pgm_pixel, 3),
            'map_xy': self.round_list(map_xy, 6),
        }
        self.points.append(pair)
        self.pending_camera_pixel = None
        self.pending_pgm_pixel = None

        self.get_logger().info(
            f'Added calibration pair #{pair["index"]}: '
            f'camera={pair["camera_pixel"]}, pgm={pair["pgm_pixel"]}, map={pair["map_xy"]}'
        )
        self.save_calibration()

    def pgm_pixel_to_map_xy(self, pgm_pixel):
        pgm_x, pgm_y = pgm_pixel
        map_x = self.origin[0] + pgm_x * self.resolution
        map_y = self.origin[1] + (self.map_height - pgm_y) * self.resolution
        return [map_x, map_y]

    def save_calibration(self):
        calibration = {
            'map': {
                'yaml': str(self.map_yaml_path),
                'width': self.map_width,
                'height': self.map_height,
                'resolution': self.resolution,
                'origin': self.origin,
                'pgm_to_map_formula': 'map_x=origin_x+pgm_x*resolution; map_y=origin_y+(height-pgm_y)*resolution',
            },
            'topics': {
                'camera_click': sorted(self.camera_click_topics),
                'map_click': self.map_click_topic,
            },
            'points': self.points,
        }

        homography = self.compute_homography()
        if homography is not None:
            calibration['homography'] = homography

        self.output_yaml_path.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_yaml_path, 'w') as stream:
            yaml.safe_dump(calibration, stream, sort_keys=False, allow_unicode=True)

        self.get_logger().info(f'Saved calibration: {self.output_yaml_path}')

    def compute_homography(self):
        if len(self.points) < 4:
            self.get_logger().info(
                f'Need {4 - len(self.points)} more pair(s) before homography can be computed'
            )
            return None

        camera_points = np.array(
            [point['camera_pixel'] for point in self.points],
            dtype=np.float32,
        )
        map_points = np.array(
            [point['map_xy'] for point in self.points],
            dtype=np.float32,
        )

        matrix, mask = cv2.findHomography(camera_points, map_points, method=0)
        if matrix is None:
            self.get_logger().warn('Homography calculation failed')
            return None

        projected = cv2.perspectiveTransform(camera_points.reshape(-1, 1, 2), matrix)
        projected = projected.reshape(-1, 2)
        errors = np.linalg.norm(projected - map_points, axis=1)

        return {
            'camera_pixel_to_map_xy': self.round_matrix(matrix, 10),
            'inlier_mask': [int(value) for value in mask.ravel().tolist()] if mask is not None else [],
            'reprojection_error_m': {
                'mean': round(float(np.mean(errors)), 6),
                'max': round(float(np.max(errors)), 6),
                'per_point': [round(float(error), 6) for error in errors.tolist()],
            },
        }

    def round_list(self, values, digits):
        return [round(float(value), digits) for value in values]

    def round_matrix(self, matrix, digits):
        return [
            [round(float(value), digits) for value in row]
            for row in matrix.tolist()
        ]


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationCollector()

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
