#!/usr/bin/env python3

from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class DirectCalibrator(Node):
    def __init__(self):
        super().__init__('direct_calibrator')

        self.declare_parameter('map_yaml', 'config/SLAM/current_map.yaml')
        self.declare_parameter('camera_topic', '/camera/image_rect')
        self.declare_parameter('output_yaml', 'config/central/camera_map_calibration.yaml')
        self.declare_parameter('map_display_scale', 3.0)
        self.declare_parameter('camera_window_name', 'Camera calibration image')
        self.declare_parameter('map_window_name', 'PGM calibration map')

        self.map_yaml_path = self.resolve_path(str(self.get_parameter('map_yaml').value))
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.output_yaml_path = self.resolve_path(str(self.get_parameter('output_yaml').value))
        self.map_display_scale = float(self.get_parameter('map_display_scale').value)
        self.camera_window_name = str(self.get_parameter('camera_window_name').value)
        self.map_window_name = str(self.get_parameter('map_window_name').value)

        self.bridge = CvBridge()
        self.map_config = self.load_map_config(self.map_yaml_path)
        self.map_image = self.load_map_image(self.map_yaml_path, self.map_config)
        self.map_width, self.map_height = self.map_image.shape[1], self.map_image.shape[0]
        self.map_display_image = self.build_display_image(self.map_image, self.map_display_scale)
        self.resolution = float(self.map_config['resolution'])
        self.origin = [float(value) for value in self.map_config['origin']]

        self.latest_camera_image = None
        self.last_camera_frame_time = None
        self.pending_camera_pixel = None
        self.pending_pgm_pixel = None
        self.points = []

        self.create_subscription(Image, self.camera_topic, self.on_camera_image, 10)
        self.timer = self.create_timer(0.03, self.spin_windows)
        self.status_timer = self.create_timer(5.0, self.log_status)

        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.map_window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.camera_window_name, self.on_camera_mouse)
        cv2.setMouseCallback(self.map_window_name, self.on_map_mouse)

        self.get_logger().info(f'Subscribed camera image: {self.camera_topic}')
        self.get_logger().info(f'Loaded map: {self.map_yaml_path} ({self.map_width}x{self.map_height})')
        self.get_logger().info('Click matching points in the PGM and camera windows. Press q or ESC to quit.')

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

    def load_map_image(self, map_yaml_path, map_config):
        image_path = Path(map_config['image'])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path

        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f'Map image not found or unreadable: {image_path}')

        return image

    def build_display_image(self, image, scale):
        scale = max(scale, 1.0)
        if scale == 1.0:
            return image

        return cv2.resize(
            image,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )

    def on_camera_image(self, msg):
        try:
            self.latest_camera_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.last_camera_frame_time = self.get_clock().now()
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera image: {exc}')

    def log_status(self):
        if self.latest_camera_image is None:
            self.get_logger().warn(
                f'Waiting for camera frames on {self.camera_topic}. '
                'Calibration must use the rectified camera image. '
                'Start the image rectification node and check: ros2 topic list | grep image_rect'
            )

    def on_camera_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.pending_camera_pixel = [float(x), float(y)]
        self.get_logger().info(f'Camera click: {self.pending_camera_pixel}')
        self.try_add_pair()

    def on_map_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        scale = max(self.map_display_scale, 1.0)
        self.pending_pgm_pixel = [float(x) / scale, float(y) / scale]
        self.get_logger().info(f'PGM click: {self.pending_pgm_pixel}')
        self.try_add_pair()

    def spin_windows(self):
        cv2.imshow(self.map_window_name, self.map_display_image)

        if self.latest_camera_image is not None:
            cv2.imshow(self.camera_window_name, self.latest_camera_image)

        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            self.get_logger().info('Closing direct calibrator')
            rclpy.shutdown()

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
                'camera_image': self.camera_topic,
                'input_mode': 'direct_opencv_clicks',
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

    def destroy_node(self):
        cv2.destroyWindow(self.camera_window_name)
        cv2.destroyWindow(self.map_window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DirectCalibrator()

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
