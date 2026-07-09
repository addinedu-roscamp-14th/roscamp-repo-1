#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage


class CalibrationVerifier(Node):
    def __init__(self):
        super().__init__('calibration_verifier')

        self.declare_parameter('calibration_yaml', 'config/central/camera_map_calibration.yaml')
        self.declare_parameter('camera_topic', '/image_rect/compressed')
        self.declare_parameter('map_display_scale', 3.0)
        self.declare_parameter('camera_window_name', 'Verifier camera image')
        self.declare_parameter('map_window_name', 'Verifier projected PGM map')

        self.calibration_yaml_path = self.resolve_path(str(self.get_parameter('calibration_yaml').value))
        self.camera_topic = str(self.get_parameter('camera_topic').value)
        self.map_display_scale = float(self.get_parameter('map_display_scale').value)
        self.camera_window_name = str(self.get_parameter('camera_window_name').value)
        self.map_window_name = str(self.get_parameter('map_window_name').value)

        self.calibration = self.load_yaml(self.calibration_yaml_path)
        self.map_yaml_path = Path(self.calibration['map']['yaml'])
        self.map_image = self.load_map_image(self.map_yaml_path)
        self.map_height, self.map_width = self.map_image.shape[:2]
        self.resolution = float(self.calibration['map']['resolution'])
        self.origin = [float(value) for value in self.calibration['map']['origin']]
        self.homography = np.array(
            self.calibration['homography']['camera_pixel_to_map_xy'],
            dtype=np.float64,
        )

        self.latest_camera_image = None
        self.projected_pgm_pixel = None
        self.pending_camera_pixel = None
        self.validation_count = 0

        self.create_subscription(CompressedImage, self.camera_topic, self.on_camera_image, 10)
        self.timer = self.create_timer(0.03, self.spin_windows)
        self.status_timer = self.create_timer(5.0, self.log_status)

        cv2.namedWindow(self.camera_window_name, cv2.WINDOW_NORMAL)
        cv2.namedWindow(self.map_window_name, cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(self.camera_window_name, self.on_camera_mouse)
        cv2.setMouseCallback(self.map_window_name, self.on_map_mouse)

        self.get_logger().info(f'Loaded calibration: {self.calibration_yaml_path}')
        self.get_logger().info(f'Subscribed camera image: {self.camera_topic}')
        self.get_logger().info(
            'Click a point in the camera window. The projected point is drawn on the PGM map. '
            'Then click the matching real point on the PGM map to measure validation error.'
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

    def load_yaml(self, yaml_path):
        if not yaml_path.exists():
            raise FileNotFoundError(f'Calibration yaml not found: {yaml_path}')

        with open(yaml_path, 'r') as stream:
            data = yaml.safe_load(stream)

        if 'homography' not in data:
            raise ValueError('Calibration yaml does not contain homography. Collect at least 4 pairs first.')

        return data

    def load_map_image(self, map_yaml_path):
        map_config = self.load_plain_yaml(map_yaml_path)
        image_path = Path(map_config['image'])
        if not image_path.is_absolute():
            image_path = map_yaml_path.parent / image_path

        image = cv2.imread(str(image_path), cv2.IMREAD_GRAYSCALE)
        if image is None:
            raise FileNotFoundError(f'Map image not found or unreadable: {image_path}')

        return image

    def load_plain_yaml(self, yaml_path):
        if not yaml_path.exists():
            raise FileNotFoundError(f'Yaml not found: {yaml_path}')

        with open(yaml_path, 'r') as stream:
            return yaml.safe_load(stream)

    def on_camera_image(self, msg):
        try:
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            frame = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
            if frame is None:
                self.get_logger().warn('Failed to decode compressed camera image')
                return

            self.latest_camera_image = frame
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert camera image: {exc}')

    def on_camera_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.pending_camera_pixel = [float(x), float(y)]
        map_xy = self.camera_pixel_to_map_xy(self.pending_camera_pixel)
        self.projected_pgm_pixel = self.map_xy_to_pgm_pixel(map_xy)
        self.get_logger().info(
            f'Camera validation click: camera={self.pending_camera_pixel}, '
            f'projected_map={[round(value, 6) for value in map_xy]}, '
            f'projected_pgm={[round(value, 3) for value in self.projected_pgm_pixel]}'
        )

    def on_map_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if self.projected_pgm_pixel is None:
            self.get_logger().warn('Click a camera point first, then click the matching PGM point.')
            return

        scale = max(self.map_display_scale, 1.0)
        actual_pgm_pixel = [float(x) / scale, float(y) / scale]
        dx_pixels = actual_pgm_pixel[0] - self.projected_pgm_pixel[0]
        dy_pixels = actual_pgm_pixel[1] - self.projected_pgm_pixel[1]
        error_pixels = float(np.hypot(dx_pixels, dy_pixels))
        error_m = error_pixels * self.resolution
        self.validation_count += 1

        self.get_logger().info(
            f'Validation #{self.validation_count}: '
            f'actual_pgm={[round(value, 3) for value in actual_pgm_pixel]}, '
            f'projected_pgm={[round(value, 3) for value in self.projected_pgm_pixel]}, '
            f'error={error_m:.4f} m ({error_pixels:.2f} px)'
        )

    def spin_windows(self):
        map_display = self.build_map_display()
        cv2.imshow(self.map_window_name, map_display)

        if self.latest_camera_image is not None:
            cv2.imshow(self.camera_window_name, self.latest_camera_image)

        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            self.get_logger().info('Closing calibration verifier')
            rclpy.shutdown()

    def build_map_display(self):
        map_bgr = cv2.cvtColor(self.map_image, cv2.COLOR_GRAY2BGR)

        for point in self.calibration.get('points', []):
            pgm_x, pgm_y = point['pgm_pixel']
            cv2.circle(map_bgr, (int(round(pgm_x)), int(round(pgm_y))), 2, (255, 0, 0), -1)

        if self.projected_pgm_pixel is not None:
            pgm_x, pgm_y = self.projected_pgm_pixel
            cv2.drawMarker(
                map_bgr,
                (int(round(pgm_x)), int(round(pgm_y))),
                (0, 0, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=8,
                thickness=1,
            )

        scale = max(self.map_display_scale, 1.0)
        return cv2.resize(
            map_bgr,
            None,
            fx=scale,
            fy=scale,
            interpolation=cv2.INTER_NEAREST,
        )

    def camera_pixel_to_map_xy(self, camera_pixel):
        point = np.array([[camera_pixel]], dtype=np.float64)
        projected = cv2.perspectiveTransform(point, self.homography)
        return projected.reshape(2).tolist()

    def map_xy_to_pgm_pixel(self, map_xy):
        map_x, map_y = map_xy
        pgm_x = (map_x - self.origin[0]) / self.resolution
        pgm_y = self.map_height - ((map_y - self.origin[1]) / self.resolution)
        return [pgm_x, pgm_y]

    def log_status(self):
        if self.latest_camera_image is None:
            self.get_logger().warn(
                f'Waiting for camera frames on {self.camera_topic}. '
                'Check: ros2 topic hz /image_rect/compressed'
            )

    def destroy_node(self):
        cv2.destroyWindow(self.camera_window_name)
        cv2.destroyWindow(self.map_window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrationVerifier()

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
