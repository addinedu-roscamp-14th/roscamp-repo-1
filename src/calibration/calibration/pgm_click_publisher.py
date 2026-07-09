#!/usr/bin/env python3

from pathlib import Path

import cv2
from geometry_msgs.msg import PointStamped
import yaml

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class PgmClickPublisher(Node):
    def __init__(self):
        super().__init__('pgm_click_publisher')

        self.declare_parameter('map_yaml', 'config/SLAM/current_map.yaml')
        self.declare_parameter('point_topic', '/central/calib/pgm_point')
        self.declare_parameter('display_scale', 3.0)
        self.declare_parameter('window_name', 'PGM calibration map')

        self.map_yaml_path = self.resolve_path(str(self.get_parameter('map_yaml').value))
        self.point_topic = str(self.get_parameter('point_topic').value)
        self.display_scale = float(self.get_parameter('display_scale').value)
        self.window_name = str(self.get_parameter('window_name').value)

        self.map_image = self.load_map_image(self.map_yaml_path)
        self.display_image = self.build_display_image(self.map_image, self.display_scale)
        self.publisher = self.create_publisher(PointStamped, self.point_topic, 10)
        self.timer = self.create_timer(0.03, self.spin_window)

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(self.window_name, self.display_image)
        cv2.setMouseCallback(self.window_name, self.on_mouse)

        height, width = self.map_image.shape[:2]
        self.get_logger().info(
            f'PGM click window opened for {width}x{height} map. '
            f'Publishing clicks to {self.point_topic}'
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

    def load_map_image(self, map_yaml_path):
        if not map_yaml_path.exists():
            raise FileNotFoundError(f'Map yaml not found: {map_yaml_path}')

        with open(map_yaml_path, 'r') as stream:
            map_config = yaml.safe_load(stream)

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

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        pgm_x = x / max(self.display_scale, 1.0)
        pgm_y = y / max(self.display_scale, 1.0)

        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map_pgm'
        msg.point.x = float(pgm_x)
        msg.point.y = float(pgm_y)
        msg.point.z = 0.0
        self.publisher.publish(msg)

        self.get_logger().info(f'PGM click published: [{pgm_x:.3f}, {pgm_y:.3f}]')

    def spin_window(self):
        cv2.imshow(self.window_name, self.display_image)
        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            self.get_logger().info('Closing PGM click window')
            rclpy.shutdown()

    def destroy_node(self):
        cv2.destroyWindow(self.window_name)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PgmClickPublisher()

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
