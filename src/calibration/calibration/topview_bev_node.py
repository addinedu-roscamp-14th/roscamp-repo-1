#!/usr/bin/env python3

from pathlib import Path

import cv2
import numpy as np
import yaml

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class TopviewBevNode(Node):
    def __init__(self):
        super().__init__('topview_bev_node')

        self.declare_parameter('input_topic', '/top_camera/image_raw')
        self.declare_parameter('output_topic', '/top_camera/bev/image_raw')
        self.declare_parameter('output_yaml', 'config/central/topview_bev_calibration.yaml')
        self.declare_parameter('output_width', 1000)
        self.declare_parameter('output_height', 700)
        self.declare_parameter('calibration_mode', False)
        self.declare_parameter('show_output_window', False)
        self.declare_parameter('close_window_after_save', True)
        self.declare_parameter('window_name', 'Topview BEV calibration')

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.output_yaml = self.resolve_path(str(self.get_parameter('output_yaml').value))
        self.output_width = int(self.get_parameter('output_width').value)
        self.output_height = int(self.get_parameter('output_height').value)
        self.calibration_mode = bool(self.get_parameter('calibration_mode').value)
        self.show_output_window = bool(self.get_parameter('show_output_window').value)
        self.close_window_after_save = bool(self.get_parameter('close_window_after_save').value)
        self.window_name = str(self.get_parameter('window_name').value)

        self.bridge = CvBridge()
        self.latest_frame = None
        self.clicked_points = []
        self.homography = None

        self.load_calibration()

        self.sub = self.create_subscription(Image, self.input_topic, self.on_image, 10)
        self.pub = self.create_publisher(Image, self.output_topic, 10)
        self.timer = self.create_timer(0.03, self.publish_bev)
        self.status_timer = self.create_timer(5.0, self.log_status)

        if self.calibration_mode:
            # 카메라 원본 해상도와 1:1로 표시해 클릭 좌표가
            # 리사이즈된 화면이 아닌 원본 픽셀 좌표가 되게 합니다.
            cv2.namedWindow(self.window_name, cv2.WINDOW_AUTOSIZE)
            cv2.setMouseCallback(self.window_name, self.on_mouse)

        self.get_logger().info(f'Subscribed topview image: {self.input_topic}')
        self.get_logger().info(f'Publishing BEV image: {self.output_topic}')
        if self.calibration_mode:
            self.get_logger().info('Click 4 floor corner points clockwise, then press s to save. Press r to reset.')

    def resolve_path(self, configured_path):
        path = Path(configured_path)
        if path.is_absolute():
            return path

        candidates = [
            Path.cwd() / path,
            Path.home() / 'poter_ws' / path,
            Path(__file__).resolve().parents[3] / path,
        ]
        for candidate in candidates:
            if candidate.exists():
                return candidate

        return candidates[0]

    def load_calibration(self):
        if not self.output_yaml.exists():
            self.get_logger().warn(f'BEV calibration yaml not found yet: {self.output_yaml}')
            return

        with open(self.output_yaml, 'r') as stream:
            data = yaml.safe_load(stream) or {}

        matrix = data.get('homography', {}).get('image_pixel_to_bev_pixel')
        if matrix is None:
            self.get_logger().warn(f'BEV calibration yaml has no homography: {self.output_yaml}')
            return

        self.homography = np.array(matrix, dtype=np.float64)
        self.output_width = int(data.get('bev', {}).get('width', self.output_width))
        self.output_height = int(data.get('bev', {}).get('height', self.output_height))
        self.get_logger().info(f'Loaded BEV calibration: {self.output_yaml}')

    def on_image(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')

    def on_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if len(self.clicked_points) >= 4:
            self.get_logger().warn('Already have 4 points. Press r to reset or s to save.')
            return

        self.clicked_points.append([float(x), float(y)])
        self.get_logger().info(f'BEV source point #{len(self.clicked_points)}: {self.clicked_points[-1]}')

        if len(self.clicked_points) == 4:
            self.update_homography_from_clicks()

    def update_homography_from_clicks(self):
        src = np.array(self.clicked_points, dtype=np.float32)
        dst = np.array(
            [
                [0.0, 0.0],
                [float(self.output_width - 1), 0.0],
                [float(self.output_width - 1), float(self.output_height - 1)],
                [0.0, float(self.output_height - 1)],
            ],
            dtype=np.float32,
        )
        self.homography = cv2.getPerspectiveTransform(src, dst)

    def publish_bev(self):
        if self.latest_frame is None:
            if self.calibration_mode:
                waiting = self.build_waiting_image(
                    'Waiting for /top_camera/image_raw'
                )
                cv2.imshow(self.window_name, waiting)
                if self.show_output_window:
                    cv2.imshow('Topview BEV output', waiting)
                key = cv2.waitKey(1)
                if key in (27, ord('q')):
                    rclpy.shutdown()
            return

        if self.homography is None:
            bev = cv2.resize(
                self.latest_frame,
                (self.output_width, self.output_height),
                interpolation=cv2.INTER_AREA,
            )
        else:
            bev = cv2.warpPerspective(
                self.latest_frame,
                self.homography,
                (self.output_width, self.output_height),
            )

        msg = self.bridge.cv2_to_imgmsg(bev, encoding='bgr8')
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'top_camera_bev_frame'
        self.pub.publish(msg)

        if self.calibration_mode:
            display = self.latest_frame.copy()
            for index, point in enumerate(self.clicked_points, start=1):
                px = int(round(point[0]))
                py = int(round(point[1]))
                cv2.circle(display, (px, py), 7, (0, 255, 255), -1)
                cv2.putText(
                    display,
                    str(index),
                    (px + 8, py - 8),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 255, 255),
                    2,
                )

            cv2.imshow(self.window_name, display)
            if self.show_output_window:
                cv2.imshow('Topview BEV output', bev)
            key = cv2.waitKey(1)
            if key == ord('r'):
                self.clicked_points = []
                self.homography = None
                self.get_logger().info('Reset BEV calibration points')
            elif key == ord('s'):
                self.save_calibration()
            elif key in (27, ord('q')):
                rclpy.shutdown()

    def save_calibration(self):
        if self.homography is None or len(self.clicked_points) != 4:
            self.get_logger().warn('Need 4 clicked points before saving BEV calibration')
            return

        data = {
            'topics': {
                'input_image': self.input_topic,
                'output_image': self.output_topic,
            },
            'bev': {
                'width': self.output_width,
                'height': self.output_height,
                'source_points_clockwise': self.round_matrix(np.array(self.clicked_points), 3),
            },
            'homography': {
                'image_pixel_to_bev_pixel': self.round_matrix(self.homography, 10),
            },
        }

        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_yaml, 'w') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)

        self.get_logger().info(f'Saved BEV calibration: {self.output_yaml}')

        if self.close_window_after_save:
            self.calibration_mode = False
            self.safe_destroy_window(self.window_name)
            self.safe_destroy_window('Topview BEV output')
            self.get_logger().info('Closed BEV calibration windows after save')

    def round_matrix(self, matrix, digits):
        return [
            [round(float(value), digits) for value in row]
            for row in matrix.tolist()
        ]

    def log_status(self):
        if self.latest_frame is None:
            self.get_logger().warn(f'Waiting for topview frames on {self.input_topic}')

    def build_waiting_image(self, text):
        image = np.zeros((360, 640, 3), dtype=np.uint8)
        cv2.putText(
            image,
            text,
            (30, 180),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )
        return image

    def destroy_node(self):
        self.safe_destroy_window(self.window_name)
        self.safe_destroy_window('Topview BEV output')
        super().destroy_node()

    def safe_destroy_window(self, window_name):
        try:
            cv2.destroyWindow(window_name)
        except cv2.error:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = TopviewBevNode()

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
