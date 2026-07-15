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


class DualCameraCalibrator(Node):
    def __init__(self):
        super().__init__('dual_camera_calibrator')

        self.declare_parameter('top_bev_topic', '/top_camera/bev/image_raw')
        self.declare_parameter('arm_image_topic', '/camera/image_raw')
        self.declare_parameter('output_yaml', 'config/central/topview_arm_camera_calibration.yaml')
        self.declare_parameter('top_window_name', 'Top BEV image')
        self.declare_parameter('arm_window_name', 'Arm camera image')
        self.declare_parameter('single_window', True)
        self.declare_parameter('combined_window_name', 'Dual camera calibration')
        self.declare_parameter('display_height', 480)
        self.declare_parameter('close_after_homography', True)

        self.top_bev_topic = str(self.get_parameter('top_bev_topic').value)
        self.arm_image_topic = str(self.get_parameter('arm_image_topic').value)
        self.output_yaml = self.resolve_path(str(self.get_parameter('output_yaml').value))
        self.top_window_name = str(self.get_parameter('top_window_name').value)
        self.arm_window_name = str(self.get_parameter('arm_window_name').value)
        self.single_window = bool(self.get_parameter('single_window').value)
        self.combined_window_name = str(self.get_parameter('combined_window_name').value)
        self.display_height = int(self.get_parameter('display_height').value)
        self.close_after_homography = bool(self.get_parameter('close_after_homography').value)
        self.top_display_scale = 1.0
        self.arm_display_scale = 1.0
        self.arm_display_x_offset = 0
        self.top_display_width = 0
        self.arm_display_width = 0
        self.should_close = False

        self.bridge = CvBridge()
        self.latest_top_frame = None
        self.latest_arm_frame = None
        self.pending_top_pixel = None
        self.pending_arm_pixel = None
        self.points = []
        self.load_existing_points()

        self.top_sub = self.create_subscription(Image, self.top_bev_topic, self.on_top_image, 10)
        self.arm_sub = self.create_subscription(Image, self.arm_image_topic, self.on_arm_image, 10)
        self.timer = self.create_timer(0.03, self.spin_windows)
        self.status_timer = self.create_timer(5.0, self.log_status)

        if self.single_window:
            cv2.namedWindow(self.combined_window_name, cv2.WINDOW_NORMAL)
            cv2.moveWindow(self.combined_window_name, 0, 520)
            cv2.setMouseCallback(self.combined_window_name, self.on_combined_mouse)
        else:
            cv2.namedWindow(self.top_window_name, cv2.WINDOW_NORMAL)
            cv2.namedWindow(self.arm_window_name, cv2.WINDOW_NORMAL)
            cv2.setMouseCallback(self.top_window_name, self.on_top_mouse)
            cv2.setMouseCallback(self.arm_window_name, self.on_arm_mouse)

        self.get_logger().info(f'Subscribed top BEV image: {self.top_bev_topic}')
        self.get_logger().info(f'Subscribed arm camera image: {self.arm_image_topic}')
        self.get_logger().info(f'Loaded {len(self.points)} existing calibration pair(s)')
        self.get_logger().info(
            'Click the same real point in both windows. '
            'At least 4 pairs are required. Press u to undo, q to quit.'
        )

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

    def load_existing_points(self):
        if not self.output_yaml.exists():
            return

        try:
            with open(self.output_yaml, 'r') as stream:
                data = yaml.safe_load(stream) or {}
        except Exception as exc:
            self.get_logger().warn(f'Failed to load existing calibration: {exc}')
            return

        points = data.get('points', [])
        if not isinstance(points, list):
            return

        self.points = []
        for index, point in enumerate(points, start=1):
            top_pixel = point.get('top_bev_pixel')
            arm_pixel = point.get('arm_camera_pixel')
            if (
                isinstance(top_pixel, list)
                and isinstance(arm_pixel, list)
                and len(top_pixel) >= 2
                and len(arm_pixel) >= 2
            ):
                self.points.append(
                    {
                        'index': index,
                        'top_bev_pixel': self.round_list(top_pixel[:2], 3),
                        'arm_camera_pixel': self.round_list(arm_pixel[:2], 3),
                    }
                )

    def on_top_image(self, msg):
        try:
            self.latest_top_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert top BEV image: {exc}')

    def on_arm_image(self, msg):
        try:
            self.latest_arm_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert arm camera image: {exc}')

    def on_top_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.pending_top_pixel = [float(x), float(y)]
        self.get_logger().info(f'Top BEV click: {self.pending_top_pixel}')
        self.try_add_pair()

    def on_arm_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        self.pending_arm_pixel = [float(x), float(y)]
        self.get_logger().info(f'Arm camera click: {self.pending_arm_pixel}')
        self.try_add_pair()

    def on_combined_mouse(self, event, x, y, flags, param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        if x < self.arm_display_x_offset:
            if self.top_display_scale <= 0:
                return
            self.pending_top_pixel = [
                float(x) / self.top_display_scale,
                float(y) / self.top_display_scale,
            ]
            self.get_logger().info(f'Top BEV click: {self.pending_top_pixel}')
        else:
            if self.arm_display_scale <= 0:
                return
            self.pending_arm_pixel = [
                float(x - self.arm_display_x_offset) / self.arm_display_scale,
                float(y) / self.arm_display_scale,
            ]
            self.get_logger().info(f'Arm camera click: {self.pending_arm_pixel}')

        self.try_add_pair()

    def try_add_pair(self):
        if self.pending_top_pixel is None or self.pending_arm_pixel is None:
            return

        pair = {
            'index': len(self.points) + 1,
            'top_bev_pixel': self.round_list(self.pending_top_pixel, 3),
            'arm_camera_pixel': self.round_list(self.pending_arm_pixel, 3),
        }
        self.points.append(pair)
        self.pending_top_pixel = None
        self.pending_arm_pixel = None

        self.get_logger().info(
            f'Added pair #{pair["index"]}: '
            f'top={pair["top_bev_pixel"]}, arm={pair["arm_camera_pixel"]}'
        )
        self.save_calibration()

    def spin_windows(self):
        if self.single_window:
            cv2.imshow(self.combined_window_name, self.build_combined_display())
            key = cv2.waitKey(1)
            if key == ord('u'):
                if self.points:
                    removed = self.points.pop()
                    self.get_logger().info(f'Undo pair #{removed["index"]}')
                    self.save_calibration()
            elif key in (27, ord('q')):
                rclpy.shutdown()
            elif self.should_close:
                rclpy.shutdown()
            return

        if self.latest_top_frame is not None:
            cv2.imshow(self.top_window_name, self.draw_points(self.latest_top_frame, 'top_bev_pixel'))
        else:
            cv2.imshow(
                self.top_window_name,
                self.build_waiting_image('Waiting for /top_camera/bev/image_raw'),
            )

        if self.latest_arm_frame is not None:
            cv2.imshow(self.arm_window_name, self.draw_points(self.latest_arm_frame, 'arm_camera_pixel'))
        else:
            cv2.imshow(
                self.arm_window_name,
                self.build_waiting_image(f'Waiting for {self.arm_image_topic}'),
            )

        key = cv2.waitKey(1)
        if key == ord('u'):
            if self.points:
                removed = self.points.pop()
                self.get_logger().info(f'Undo pair #{removed["index"]}')
                self.save_calibration()
        elif key in (27, ord('q')):
            rclpy.shutdown()

    def build_combined_display(self):
        top_frame = (
            self.draw_points(self.latest_top_frame, 'top_bev_pixel')
            if self.latest_top_frame is not None
            else self.build_waiting_image('Waiting for /top_camera/bev/image_raw')
        )
        arm_frame = (
            self.draw_points(self.latest_arm_frame, 'arm_camera_pixel')
            if self.latest_arm_frame is not None
            else self.build_waiting_image(f'Waiting for {self.arm_image_topic}')
        )

        top_display, self.top_display_scale = self.resize_to_height(
            top_frame,
            self.display_height,
        )
        arm_display, self.arm_display_scale = self.resize_to_height(
            arm_frame,
            self.display_height,
        )
        self.top_display_width = top_display.shape[1]
        self.arm_display_width = arm_display.shape[1]
        self.arm_display_x_offset = self.top_display_width

        return np.hstack((top_display, arm_display))

    def resize_to_height(self, frame, target_height):
        height, width = frame.shape[:2]
        if height <= 0:
            return frame, 1.0

        scale = float(target_height) / float(height)
        target_width = max(1, int(round(float(width) * scale)))
        resized = cv2.resize(
            frame,
            (target_width, target_height),
            interpolation=cv2.INTER_AREA,
        )
        return resized, scale

    def draw_points(self, frame, key):
        display = frame.copy()
        for point in self.points:
            px, py = point[key]
            center = (int(round(px)), int(round(py)))
            cv2.circle(display, center, 5, (0, 255, 255), -1)
            cv2.putText(
                display,
                str(point['index']),
                (center[0] + 7, center[1] - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (0, 255, 255),
                2,
            )

        return display

    def save_calibration(self):
        calibration = {
            'topics': {
                'top_bev_image': self.top_bev_topic,
                'arm_image': self.arm_image_topic,
            },
            'points': self.points,
        }

        homography = self.compute_homography()
        if homography is not None:
            calibration['homography'] = homography
            if self.close_after_homography:
                self.should_close = True

        self.output_yaml.parent.mkdir(parents=True, exist_ok=True)
        with open(self.output_yaml, 'w') as stream:
            yaml.safe_dump(calibration, stream, sort_keys=False, allow_unicode=True)

        self.get_logger().info(f'Saved dual-camera calibration: {self.output_yaml}')

    def compute_homography(self):
        if len(self.points) < 4:
            self.get_logger().info(
                f'Need {4 - len(self.points)} more pair(s) before homography can be computed'
            )
            return None

        top_points = np.array(
            [point['top_bev_pixel'] for point in self.points],
            dtype=np.float32,
        )
        arm_points = np.array(
            [point['arm_camera_pixel'] for point in self.points],
            dtype=np.float32,
        )

        top_to_arm, mask = cv2.findHomography(top_points, arm_points, method=0)
        arm_to_top, _ = cv2.findHomography(arm_points, top_points, method=0)
        if top_to_arm is None or arm_to_top is None:
            self.get_logger().warn('Dual-camera homography calculation failed')
            return None

        projected = cv2.perspectiveTransform(top_points.reshape(-1, 1, 2), top_to_arm)
        projected = projected.reshape(-1, 2)
        errors = np.linalg.norm(projected - arm_points, axis=1)

        return {
            'top_bev_pixel_to_arm_camera_pixel': self.round_matrix(top_to_arm, 10),
            'arm_camera_pixel_to_top_bev_pixel': self.round_matrix(arm_to_top, 10),
            'inlier_mask': [int(value) for value in mask.ravel().tolist()] if mask is not None else [],
            'reprojection_error_px': {
                'mean': round(float(np.mean(errors)), 3),
                'max': round(float(np.max(errors)), 3),
                'per_point': [round(float(error), 3) for error in errors.tolist()],
            },
        }

    def round_list(self, values, digits):
        return [round(float(value), digits) for value in values]

    def round_matrix(self, matrix, digits):
        return [
            [round(float(value), digits) for value in row]
            for row in matrix.tolist()
        ]

    def log_status(self):
        if self.latest_top_frame is None:
            self.get_logger().warn(f'Waiting for top BEV frames on {self.top_bev_topic}')
        if self.latest_arm_frame is None:
            self.get_logger().warn(f'Waiting for arm camera frames on {self.arm_image_topic}')

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
        self.safe_destroy_window(self.combined_window_name)
        self.safe_destroy_window(self.top_window_name)
        self.safe_destroy_window(self.arm_window_name)
        super().destroy_node()

    def safe_destroy_window(self, window_name):
        try:
            cv2.destroyWindow(window_name)
        except cv2.error:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = DualCameraCalibrator()

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
