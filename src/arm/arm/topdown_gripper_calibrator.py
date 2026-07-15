"""Calibrate top-down ROI pixels to gripper-view image pixels."""

from pathlib import Path

import cv2
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import yaml

from ._config import BAUD, PORT
from ._robot_utils import connect_robot


class TopdownGripperCalibrator(Node):
    """Collect corresponding points from top-down and gripper cameras."""

    def __init__(self):
        super().__init__('topdown_gripper_calibrator')

        self.declare_parameter('topdown_topic', '/image_rect/compressed')
        self.declare_parameter('gripper_camera_path', '/dev/video4')
        self.declare_parameter('gripper_width', 640)
        self.declare_parameter('gripper_height', 480)
        self.declare_parameter('gripper_fps', 10.0)
        self.declare_parameter('gripper_fourcc', 'YUYV')
        self.declare_parameter('serial_port', PORT)
        self.declare_parameter('baud_rate', BAUD)
        self.declare_parameter('torque_release_seconds', 5.0)
        self.declare_parameter(
            'output_yaml', 'config/arm/topdown_to_gripper_calibration.yaml'
        )
        self.declare_parameter('resume_existing', True)
        self.declare_parameter('ransac_threshold_px', 3.0)

        self.topdown_topic = str(self.get_parameter('topdown_topic').value)
        self.gripper_camera_path = str(
            self.get_parameter('gripper_camera_path').value
        )
        self.gripper_width = int(self.get_parameter('gripper_width').value)
        self.gripper_height = int(self.get_parameter('gripper_height').value)
        self.gripper_fps = float(self.get_parameter('gripper_fps').value)
        self.gripper_fourcc = str(
            self.get_parameter('gripper_fourcc').value
        ).upper()
        self.serial_port = str(self.get_parameter('serial_port').value)
        self.baud_rate = int(self.get_parameter('baud_rate').value)
        self.torque_release_seconds = float(
            self.get_parameter('torque_release_seconds').value
        )
        self.output_path = self._resolve_path(
            str(self.get_parameter('output_yaml').value)
        )
        self.resume_existing = bool(self.get_parameter('resume_existing').value)
        self.ransac_threshold = float(
            self.get_parameter('ransac_threshold_px').value
        )

        self.topdown_frame = None
        self.gripper_frame = None
        self.roi = None
        self.points = []
        self.homography = None
        self.inlier_mask = []
        self.reprojection_errors = []
        self.pending_topdown = None
        self.pending_gripper = None
        self.validation_mode = False
        self.pending_validation = None
        self.validation_projection = None
        self.validation_errors = []
        self.gripper_read_failures = 0
        self.gripper_failure_warning_interval = 100
        self.robot = None
        self.torque_released = False
        self.torque_restore_timer = None
        self.closed = False

        if self.resume_existing and self.output_path.exists():
            self._load_existing()

        self.gripper_capture = cv2.VideoCapture(
            self.gripper_camera_path, cv2.CAP_V4L2
        )
        if not self.gripper_capture.isOpened():
            raise RuntimeError(
                f'Failed to open gripper camera: {self.gripper_camera_path}'
            )
        if len(self.gripper_fourcc) != 4:
            raise RuntimeError('gripper_fourcc must contain exactly 4 characters')
        self.gripper_capture.set(
            cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.gripper_fourcc)
        )
        self.gripper_capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.gripper_width)
        self.gripper_capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.gripper_height)
        self.gripper_capture.set(cv2.CAP_PROP_FPS, self.gripper_fps)

        actual_fourcc = int(self.gripper_capture.get(cv2.CAP_PROP_FOURCC))
        actual_fourcc_text = ''.join(
            chr((actual_fourcc >> (8 * index)) & 0xFF) for index in range(4)
        )
        actual_width = int(
            self.gripper_capture.get(cv2.CAP_PROP_FRAME_WIDTH)
        )
        actual_height = int(
            self.gripper_capture.get(cv2.CAP_PROP_FRAME_HEIGHT)
        )
        actual_fps = self.gripper_capture.get(cv2.CAP_PROP_FPS)
        self.get_logger().info(
            f'Negotiated gripper format: {actual_width}x{actual_height}@'
            f'{actual_fps:g}, {actual_fourcc_text!r}'
        )

        self.create_subscription(
            CompressedImage, self.topdown_topic, self._on_topdown_image, 10
        )
        self.create_timer(0.03, self._update_windows)

        cv2.namedWindow('Topdown ROI calibration', cv2.WINDOW_NORMAL)
        cv2.namedWindow('Gripper view calibration', cv2.WINDOW_NORMAL)
        cv2.setMouseCallback(
            'Topdown ROI calibration', self._on_topdown_click
        )
        cv2.setMouseCallback(
            'Gripper view calibration', self._on_gripper_click
        )

        self.get_logger().info(f'Top-down image: {self.topdown_topic}')
        self.get_logger().info(
            f'Gripper camera: {self.gripper_camera_path}, '
            f'{self.gripper_width}x{self.gripper_height}@{self.gripper_fps:g}, '
            f'{self.gripper_fourcc}'
        )
        self.get_logger().info(f'Calibration output: {self.output_path}')
        self.get_logger().info(
            'Keys: R=select/reset ROI, V=validation mode, U=undo, '
            'S=save, T=release torque for 5 seconds, Q/ESC=quit'
        )

    def _release_torque_temporarily(self):
        if self.torque_released:
            self.get_logger().warning('Torque is already released')
            return

        try:
            if self.robot is None:
                self.robot = connect_robot(self.serial_port, self.baud_rate)

            self.get_logger().warning(
                'Releasing all joint torque. Support the arm by hand.'
            )
            self.torque_released = True
            result = self.robot.release_all_servos()
            if result not in (None, 1):
                raise RuntimeError(f'release_all_servos returned {result}')

            self.torque_restore_timer = self.create_timer(
                self.torque_release_seconds, self._restore_torque
            )
            self.get_logger().warning(
                f'Torque released for {self.torque_release_seconds:g} seconds'
            )
        except Exception as exc:
            self.get_logger().error(f'Failed to release torque: {exc}')
            self._restore_torque()

    def _restore_torque(self):
        if self.torque_restore_timer is not None:
            self.torque_restore_timer.cancel()
            self.destroy_timer(self.torque_restore_timer)
            self.torque_restore_timer = None

        if self.robot is None or not self.torque_released:
            return

        try:
            result = self.robot.focus_all_servos()
            if result not in (None, 1):
                raise RuntimeError(f'focus_all_servos returned {result}')
            self.get_logger().info('All joint torque restored')
        except Exception as exc:
            self.get_logger().error(
                f'CRITICAL: failed to restore joint torque: {exc}'
            )
        finally:
            self.torque_released = False

    def _resolve_path(self, configured_path):
        path = Path(configured_path)
        if path.is_absolute():
            return path

        candidates = [
            Path.cwd() / path,
            Path(__file__).resolve().parents[3] / path,
        ]
        for candidate in candidates:
            if candidate.exists() or candidate.parent.exists():
                return candidate
        return candidates[0]

    def _load_existing(self):
        try:
            with self.output_path.open('r', encoding='utf-8') as stream:
                data = yaml.safe_load(stream) or {}

            roi = data.get('topdown', {}).get('roi')
            if roi:
                self.roi = (
                    int(roi['x']),
                    int(roi['y']),
                    int(roi['width']),
                    int(roi['height']),
                )
            self.points = list(data.get('points', []))
            matrix = data.get('homography', {}).get(
                'topdown_roi_to_gripper_pixel'
            )
            if matrix is not None:
                self.homography = np.asarray(matrix, dtype=np.float64)
            self.validation_errors = list(
                data.get('validation', {}).get('errors_px', [])
            )
            self.get_logger().info(
                f'Resumed calibration with {len(self.points)} point pairs'
            )
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            self.get_logger().warning(
                f'Could not load existing calibration: {exc}'
            )
            self.roi = None
            self.points = []
            self.homography = None

    def _on_topdown_image(self, message):
        encoded = np.frombuffer(message.data, dtype=np.uint8)
        frame = cv2.imdecode(encoded, cv2.IMREAD_COLOR)
        if frame is None:
            self.get_logger().warning('Failed to decode top-down image')
            return
        self.topdown_frame = frame

    def _on_topdown_click(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN or self.roi is None:
            return

        roi_point = [float(x), float(y)]
        if self.validation_mode:
            if self.homography is None:
                self.get_logger().warning(
                    'At least four calibration pairs are required'
                )
                return
            self.pending_validation = roi_point
            self.validation_projection = self._project_point(roi_point)
            self.get_logger().info(
                'Validation prediction: '
                f'topdown_roi={roi_point}, '
                f'gripper={self._round_list(self.validation_projection, 2)}. '
                'Click the actual point in the gripper view.'
            )
            return

        self.pending_topdown = roi_point
        self.get_logger().info(f'Top-down ROI click: {roi_point}')
        self._try_add_pair()

    def _on_gripper_click(self, event, x, y, _flags, _param):
        if event != cv2.EVENT_LBUTTONDOWN:
            return

        gripper_point = [float(x), float(y)]
        if self.validation_mode and self.pending_validation is not None:
            predicted = np.asarray(self.validation_projection, dtype=np.float64)
            actual = np.asarray(gripper_point, dtype=np.float64)
            error = float(np.linalg.norm(predicted - actual))
            self.validation_errors.append(round(error, 3))
            self.get_logger().info(
                f'Validation error: {error:.2f} px '
                f'(predicted={self._round_list(predicted, 2)}, '
                f'actual={gripper_point})'
            )
            self.pending_validation = None
            self.validation_projection = None
            self._save()
            return

        if self.validation_mode:
            self.get_logger().warning(
                'Click a top-down validation point first'
            )
            return

        self.pending_gripper = gripper_point
        self.get_logger().info(f'Gripper-view click: {gripper_point}')
        self._try_add_pair()

    def _try_add_pair(self):
        if self.pending_topdown is None or self.pending_gripper is None:
            return

        roi_x, roi_y, _, _ = self.roi
        pair = {
            'index': len(self.points) + 1,
            'topdown_roi_pixel': self._round_list(self.pending_topdown, 3),
            'topdown_image_pixel': self._round_list(
                [
                    self.pending_topdown[0] + roi_x,
                    self.pending_topdown[1] + roi_y,
                ],
                3,
            ),
            'gripper_pixel': self._round_list(self.pending_gripper, 3),
        }
        self.points.append(pair)
        self.pending_topdown = None
        self.pending_gripper = None

        self._compute_homography()
        self._save()
        self.get_logger().info(
            f'Added pair #{pair["index"]}: '
            f'topdown_roi={pair["topdown_roi_pixel"]}, '
            f'gripper={pair["gripper_pixel"]}'
        )

    def _compute_homography(self):
        if len(self.points) < 4:
            self.homography = None
            self.inlier_mask = []
            self.reprojection_errors = []
            return

        source = np.asarray(
            [point['topdown_roi_pixel'] for point in self.points],
            dtype=np.float32,
        )
        destination = np.asarray(
            [point['gripper_pixel'] for point in self.points],
            dtype=np.float32,
        )
        matrix, mask = cv2.findHomography(
            source, destination, cv2.RANSAC, self.ransac_threshold
        )
        if matrix is None:
            self.get_logger().warning('Homography calculation failed')
            return

        projected = cv2.perspectiveTransform(
            source.reshape(-1, 1, 2), matrix
        ).reshape(-1, 2)
        errors = np.linalg.norm(projected - destination, axis=1)

        self.homography = matrix
        self.inlier_mask = (
            [int(value) for value in mask.ravel()] if mask is not None else []
        )
        self.reprojection_errors = [float(value) for value in errors]
        self.get_logger().info(
            'Homography updated: '
            f'mean={float(np.mean(errors)):.2f} px, '
            f'max={float(np.max(errors)):.2f} px'
        )

    def _project_point(self, point):
        source = np.asarray([[point]], dtype=np.float32)
        projected = cv2.perspectiveTransform(source, self.homography)
        return projected[0, 0].tolist()

    def _select_roi(self):
        if self.topdown_frame is None:
            self.get_logger().warning('Waiting for a top-down frame')
            return

        selected = cv2.selectROI(
            'Select topdown ROI', self.topdown_frame, showCrosshair=True
        )
        cv2.destroyWindow('Select topdown ROI')
        x, y, width, height = [int(value) for value in selected]
        if width <= 0 or height <= 0:
            self.get_logger().warning('ROI selection canceled')
            return

        if self.points:
            self.get_logger().warning(
                'ROI changed; previous calibration points were cleared'
            )
        self.roi = (x, y, width, height)
        self.points = []
        self.homography = None
        self.inlier_mask = []
        self.reprojection_errors = []
        self.validation_errors = []
        self.pending_topdown = None
        self.pending_gripper = None
        self._save()
        self.get_logger().info(f'ROI selected: {self.roi}')

    def _undo(self):
        if not self.points:
            self.get_logger().warning('No calibration pair to undo')
            return
        removed = self.points.pop()
        for index, point in enumerate(self.points, start=1):
            point['index'] = index
        self._compute_homography()
        self._save()
        self.get_logger().info(f'Removed pair #{removed["index"]}')

    def _save(self):
        if self.roi is None:
            return

        roi_x, roi_y, roi_width, roi_height = self.roi
        topdown_size = None
        if self.topdown_frame is not None:
            topdown_size = [
                int(self.topdown_frame.shape[1]),
                int(self.topdown_frame.shape[0]),
            ]
        gripper_size = None
        if self.gripper_frame is not None:
            gripper_size = [
                int(self.gripper_frame.shape[1]),
                int(self.gripper_frame.shape[0]),
            ]

        data = {
            'topdown': {
                'topic': self.topdown_topic,
                'image_size': topdown_size,
                'roi': {
                    'x': roi_x,
                    'y': roi_y,
                    'width': roi_width,
                    'height': roi_height,
                },
            },
            'gripper_camera': {
                'device': self.gripper_camera_path,
                'image_size': gripper_size,
            },
            'points': self.points,
        }
        if self.homography is not None:
            data['homography'] = {
                'topdown_roi_to_gripper_pixel': [
                    [round(float(value), 10) for value in row]
                    for row in self.homography
                ],
                'inlier_mask': self.inlier_mask,
                'ransac_threshold_px': self.ransac_threshold,
                'reprojection_error_px': {
                    'mean': round(
                        float(np.mean(self.reprojection_errors)), 3
                    ),
                    'max': round(
                        float(np.max(self.reprojection_errors)), 3
                    ),
                    'per_point': [
                        round(value, 3)
                        for value in self.reprojection_errors
                    ],
                },
            }
        if self.validation_errors:
            data['validation'] = {
                'count': len(self.validation_errors),
                'mean_error_px': round(
                    float(np.mean(self.validation_errors)), 3
                ),
                'max_error_px': round(
                    float(np.max(self.validation_errors)), 3
                ),
                'errors_px': self.validation_errors,
            }

        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        with self.output_path.open('w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=True)

    def _draw_status(self, image, lines):
        for index, line in enumerate(lines):
            y = 24 + index * 24
            cv2.putText(
                image,
                line,
                (10, y),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )

    def _topdown_display(self):
        if self.topdown_frame is None:
            return np.zeros((240, 400, 3), dtype=np.uint8)
        if self.roi is None:
            display = self.topdown_frame.copy()
            self._draw_status(display, ['Press R to select arm ROI'])
            return display

        x, y, width, height = self.roi
        x2 = min(x + width, self.topdown_frame.shape[1])
        y2 = min(y + height, self.topdown_frame.shape[0])
        display = self.topdown_frame[y:y2, x:x2].copy()
        for point in self.points:
            px, py = [int(round(value)) for value in point['topdown_roi_pixel']]
            cv2.circle(display, (px, py), 5, (0, 255, 0), -1)
            cv2.putText(
                display,
                str(point['index']),
                (px + 7, py - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        mode = 'VALIDATE' if self.validation_mode else 'CALIBRATE'
        self._draw_status(
            display,
            [f'{mode} | pairs={len(self.points)}', 'R ROI | V mode | U undo | S save'],
        )
        return display

    def _gripper_display(self):
        if self.gripper_frame is None:
            display = np.zeros((240, 400, 3), dtype=np.uint8)
            self._draw_status(
                display,
                [
                    f'Waiting for {self.gripper_camera_path}',
                    f'Failed reads: {self.gripper_read_failures}',
                ],
            )
            return display
        display = self.gripper_frame.copy()
        for point in self.points:
            px, py = [int(round(value)) for value in point['gripper_pixel']]
            cv2.circle(display, (px, py), 5, (0, 255, 0), -1)
            cv2.putText(
                display,
                str(point['index']),
                (px + 7, py - 7),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                1,
                cv2.LINE_AA,
            )
        if self.validation_projection is not None:
            px, py = [
                int(round(value)) for value in self.validation_projection
            ]
            cv2.drawMarker(
                display,
                (px, py),
                (0, 0, 255),
                cv2.MARKER_CROSS,
                24,
                2,
            )
        self._draw_status(
            display,
            ['Click the same physical point', 'Red cross = validation prediction'],
        )
        return display

    def _update_windows(self):
        success, frame = self.gripper_capture.read()
        if success:
            self.gripper_frame = frame
            self.gripper_read_failures = 0
        else:
            self.gripper_read_failures += 1
            if (
                self.gripper_read_failures == 1
                or self.gripper_read_failures
                % self.gripper_failure_warning_interval == 0
            ):
                self.get_logger().warning(
                    f'No frame from {self.gripper_camera_path}; '
                    f'consecutive failures={self.gripper_read_failures}. '
                    'Check device mapping, camera ownership, format, and USB bandwidth.'
                )

        cv2.imshow('Topdown ROI calibration', self._topdown_display())
        cv2.imshow('Gripper view calibration', self._gripper_display())
        key = cv2.waitKey(1) & 0xFF
        if key in (27, ord('q'), ord('Q')):
            rclpy.shutdown()
        elif key in (ord('r'), ord('R')):
            self._select_roi()
        elif key in (ord('v'), ord('V')):
            self.validation_mode = not self.validation_mode
            self.pending_validation = None
            self.validation_projection = None
            self.get_logger().info(
                f'Validation mode: {self.validation_mode}'
            )
        elif key in (ord('u'), ord('U')):
            self._undo()
        elif key in (ord('s'), ord('S')):
            self._save()
            self.get_logger().info(f'Saved calibration: {self.output_path}')
        elif key in (ord('t'), ord('T')):
            self._release_torque_temporarily()

    @staticmethod
    def _round_list(values, digits):
        return [round(float(value), digits) for value in values]

    def close(self):
        if self.closed:
            return
        self._restore_torque()
        self._save()
        self.gripper_capture.release()
        cv2.destroyAllWindows()
        self.closed = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TopdownGripperCalibrator()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    except RuntimeError as exc:
        if node is None:
            print(f'Calibration startup failed: {exc}')
        else:
            node.get_logger().error(str(exc))
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
