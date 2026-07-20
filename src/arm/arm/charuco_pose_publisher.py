"""Detect a ChArUco board and publish its calibrated 6D camera pose."""

import time

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster

from .aruco_pose_publisher import (
    ArucoPosePublisher,
    rotation_matrix_to_quaternion,
)


def create_charuco_board(
    squares_x,
    squares_y,
    square_length,
    marker_length,
    dictionary,
    legacy_pattern=False,
):
    """Create a board across OpenCV 4.x and 5.x APIs."""
    if hasattr(cv2.aruco, 'CharucoBoard_create'):
        board = cv2.aruco.CharucoBoard_create(
            squares_x,
            squares_y,
            square_length,
            marker_length,
            dictionary,
        )
    else:
        board = cv2.aruco.CharucoBoard(
            (squares_x, squares_y),
            square_length,
            marker_length,
            dictionary,
        )
    if hasattr(board, 'setLegacyPattern'):
        board.setLegacyPattern(bool(legacy_pattern))
    return board


def board_chessboard_corners(board):
    """Return ChArUco object corners across OpenCV versions."""
    if hasattr(board, 'getChessboardCorners'):
        return np.asarray(board.getChessboardCorners(), dtype=np.float64)
    return np.asarray(board.chessboardCorners, dtype=np.float64)


def select_charuco_correspondences(board_corners, image_corners, corner_ids):
    """Match detected corner IDs to board-space and image-space points."""
    ids = np.asarray(corner_ids, dtype=np.int32).reshape(-1)
    image_points = np.asarray(image_corners, dtype=np.float64).reshape(-1, 2)
    if len(ids) != len(image_points):
        raise ValueError('ChArUco corner IDs and image points differ in length')
    if len(ids) == 0 or np.any(ids < 0) or np.any(ids >= len(board_corners)):
        raise ValueError('ChArUco corner IDs are outside the board definition')
    return board_corners[ids], image_points


def charuco_drawing_arrays(image_corners, corner_ids):
    """Normalize OpenCV 4.x and 5.x detector outputs for drawing."""
    corners = np.asarray(image_corners, dtype=np.float32).reshape(-1, 1, 2)
    ids = np.asarray(corner_ids, dtype=np.int32).reshape(-1, 1)
    if len(corners) != len(ids):
        raise ValueError('ChArUco drawing corners and IDs differ in length')
    return corners, ids


def projected_axes_are_visible(
    rotation,
    translation,
    camera_matrix,
    distortion,
    axis_length,
    image_shape,
):
    """Return whether OpenCV can draw every board-axis endpoint safely."""
    axis_points = np.array([
        [0.0, 0.0, 0.0],
        [axis_length, 0.0, 0.0],
        [0.0, axis_length, 0.0],
        [0.0, 0.0, axis_length],
    ], dtype=np.float32)
    projected, _ = cv2.projectPoints(
        axis_points,
        rotation,
        translation,
        camera_matrix,
        distortion,
    )
    pixels = np.asarray(projected, dtype=np.float64).reshape(-1, 2)
    if not np.all(np.isfinite(pixels)):
        return False
    height, width = image_shape[:2]
    return bool(np.all(
        (pixels[:, 0] >= 0.0)
        & (pixels[:, 0] < width)
        & (pixels[:, 1] >= 0.0)
        & (pixels[:, 1] < height)
    ))


class CharucoPosePublisher(Node):
    """Estimate a ChArUco board pose using calibrated camera intrinsics."""

    def __init__(self):
        super().__init__('charuco_pose_publisher')
        self.declare_parameter(
            'image_topic', '/arm/gripper_camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/arm/gripper_camera/camera_info'
        )
        self.declare_parameter(
            'annotated_topic', '/arm/gripper_camera/charuco_annotated'
        )
        self.declare_parameter(
            'pose_topic', '/arm/gripper_camera/charuco_pose'
        )
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('board_frame_id', 'arm/handeye_target')
        self.declare_parameter('dictionary', 'DICT_4X4_50')
        self.declare_parameter('squares_x', 11)
        self.declare_parameter('squares_y', 8)
        self.declare_parameter('square_length_m', 0.015)
        self.declare_parameter('marker_length_m', 0.011)
        self.declare_parameter('legacy_pattern', True)
        self.declare_parameter('detection_rate_hz', 5.0)
        self.declare_parameter('opencv_num_threads', 1)
        self.declare_parameter('minimum_charuco_corners', 6)
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('publish_annotated', True)
        self.declare_parameter('use_node_time_for_pose', False)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value
        )
        self.pose_topic = str(self.get_parameter('pose_topic').value)
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )
        self.board_frame_id = str(
            self.get_parameter('board_frame_id').value
        )
        dictionary_name = str(self.get_parameter('dictionary').value)
        self.squares_x = int(self.get_parameter('squares_x').value)
        self.squares_y = int(self.get_parameter('squares_y').value)
        self.square_length = float(
            self.get_parameter('square_length_m').value
        )
        self.marker_length = float(
            self.get_parameter('marker_length_m').value
        )
        self.legacy_pattern = bool(
            self.get_parameter('legacy_pattern').value
        )
        self.detection_rate_hz = float(
            self.get_parameter('detection_rate_hz').value
        )
        self.opencv_num_threads = int(
            self.get_parameter('opencv_num_threads').value
        )
        self.minimum_corners = int(
            self.get_parameter('minimum_charuco_corners').value
        )
        self.max_reprojection_error = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.use_node_time_for_pose = bool(
            self.get_parameter('use_node_time_for_pose').value
        )

        if self.squares_x < 3 or self.squares_y < 3:
            raise ValueError('ChArUco board must have at least 3x3 squares')
        if not 0.0 < self.marker_length < self.square_length:
            raise ValueError(
                'marker_length_m must be positive and below square_length_m'
            )
        maximum_corners = (self.squares_x - 1) * (self.squares_y - 1)
        if not 4 <= self.minimum_corners <= maximum_corners:
            raise ValueError(
                'minimum_charuco_corners must be between 4 and '
                f'{maximum_corners}'
            )
        if self.max_reprojection_error <= 0.0:
            raise ValueError('max_reprojection_error_px must be positive')
        if self.detection_rate_hz <= 0.0:
            raise ValueError('detection_rate_hz must be positive')
        if self.opencv_num_threads < 1:
            raise ValueError('opencv_num_threads must be at least 1')
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')

        cv2.setNumThreads(self.opencv_num_threads)
        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        self.board = create_charuco_board(
            self.squares_x,
            self.squares_y,
            self.square_length,
            self.marker_length,
            self.dictionary,
            self.legacy_pattern,
        )
        self.board_corners = board_chessboard_corners(self.board)
        self.detector_parameters = self._create_detector_parameters()
        self.marker_detector = None
        self.charuco_detector = None
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_frame = ''
        self._configure_detectors()

        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.detected_once = False
        self.last_marker_count = None
        self.last_corner_count = None
        self.last_detection_time = 0.0
        self.missing_info_warning_count = 0

        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 10
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, 10
        )
        self.create_subscription(
            Image,
            self.image_topic,
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            'Detecting ChArUco board: '
            f'{self.squares_x}x{self.squares_y}, '
            f'square={self.square_length:g}m, '
            f'marker={self.marker_length:g}m, '
            f'dictionary={dictionary_name}, '
            f'legacy_pattern={self.legacy_pattern}, '
            f'rate={self.detection_rate_hz:g}Hz, '
            f'opencv_threads={self.opencv_num_threads}'
        )
        self.get_logger().info(
            f'Publishing board TF: camera -> {self.board_frame_id}'
        )

    @staticmethod
    def _create_detector_parameters():
        if hasattr(cv2.aruco, 'DetectorParameters'):
            return cv2.aruco.DetectorParameters()
        return cv2.aruco.DetectorParameters_create()

    def _configure_detectors(self):
        if hasattr(cv2.aruco, 'CharucoDetector'):
            charuco_parameters = cv2.aruco.CharucoParameters()
            charuco_parameters.tryRefineMarkers = True
            if self.camera_matrix is not None:
                charuco_parameters.cameraMatrix = self.camera_matrix
                charuco_parameters.distCoeffs = self.distortion
            self.charuco_detector = cv2.aruco.CharucoDetector(
                self.board,
                charuco_parameters,
                self.detector_parameters,
            )
            self.marker_detector = None
        elif hasattr(cv2.aruco, 'ArucoDetector'):
            self.marker_detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.detector_parameters
            )
            self.charuco_detector = None

    def on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        distortion = np.asarray(message.d, dtype=np.float64)
        calibration_changed = (
            self.camera_matrix is None
            or not np.array_equal(self.camera_matrix, matrix)
            or not np.array_equal(self.distortion, distortion)
        )
        self.camera_matrix = matrix
        self.distortion = distortion
        self.camera_info_frame = message.header.frame_id
        if calibration_changed:
            self._configure_detectors()

    def _detect_board(self, frame):
        if self.charuco_detector is not None:
            return self.charuco_detector.detectBoard(frame)
        if self.marker_detector is not None:
            marker_corners, marker_ids, _ = (
                self.marker_detector.detectMarkers(frame)
            )
        else:
            marker_corners, marker_ids, _ = cv2.aruco.detectMarkers(
                frame,
                self.dictionary,
                parameters=self.detector_parameters,
            )
        if marker_ids is None or self.camera_matrix is None:
            return None, None, marker_corners, marker_ids
        _count, charuco_corners, charuco_ids = (
            cv2.aruco.interpolateCornersCharuco(
                marker_corners,
                marker_ids,
                frame,
                self.board,
                cameraMatrix=self.camera_matrix,
                distCoeffs=self.distortion,
            )
        )
        return charuco_corners, charuco_ids, marker_corners, marker_ids

    def on_image(self, message):
        now = time.monotonic()
        if now - self.last_detection_time < 1.0 / self.detection_rate_hz:
            return
        self.last_detection_time = now
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert camera image: {exc}')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        charuco_corners, charuco_ids, marker_corners, marker_ids = (
            self._detect_board(gray)
        )
        marker_count = 0 if marker_ids is None else len(marker_ids)
        corner_count = 0 if charuco_ids is None else len(charuco_ids)
        if (
            marker_count != self.last_marker_count
            or corner_count != self.last_corner_count
        ):
            self.get_logger().info(
                'Detected ChArUco: '
                f'markers={marker_count}, corners={corner_count}'
            )
            self.last_marker_count = marker_count
            self.last_corner_count = corner_count

        if corner_count >= self.minimum_corners:
            if self.camera_matrix is None:
                self.missing_info_warning_count += 1
                if self.missing_info_warning_count == 1:
                    self.get_logger().warning(
                        'Board detected, but calibrated CameraInfo is missing'
                    )
            else:
                pose = self.estimate_pose(charuco_corners, charuco_ids)
                if pose is not None:
                    rotation, translation, error = pose
                    self.publish_pose(message, rotation, translation, error)
                    axis_length = self.square_length * 2.0
                    if projected_axes_are_visible(
                        rotation,
                        translation,
                        self.camera_matrix,
                        self.distortion,
                        axis_length,
                        frame.shape,
                    ):
                        cv2.drawFrameAxes(
                            frame,
                            self.camera_matrix,
                            self.distortion,
                            rotation,
                            translation,
                            axis_length,
                        )
                    cv2.putText(
                        frame,
                        f'POSE OK  corners={corner_count}',
                        (12, 28),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.65,
                        (0, 255, 0),
                        2,
                        cv2.LINE_AA,
                    )

        if self.publish_annotated:
            cv2.putText(
                frame,
                f'DETECT markers={marker_count} corners={corner_count}',
                (12, frame.shape[0] - 16),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            if marker_ids is not None:
                cv2.aruco.drawDetectedMarkers(
                    frame, marker_corners, marker_ids
                )
            if charuco_ids is not None:
                drawing_corners, drawing_ids = charuco_drawing_arrays(
                    charuco_corners, charuco_ids
                )
                cv2.aruco.drawDetectedCornersCharuco(
                    frame, drawing_corners, drawing_ids
                )
            annotated = ArucoPosePublisher.make_bgr8_image(frame, message)
            self.annotated_publisher.publish(annotated)

    def estimate_pose(self, image_corners, corner_ids):
        object_points, image_points = select_charuco_correspondences(
            self.board_corners, image_corners, corner_ids
        )
        success, rotation, translation = cv2.solvePnP(
            object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not success or float(translation[2, 0]) <= 0.0:
            return None
        projected, _ = cv2.projectPoints(
            object_points,
            rotation,
            translation,
            self.camera_matrix,
            self.distortion,
        )
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(-1, 2) - image_points) ** 2,
            axis=1,
        ))))
        if error > self.max_reprojection_error:
            self.get_logger().warning(
                'Rejected ChArUco pose: '
                f'reprojection error={error:.2f}px exceeds '
                f'{self.max_reprojection_error:.2f}px'
            )
            return None
        return rotation, translation, error

    def publish_pose(self, image_message, rotation, translation, error):
        camera_frame = (
            self.camera_frame_id
            or self.camera_info_frame
            or image_message.header.frame_id
        )
        if not camera_frame:
            self.get_logger().error('Cannot publish board pose without frame_id')
            return
        rotation_matrix, _ = cv2.Rodrigues(rotation)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        xyz = translation.reshape(3)

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self.use_node_time_for_pose
            else image_message.header.stamp
        )
        transform.header.frame_id = camera_frame
        transform.child_frame_id = self.board_frame_id
        transform.transform.translation.x = float(xyz[0])
        transform.transform.translation.y = float(xyz[1])
        transform.transform.translation.z = float(xyz[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)

        pose = PoseStamped()
        pose.header = transform.header
        pose.pose.position.x = transform.transform.translation.x
        pose.pose.position.y = transform.transform.translation.y
        pose.pose.position.z = transform.transform.translation.z
        pose.pose.orientation = transform.transform.rotation
        self.pose_publisher.publish(pose)
        if not self.detected_once:
            self.detected_once = True
            self.get_logger().info(
                f'ChArUco acquired: corners={self.last_corner_count}, '
                f'z={xyz[2]:.4f}m, reprojection_error={error:.2f}px'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = CharucoPosePublisher()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
