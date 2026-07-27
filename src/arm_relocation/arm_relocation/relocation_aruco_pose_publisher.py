"""Detect every ArUco marker and publish poses plus per-frame pixel areas."""

import json
import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster


def rotation_matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix to normalized XYZW."""
    trace = float(np.trace(matrix))
    if trace > 0.0:
        scale = math.sqrt(trace + 1.0) * 2.0
        quaternion = np.array([
            (matrix[2, 1] - matrix[1, 2]) / scale,
            (matrix[0, 2] - matrix[2, 0]) / scale,
            (matrix[1, 0] - matrix[0, 1]) / scale,
            0.25 * scale,
        ])
    else:
        index = int(np.argmax(np.diag(matrix)))
        if index == 0:
            scale = math.sqrt(
                1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]
            ) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(
                1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]
            ) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(
                1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]
            ) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])
    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('rotation produced a zero quaternion')
    return quaternion / norm


def marker_pixel_area(corners):
    """Return the quadrilateral area in pixels."""
    points = np.asarray(corners, dtype=np.float64).reshape(4, 2)
    x = points[:, 0]
    y = points[:, 1]
    return 0.5 * abs(float(np.dot(x, np.roll(y, 1))
                           - np.dot(y, np.roll(x, 1))))


class RelocationArucoPosePublisher(Node):
    """Publish a TF for every valid ID and JSON metadata for each image."""

    def __init__(self):
        super().__init__('relocation_aruco_pose_publisher')
        self.declare_parameter(
            'image_topic', '/arm/gripper_camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/arm/gripper_camera/camera_info'
        )
        self.declare_parameter(
            'annotated_topic',
            '/arm/gripper_camera/relocation_aruco_annotated',
        )
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('marker_frame_prefix', 'arm/relocation_marker_')
        self.declare_parameter('marker_size_m', 0.015)
        self.declare_parameter('dictionary', 'DICT_5X5_50')
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('use_node_time_for_pose', True)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(
            self.get_parameter('camera_info_topic').value
        )
        self.annotated_topic = str(
            self.get_parameter('annotated_topic').value
        )
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )
        self.frame_prefix = str(
            self.get_parameter('marker_frame_prefix').value
        )
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.max_reprojection_error = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.use_node_time = bool(
            self.get_parameter('use_node_time_for_pose').value
        )
        dictionary_name = str(self.get_parameter('dictionary').value)
        if self.marker_size <= 0.0:
            raise ValueError('marker_size_m must be positive')
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'unknown ArUco dictionary: {dictionary_name}')

        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector = cv2.aruco.ArucoDetector(
                dictionary, cv2.aruco.DetectorParameters()
            )
            self.dictionary = None
            self.detector_parameters = None
        else:
            self.detector = None
            self.dictionary = dictionary
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create()
            )

        half = self.marker_size * 0.5
        self.object_points = np.array([
            [-half, half, 0.0],
            [half, half, 0.0],
            [half, -half, 0.0],
            [-half, -half, 0.0],
        ], dtype=np.float64)
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_frame = ''
        self.metadata_publisher = self.create_publisher(
            String, '/arm/gripper_camera/relocation_detections', 10
        )
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 10
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, 10
        )
        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )

    def on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] > 0.0 and matrix[1, 1] > 0.0:
            self.camera_matrix = matrix
            self.distortion = np.asarray(message.d, dtype=np.float64)
            self.camera_info_frame = message.header.frame_id

    def detect(self, frame):
        if self.detector is not None:
            return self.detector.detectMarkers(frame)
        return cv2.aruco.detectMarkers(
            frame,
            self.dictionary,
            parameters=self.detector_parameters,
        )

    def estimate_pose(self, image_points):
        if self.camera_matrix is None:
            return None
        success, rvec, tvec = cv2.solvePnP(
            self.object_points,
            image_points,
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success or float(tvec[2, 0]) <= 0.0:
            return None
        projected, _ = cv2.projectPoints(
            self.object_points,
            rvec,
            tvec,
            self.camera_matrix,
            self.distortion,
        )
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(4, 2) - image_points) ** 2, axis=1
        ))))
        if error > self.max_reprojection_error:
            return None
        return rvec, tvec, error

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'image conversion failed: {exc}')
            return
        corners, ids, _ = self.detect(frame)
        stamp = (
            self.get_clock().now().to_msg()
            if self.use_node_time else message.header.stamp
        )
        detections = []
        if ids is not None and self.camera_matrix is not None:
            for index, raw_id in enumerate(ids.flatten()):
                marker_id = int(raw_id)
                points = np.asarray(
                    corners[index], dtype=np.float64
                ).reshape(4, 2)
                estimate = self.estimate_pose(points)
                if estimate is None:
                    continue
                rvec, tvec, error = estimate
                self.publish_transform(marker_id, stamp, message, rvec, tvec)
                detections.append({
                    'id': marker_id,
                    'area_px': marker_pixel_area(points),
                    'reprojection_error_px': error,
                })
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rvec,
                    tvec,
                    self.marker_size * 0.5,
                )
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        metadata = String()
        metadata.data = json.dumps({
            'stamp_ns': int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec),
            'detections': detections,
        })
        self.metadata_publisher.publish(metadata)
        self.annotated_publisher.publish(self.make_image(frame, message))

    def publish_transform(self, marker_id, stamp, source, rvec, tvec):
        camera_frame = (
            self.camera_frame_id
            or self.camera_info_frame
            or source.header.frame_id
        )
        if not camera_frame:
            return
        matrix, _ = cv2.Rodrigues(rvec)
        quaternion = rotation_matrix_to_quaternion(matrix)
        translation = tvec.reshape(3)
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = camera_frame
        transform.child_frame_id = f'{self.frame_prefix}{marker_id}'
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
        transform.transform.rotation.x = float(quaternion[0])
        transform.transform.rotation.y = float(quaternion[1])
        transform.transform.rotation.z = float(quaternion[2])
        transform.transform.rotation.w = float(quaternion[3])
        self.tf_broadcaster.sendTransform(transform)

    @staticmethod
    def make_image(frame, source):
        contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
        message = Image()
        message.header = source.header
        message.height, message.width = contiguous.shape[:2]
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = int(message.width) * 3
        message.data = contiguous.tobytes()
        return message


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RelocationArucoPosePublisher()
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
