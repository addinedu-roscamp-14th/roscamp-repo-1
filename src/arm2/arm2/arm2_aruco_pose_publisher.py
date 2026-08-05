"""Detect one ArUco marker and publish its 6D pose in the camera frame."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import (
    Buffer,
    TransformBroadcaster,
    TransformException,
    TransformListener,
)


def rotation_matrix_to_quaternion(matrix):
    """Convert a 3x3 rotation matrix to a normalized XYZW quaternion."""
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
        diagonal = np.diag(matrix)
        index = int(np.argmax(diagonal))
        if index == 0:
            scale = math.sqrt(1.0 + matrix[0, 0] - matrix[1, 1] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                0.25 * scale,
                (matrix[0, 1] + matrix[1, 0]) / scale,
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[2, 1] - matrix[1, 2]) / scale,
            ])
        elif index == 1:
            scale = math.sqrt(1.0 + matrix[1, 1] - matrix[0, 0] - matrix[2, 2]) * 2.0
            quaternion = np.array([
                (matrix[0, 1] + matrix[1, 0]) / scale,
                0.25 * scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                (matrix[0, 2] - matrix[2, 0]) / scale,
            ])
        else:
            scale = math.sqrt(1.0 + matrix[2, 2] - matrix[0, 0] - matrix[1, 1]) * 2.0
            quaternion = np.array([
                (matrix[0, 2] + matrix[2, 0]) / scale,
                (matrix[1, 2] + matrix[2, 1]) / scale,
                0.25 * scale,
                (matrix[1, 0] - matrix[0, 1]) / scale,
            ])

    norm = float(np.linalg.norm(quaternion))
    if norm < 1e-12:
        raise ValueError('Rotation matrix produced a zero quaternion')
    return quaternion / norm


def quaternion_to_rotation_matrix(quaternion):
    """Convert a normalized XYZW quaternion to a 3x3 rotation matrix."""
    x, y, z, w = np.asarray(quaternion, dtype=np.float64)
    norm = float(np.linalg.norm([x, y, z, w]))
    if norm < 1e-12:
        raise ValueError('Cannot convert a zero quaternion')
    x, y, z, w = np.asarray([x, y, z, w]) / norm
    return np.array([
        [1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w),
         2.0 * (x * z + y * w)],
        [2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z),
         2.0 * (y * z - x * w)],
        [2.0 * (x * z - y * w), 2.0 * (y * z + x * w),
         1.0 - 2.0 * (x * x + y * y)],
    ], dtype=np.float64)


class ArucoPosePublisher(Node):
    """Estimate a configured marker pose using calibrated camera intrinsics."""

    def __init__(self):
        super().__init__('arm2_aruco_pose_publisher')

        self.declare_parameter(
            'image_topic', '/arm2/gripper_camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/arm2/gripper_camera/camera_info'
        )
        self.declare_parameter(
            'annotated_topic', '/arm2/gripper_camera/aruco_annotated'
        )
        self.declare_parameter(
            'pose_topic', '/arm2/gripper_camera/aruco_pose'
        )
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('output_frame_id', 'arm2/base_link')
        self.declare_parameter('marker_frame_id', 'arm2/container_marker')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('secondary_marker_id', -1)
        self.declare_parameter(
            'secondary_marker_frame_id', 'arm2/stack_target_marker'
        )
        self.declare_parameter(
            'secondary_pose_topic', '/arm2/gripper_camera/stack_target_pose'
        )
        self.declare_parameter(
            'additional_marker_ids',
            [1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 16],
        )
        self.declare_parameter(
            'additional_marker_frame_ids',
            [
                'arm2/container_marker_1',
                'arm2/container_marker_2',
                'arm2/container_marker_3',
                'arm2/container_marker_4',
                'arm2/container_marker_5',
                'arm2/container_marker_6',
                'arm2/container_marker_7',
                'arm2/container_marker_8',
                'arm2/trailer_marker_9',
                'arm2/trailer_marker_10',
                'arm2/destination_marker_11',
                'arm2/destination_marker_12',
                'arm2/destination_marker_13',
                'arm2/destination_marker_14',
                'arm2/destination_marker_15',
                'arm2/destination_marker_16',
            ],
        )
        self.declare_parameter('marker_size_m', 0.020)
        self.declare_parameter('dictionary', 'DICT_5X5_50')
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
        self.output_frame_id = str(
            self.get_parameter('output_frame_id').value
        )
        self.marker_frame_id = str(
            self.get_parameter('marker_frame_id').value
        )
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.secondary_marker_id = int(
            self.get_parameter('secondary_marker_id').value
        )
        self.secondary_marker_frame_id = str(
            self.get_parameter('secondary_marker_frame_id').value
        )
        additional_ids = [
            int(value)
            for value in self.get_parameter('additional_marker_ids').value
        ]
        additional_frames = [
            str(value)
            for value in self.get_parameter(
                'additional_marker_frame_ids'
            ).value
        ]
        if len(additional_ids) != len(additional_frames):
            raise ValueError(
                'additional_marker_ids and additional_marker_frame_ids '
                'must have the same length'
            )
        self.additional_markers = dict(zip(additional_ids, additional_frames))
        self.marker_size = float(
            self.get_parameter('marker_size_m').value
        )
        dictionary_name = str(self.get_parameter('dictionary').value)
        self.max_reprojection_error = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.publish_annotated = bool(
            self.get_parameter('publish_annotated').value
        )
        self.use_node_time_for_pose = bool(
            self.get_parameter('use_node_time_for_pose').value
        )

        if self.marker_size <= 0.0:
            raise ValueError('marker_size_m must be greater than zero')
        if self.max_reprojection_error <= 0.0:
            raise ValueError('max_reprojection_error_px must be greater than zero')
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')

        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            self.detector_parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, self.detector_parameters
            )
        else:
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create()
            )
            self.detector = None
        self.bridge = CvBridge()
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_frame = ''
        self.missing_info_warning_count = 0
        self.invalid_info_warning_count = 0
        self.last_detected_ids = None
        self.detected_once = False
        self.last_transform_warning_ns = 0

        half_size = self.marker_size * 0.5
        self.object_points = np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float64)

        self.pose_publisher = self.create_publisher(
            PoseStamped, self.pose_topic, 10
        )
        self.secondary_pose_publisher = None
        if self.secondary_marker_id >= 0:
            secondary_topic = str(
                self.get_parameter('secondary_pose_topic').value
            )
            self.secondary_pose_publisher = self.create_publisher(
                PoseStamped, secondary_topic, 10
            )
        self.additional_pose_publishers = {
            marker_id: self.create_publisher(
                PoseStamped,
                f'/arm2/gripper_camera/destination_{marker_id}_pose',
                10,
            )
            for marker_id in self.additional_markers
        }
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
            f'Detecting ArUco id={self.marker_id}, dictionary={dictionary_name}, '
            f'size={self.marker_size:g} m on {self.image_topic}'
        )
        self.get_logger().info(
            f'Publishing marker TF: {self.output_frame_id} -> '
            f'{self.marker_frame_id}'
        )
        if self.secondary_marker_id >= 0:
            self.get_logger().info(
                'Also detecting ArUco '
                f'id={self.secondary_marker_id}: camera -> '
                f'{self.secondary_marker_frame_id}'
            )
        for marker_id, frame_id in self.additional_markers.items():
            self.get_logger().info(
                f'Also detecting ArUco id={marker_id}: camera -> {frame_id}'
            )
        if self.use_node_time_for_pose:
            self.get_logger().info(
                'Marker pose timestamp source: detector node clock'
            )

    def on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            self.invalid_info_warning_count += 1
            if self.invalid_info_warning_count == 1 or (
                self.invalid_info_warning_count % 100 == 0
            ):
                self.get_logger().warning(
                    'Ignored CameraInfo with invalid focal length'
                )
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(message.d, dtype=np.float64)
        self.camera_info_frame = message.header.frame_id
        if self.invalid_info_warning_count > 0:
            self.get_logger().info('Received calibrated CameraInfo')
        self.invalid_info_warning_count = 0

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Failed to convert camera image: {exc}')
            return

        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(frame)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                frame,
                self.dictionary,
                parameters=self.detector_parameters,
            )
        detected_ids = (
            tuple(int(value) for value in ids.flatten())
            if ids is not None else ()
        )
        if detected_ids != self.last_detected_ids:
            if detected_ids:
                self.get_logger().info(f'Detected ArUco IDs: {detected_ids}')
            self.last_detected_ids = detected_ids

        selected_corners = None
        secondary_corners = None
        additional_corners = {}
        if ids is not None:
            for index, detected_id in enumerate(ids.flatten()):
                detected_id = int(detected_id)
                if detected_id == self.marker_id:
                    selected_corners = corners[index].reshape(4, 2)
                if detected_id == self.secondary_marker_id:
                    secondary_corners = corners[index].reshape(4, 2)
                if detected_id in self.additional_markers:
                    additional_corners[detected_id] = corners[index].reshape(
                        4, 2
                    )

        if selected_corners is not None and self.camera_matrix is None:
            self.missing_info_warning_count += 1
            if self.missing_info_warning_count == 1 or (
                self.missing_info_warning_count % 100 == 0
            ):
                self.get_logger().warning(
                    'Marker detected, but pose requires calibrated CameraInfo on '
                    f'{self.camera_info_topic}'
                )
        elif selected_corners is not None:
            pose = self.estimate_pose(selected_corners)
            if pose is not None:
                rotation_vector, translation_vector, reprojection_error = pose
                self.publish_pose(
                    message,
                    rotation_vector,
                    translation_vector,
                    reprojection_error,
                    self.marker_frame_id,
                    self.pose_publisher,
                )
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rotation_vector,
                    translation_vector,
                    self.marker_size * 0.5,
                )

        if secondary_corners is not None and self.camera_matrix is not None:
            pose = self.estimate_pose(secondary_corners)
            if pose is not None:
                rotation_vector, translation_vector, reprojection_error = pose
                self.publish_pose(
                    message,
                    rotation_vector,
                    translation_vector,
                    reprojection_error,
                    self.secondary_marker_frame_id,
                    self.secondary_pose_publisher,
                )
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rotation_vector,
                    translation_vector,
                    self.marker_size * 0.5,
                )

        if self.camera_matrix is not None:
            for marker_id, marker_corners in additional_corners.items():
                pose = self.estimate_pose(marker_corners)
                if pose is None:
                    continue
                rotation_vector, translation_vector, reprojection_error = pose
                self.publish_pose(
                    message,
                    rotation_vector,
                    translation_vector,
                    reprojection_error,
                    self.additional_markers[marker_id],
                    self.additional_pose_publishers[marker_id],
                )
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rotation_vector,
                    translation_vector,
                    self.marker_size * 0.5,
                )

        if self.publish_annotated:
            if ids is not None:
                cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            annotated = self.make_bgr8_image(frame, message)
            self.annotated_publisher.publish(annotated)

    @staticmethod
    def make_bgr8_image(frame, source_message):
        """Build an Image directly to avoid cv_bridge output type issues."""
        contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
        annotated = Image()
        annotated.header = source_message.header
        annotated.height = int(contiguous.shape[0])
        annotated.width = int(contiguous.shape[1])
        annotated.encoding = 'bgr8'
        annotated.is_bigendian = False
        annotated.step = annotated.width * 3
        annotated.data = contiguous.tobytes()
        return annotated

    def estimate_pose(self, image_points):
        success, rotation_vector, translation_vector = cv2.solvePnP(
            self.object_points,
            np.asarray(image_points, dtype=np.float64),
            self.camera_matrix,
            self.distortion,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not success or float(translation_vector[2, 0]) <= 0.0:
            return None

        projected, _ = cv2.projectPoints(
            self.object_points,
            rotation_vector,
            translation_vector,
            self.camera_matrix,
            self.distortion,
        )
        error = float(np.sqrt(np.mean(np.sum(
            (projected.reshape(4, 2) - image_points) ** 2,
            axis=1,
        ))))
        if error > self.max_reprojection_error:
            self.get_logger().warning(
                f'Rejected ArUco pose: reprojection error={error:.2f} px'
            )
            return None
        return rotation_vector, translation_vector, error

    def publish_pose(
        self,
        image_message,
        rotation_vector,
        translation_vector,
        reprojection_error,
        marker_frame_id,
        pose_publisher,
    ):
        camera_frame = (
            self.camera_frame_id
            or self.camera_info_frame
            or image_message.header.frame_id
        )
        if not camera_frame:
            self.get_logger().error(
                'Cannot publish ArUco pose because camera frame_id is empty'
            )
            return

        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        translation = translation_vector.reshape(3)

        try:
            camera_transform = self.tf_buffer.lookup_transform(
                self.output_frame_id, camera_frame, Time()
            )
        except TransformException as exc:
            now_ns = self.get_clock().now().nanoseconds
            if now_ns - self.last_transform_warning_ns >= 5_000_000_000:
                self.get_logger().warning(
                    f'Cannot transform detected marker from {camera_frame} '
                    f'to {self.output_frame_id}: {exc}'
                )
                self.last_transform_warning_ns = now_ns
            return

        camera_translation = np.array([
            camera_transform.transform.translation.x,
            camera_transform.transform.translation.y,
            camera_transform.transform.translation.z,
        ], dtype=np.float64)
        camera_rotation = quaternion_to_rotation_matrix([
            camera_transform.transform.rotation.x,
            camera_transform.transform.rotation.y,
            camera_transform.transform.rotation.z,
            camera_transform.transform.rotation.w,
        ])
        translation = camera_translation + camera_rotation @ translation
        rotation_matrix = camera_rotation @ rotation_matrix
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)

        transform = TransformStamped()
        transform.header.stamp = (
            self.get_clock().now().to_msg()
            if self.use_node_time_for_pose
            else image_message.header.stamp
        )
        transform.header.frame_id = self.output_frame_id
        transform.child_frame_id = marker_frame_id
        transform.transform.translation.x = float(translation[0])
        transform.transform.translation.y = float(translation[1])
        transform.transform.translation.z = float(translation[2])
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
        pose_publisher.publish(pose)

        if not self.detected_once:
            self.detected_once = True
            self.get_logger().info(
                f'ArUco acquired: z={translation[2]:.4f} m, '
                f'reprojection_error={reprojection_error:.2f} px'
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArucoPosePublisher()
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
