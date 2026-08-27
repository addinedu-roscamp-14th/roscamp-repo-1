"""Detect two target ArUco IDs independently across camera images."""

import math

import cv2
from cv_bridge import CvBridge, CvBridgeError
from geometry_msgs.msg import PoseStamped, TransformStamped
import numpy as np
from porter_interfaces.srv import ExecutePickPlace
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from tf2_ros import TransformBroadcaster


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
        raise ValueError('Rotation matrix produced a zero quaternion')
    return quaternion / norm


class DualArucoPosePublisher(Node):
    """Publish TF frames assigning one configured ID to pick and one to place."""

    def __init__(self):
        super().__init__('dual_aruco_pose_publisher')
        self.declare_parameter(
            'image_topic', '/arm/gripper_camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/arm/gripper_camera/camera_info'
        )
        self.declare_parameter(
            'annotated_topic',
            '/arm/gripper_camera/pick_place_aruco_annotated',
        )
        self.declare_parameter('camera_frame_id', '')
        self.declare_parameter('pick_marker_id', 1)
        self.declare_parameter('place_marker_id', 0)
        self.declare_parameter('pick_marker_frame', 'arm/pick_marker')
        self.declare_parameter('place_marker_frame', 'arm/place_marker')
        self.declare_parameter('marker_size_m', 0.015)
        self.declare_parameter('dictionary', 'DICT_5X5_50')
        self.declare_parameter('max_reprojection_error_px', 3.0)
        self.declare_parameter('use_node_time_for_pose', True)
        self.declare_parameter(
            'scan_marker_ids', list(range(9)) + list(range(18, 24))
        )

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
        pick_id = int(self.get_parameter('pick_marker_id').value)
        place_id = int(self.get_parameter('place_marker_id').value)
        if pick_id == place_id:
            raise ValueError('pick_marker_id and place_marker_id must differ')
        self.marker_frames = {
            pick_id: str(self.get_parameter('pick_marker_frame').value),
            place_id: str(self.get_parameter('place_marker_frame').value),
        }
        self.scan_marker_frames = {
            int(marker_id): f'arm/marker_{int(marker_id)}'
            for marker_id in self.get_parameter('scan_marker_ids').value
        }
        self.marker_size = float(
            self.get_parameter('marker_size_m').value
        )
        self.max_reprojection_error = float(
            self.get_parameter('max_reprojection_error_px').value
        )
        self.use_node_time_for_pose = bool(
            self.get_parameter('use_node_time_for_pose').value
        )
        dictionary_name = str(self.get_parameter('dictionary').value)
        if self.marker_size <= 0.0:
            raise ValueError('marker_size_m must be positive')
        if self.max_reprojection_error <= 0.0:
            raise ValueError('max_reprojection_error_px must be positive')
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')

        dictionary_id = getattr(cv2.aruco, dictionary_name)
        self.dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
        if hasattr(cv2.aruco, 'ArucoDetector'):
            parameters = cv2.aruco.DetectorParameters()
            self.detector = cv2.aruco.ArucoDetector(
                self.dictionary, parameters
            )
            self.detector_parameters = parameters
        else:
            self.detector = None
            self.detector_parameters = (
                cv2.aruco.DetectorParameters_create()
            )

        half_size = self.marker_size * 0.5
        self.object_points = np.array([
            [-half_size, half_size, 0.0],
            [half_size, half_size, 0.0],
            [half_size, -half_size, 0.0],
            [-half_size, -half_size, 0.0],
        ], dtype=np.float64)
        self.bridge = CvBridge()
        self.tf_broadcaster = TransformBroadcaster(self)
        self.camera_matrix = None
        self.distortion = None
        self.camera_info_frame = ''
        self.last_detected_ids = None
        self.rejection_counts = {
            marker_id: 0
            for marker_id in set(self.marker_frames) | set(self.scan_marker_frames)
        }
        self.pose_publishers = {
            str(self.get_parameter('pick_marker_frame').value): (
                self.create_publisher(
                    PoseStamped, '/arm/gripper_camera/pick_aruco_pose', 10
                )
            ),
            str(self.get_parameter('place_marker_frame').value): (
                self.create_publisher(
                    PoseStamped, '/arm/gripper_camera/place_aruco_pose', 10
                )
            ),
        }
        self.annotated_publisher = self.create_publisher(
            Image, self.annotated_topic, 10
        )
        self.create_subscription(
            CameraInfo, self.camera_info_topic, self.on_camera_info, 10
        )
        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.create_service(
            ExecutePickPlace,
            '/arm/pick_place/configure_targets',
            self.configure_targets,
        )
        self.get_logger().info(
            'Detecting pick/place ArUco markers: '
            + ', '.join(
                f'id={marker_id}->{frame}'
                for marker_id, frame in self.marker_frames.items()
            )
        )

    def configure_targets(self, request, response):
        """Atomically switch the two marker IDs used by the next operation."""
        if bool(getattr(self, 'detection_enabled', False)):
            response.accepted = False
            response.message = 'cannot change targets during active detection'
            return response
        pick_id = int(request.pick_id)
        place_id = int(request.place_id)
        if not 0 <= pick_id <= 49 or not 0 <= place_id <= 49:
            response.accepted = False
            response.message = 'pick_id/place_id must be within 0..49'
            return response
        if pick_id == place_id:
            response.accepted = False
            response.message = 'pick_id and place_id must be different'
            return response
        frames = tuple(self.marker_frames.values())
        self.marker_frames = {
            pick_id: frames[0],
            place_id: frames[1],
        }
        self.rejection_counts = {
            marker_id: 0
            for marker_id in {pick_id, place_id} | set(
                getattr(self, 'scan_marker_frames', {})
            )
        }
        self.last_detected_ids = None
        response.accepted = True
        response.message = (
            f'ArUco targets configured: pick_id={pick_id}, '
            f'place_id={place_id}'
        )
        self.get_logger().info(response.message)
        return response

    def on_camera_info(self, message):
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        if matrix[0, 0] <= 0.0 or matrix[1, 1] <= 0.0:
            return
        self.camera_matrix = matrix
        self.distortion = np.asarray(message.d, dtype=np.float64)
        self.camera_info_frame = message.header.frame_id

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
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
            self.get_logger().info(f'Detected ArUco IDs: {detected_ids}')
            self.last_detected_ids = detected_ids

        if ids is not None:
            for index, raw_id in enumerate(ids.flatten()):
                marker_id = int(raw_id)
                if (
                    marker_id not in self.marker_frames
                    and marker_id not in self.scan_marker_frames
                ):
                    continue
                image_points = corners[index].reshape(4, 2)
                pose = self.estimate_pose(marker_id, image_points)
                if pose is None:
                    continue
                rotation_vector, translation_vector = pose
                self.publish_pose(
                    marker_id,
                    message,
                    rotation_vector,
                    translation_vector,
                )
                cv2.drawFrameAxes(
                    frame,
                    self.camera_matrix,
                    self.distortion,
                    rotation_vector,
                    translation_vector,
                    self.marker_size * 0.5,
                )
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
        self.annotated_publisher.publish(self.make_image(frame, message))

    def estimate_pose(self, marker_id, image_points):
        if self.camera_matrix is None:
            return None
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
            self.rejection_counts[marker_id] += 1
            if self.rejection_counts[marker_id] in (1, 20, 100):
                self.get_logger().warning(
                    f'Rejected ArUco id={marker_id}: '
                    f'reprojection error={error:.2f}px'
                )
            return None
        self.rejection_counts[marker_id] = 0
        return rotation_vector, translation_vector

    def publish_pose(
        self, marker_id, image_message, rotation_vector, translation_vector
    ):
        camera_frame = (
            self.camera_frame_id
            or self.camera_info_frame
            or image_message.header.frame_id
        )
        if not camera_frame:
            return
        rotation_matrix, _ = cv2.Rodrigues(rotation_vector)
        quaternion = rotation_matrix_to_quaternion(rotation_matrix)
        translation = translation_vector.reshape(3)
        stamp = (
            self.get_clock().now().to_msg()
            if self.use_node_time_for_pose
            else image_message.header.stamp
        )
        frames = []
        if marker_id in self.marker_frames:
            frames.append(self.marker_frames[marker_id])
        if marker_id in self.scan_marker_frames:
            frames.append(self.scan_marker_frames[marker_id])
        for frame in dict.fromkeys(frames):
            transform = TransformStamped()
            transform.header.stamp = stamp
            transform.header.frame_id = camera_frame
            transform.child_frame_id = frame
            transform.transform.translation.x = float(translation[0])
            transform.transform.translation.y = float(translation[1])
            transform.transform.translation.z = float(translation[2])
            transform.transform.rotation.x = float(quaternion[0])
            transform.transform.rotation.y = float(quaternion[1])
            transform.transform.rotation.z = float(quaternion[2])
            transform.transform.rotation.w = float(quaternion[3])
            self.tf_broadcaster.sendTransform(transform)
            if frame in self.pose_publishers:
                pose = PoseStamped()
                pose.header = transform.header
                pose.pose.position.x = transform.transform.translation.x
                pose.pose.position.y = transform.transform.translation.y
                pose.pose.position.z = transform.transform.translation.z
                pose.pose.orientation = transform.transform.rotation
                self.pose_publishers[frame].publish(pose)

    @staticmethod
    def make_image(frame, source):
        contiguous = np.ascontiguousarray(frame, dtype=np.uint8)
        message = Image()
        message.header = source.header
        message.height = int(contiguous.shape[0])
        message.width = int(contiguous.shape[1])
        message.encoding = 'bgr8'
        message.is_bigendian = False
        message.step = message.width * 3
        message.data = contiguous.tobytes()
        return message


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DualArucoPosePublisher()
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
