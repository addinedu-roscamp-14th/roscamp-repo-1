"""Monitor rectified ArUco center repeatability against a fixed reference."""

import json
import math

import cv2

from cv_bridge import CvBridge, CvBridgeError

import numpy as np

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import CameraInfo, Image

from std_msgs.msg import String


def classify_pixel_delta(delta_px):
    """Classify restart repeatability using the project thresholds."""
    if delta_px <= 2.0:
        return 'stable'
    if delta_px <= 5.0:
        return 'warning'
    return 'unstable'


def summarize_centers(points, reference, marker_size_mm):
    """Return robust center, dispersion, reference error, and scale."""
    values = np.asarray(points, dtype=np.float64)
    if values.ndim != 2 or values.shape[1] != 3 or len(values) < 2:
        raise ValueError('at least two [u, v, side_px] samples are required')
    center_values = values[:, :2]
    center = np.median(center_values, axis=0)
    offset = center - np.asarray(reference, dtype=np.float64)
    delta = float(np.linalg.norm(offset))
    radial = np.linalg.norm(center_values - center, axis=1)
    side_px = float(np.median(values[:, 2]))
    mm_per_px = marker_size_mm / side_px if side_px > 0.0 else None
    return {
        'median_u_px': float(center[0]),
        'median_v_px': float(center[1]),
        'std_u_px': float(np.std(center_values[:, 0], ddof=1)),
        'std_v_px': float(np.std(center_values[:, 1], ddof=1)),
        'radial_rms_px': float(math.sqrt(np.mean(radial ** 2))),
        'max_from_median_px': float(np.max(radial)),
        'delta_u_px': float(offset[0]),
        'delta_v_px': float(offset[1]),
        'delta_px': delta,
        'status': classify_pixel_delta(delta),
        'median_marker_side_px': side_px,
        'mm_per_px': mm_per_px,
        'delta_mm': delta * mm_per_px if mm_per_px is not None else None,
    }


class CameraRepeatabilityMonitor(Node):
    """Measure batches of rectified marker centers without altering images."""

    def __init__(self):
        """Configure marker detection, reference values, and ROS interfaces."""
        super().__init__('arm2_camera_repeatability_monitor')
        self.declare_parameter(
            'image_topic', '/arm2/gripper_camera/image_raw'
        )
        self.declare_parameter(
            'camera_info_topic', '/arm2/gripper_camera/camera_info'
        )
        self.declare_parameter(
            'result_topic', '/arm2/gripper_camera/repeatability'
        )
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_mm', 26.0)
        self.declare_parameter('sample_count', 200)
        self.declare_parameter('reference_u_px', 300.350)
        self.declare_parameter('reference_v_px', 293.686)
        self.declare_parameter('dictionary', 'DICT_5X5_50')

        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size_mm = float(
            self.get_parameter('marker_size_mm').value
        )
        self.sample_count = int(self.get_parameter('sample_count').value)
        self.reference = np.array([
            float(self.get_parameter('reference_u_px').value),
            float(self.get_parameter('reference_v_px').value),
        ])
        if self.marker_size_mm <= 0.0:
            raise ValueError('marker_size_mm must be positive')
        if self.sample_count < 2:
            raise ValueError('sample_count must be at least two')

        dictionary_name = str(self.get_parameter('dictionary').value)
        if not hasattr(cv2.aruco, dictionary_name):
            raise ValueError(f'Unknown ArUco dictionary: {dictionary_name}')
        dictionary = cv2.aruco.getPredefinedDictionary(
            getattr(cv2.aruco, dictionary_name)
        )
        parameters = cv2.aruco.DetectorParameters()
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        self.bridge = CvBridge()
        self.camera_matrix = None
        self.distortion = None
        self.projection = None
        self.camera_signature = None
        self.samples = []

        result_topic = str(self.get_parameter('result_topic').value)
        self.publisher = self.create_publisher(String, result_topic, 10)
        self.create_subscription(
            CameraInfo,
            str(self.get_parameter('camera_info_topic').value),
            self.on_camera_info,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self.on_image,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f'Reference center fixed at ({self.reference[0]:.3f}, '
            f'{self.reference[1]:.3f}) px; collecting '
            f'{self.sample_count} samples per result'
        )

    def on_camera_info(self, message):
        """Store calibration and reset a batch if calibration changes."""
        matrix = np.asarray(message.k, dtype=np.float64).reshape(3, 3)
        distortion = np.asarray(message.d, dtype=np.float64)
        projection = np.asarray(
            message.p, dtype=np.float64
        ).reshape(3, 4)[:, :3]
        signature = (
            int(message.width),
            int(message.height),
            tuple(matrix.flatten()),
            tuple(distortion),
            tuple(projection.flatten()),
        )
        if (
            self.camera_signature is not None
            and signature != self.camera_signature
        ):
            self.get_logger().warning(
                'CameraInfo changed; discarded the current measurement batch'
            )
            self.samples.clear()
        self.camera_signature = signature
        self.camera_matrix = matrix
        self.distortion = distortion
        self.projection = projection

    def on_image(self, message):
        """Add one detected center and publish each completed batch."""
        if self.camera_matrix is None:
            return
        try:
            frame = self.bridge.imgmsg_to_cv2(
                message, desired_encoding='bgr8'
            )
        except CvBridgeError as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        corners, ids, _rejected = self.detector.detectMarkers(frame)
        if ids is None:
            return
        for marker_corners, detected_id in zip(corners, ids.flatten()):
            if int(detected_id) != self.marker_id:
                continue
            raw = np.asarray(
                marker_corners, dtype=np.float64
            ).reshape(4, 2)
            rectified = cv2.undistortPoints(
                raw.reshape(-1, 1, 2),
                self.camera_matrix,
                self.distortion,
                P=self.projection,
            ).reshape(4, 2)
            center = np.mean(rectified, axis=0)
            side_px = float(np.mean([
                np.linalg.norm(
                    rectified[(index + 1) % 4] - rectified[index]
                )
                for index in range(4)
            ]))
            self.samples.append([
                float(center[0]), float(center[1]), side_px
            ])
            break
        if len(self.samples) < self.sample_count:
            return

        result = summarize_centers(
            self.samples[:self.sample_count],
            self.reference,
            self.marker_size_mm,
        )
        result['sample_count'] = self.sample_count
        result['reference_u_px'] = float(self.reference[0])
        result['reference_v_px'] = float(self.reference[1])
        message_out = String()
        message_out.data = json.dumps(result, ensure_ascii=False)
        self.publisher.publish(message_out)
        log = self.get_logger().info
        if result['status'] == 'warning':
            log = self.get_logger().warning
        elif result['status'] == 'unstable':
            log = self.get_logger().error
        log(
            f"camera repeatability={result['status']}: "
            f"delta={result['delta_px']:.3f}px/"
            f"{result['delta_mm']:.3f}mm, "
            f"center=({result['median_u_px']:.3f}, "
            f"{result['median_v_px']:.3f}), "
            f"RMS={result['radial_rms_px']:.3f}px"
        )
        self.samples.clear()


def main(args=None):
    """Run the repeatability monitor."""
    rclpy.init(args=args)
    node = None
    try:
        node = CameraRepeatabilityMonitor()
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
