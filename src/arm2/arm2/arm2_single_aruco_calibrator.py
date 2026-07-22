"""Calibrate one camera from many views of one known-size ArUco marker."""

from pathlib import Path
import shutil

import cv2
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


def marker_object_points(marker_size):
    """Return four planar marker corners matching OpenCV ArUco ordering."""
    half = float(marker_size) * 0.5
    return np.array([
        [-half, half, 0.0],
        [half, half, 0.0],
        [half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float32)


def write_camera_yaml(path, camera_name, width, height, matrix, distortion):
    """Write calibration in camera_info_manager YAML format."""
    values_k = ', '.join(f'{value:.10g}' for value in matrix.reshape(-1))
    coefficients = np.zeros(5, dtype=np.float64)
    flat_distortion = np.asarray(distortion).reshape(-1)
    count = min(len(coefficients), len(flat_distortion))
    coefficients[:count] = flat_distortion[:count]
    values_d = ', '.join(f'{value:.10g}' for value in coefficients)
    projection = np.array([
        [matrix[0, 0], 0.0, matrix[0, 2], 0.0],
        [0.0, matrix[1, 1], matrix[1, 2], 0.0],
        [0.0, 0.0, 1.0, 0.0],
    ])
    values_p = ', '.join(f'{value:.10g}' for value in projection.reshape(-1))
    content = f"""image_width: {width}
image_height: {height}
camera_name: {camera_name}
camera_matrix:
  rows: 3
  cols: 3
  data: [{values_k}]
distortion_model: plumb_bob
distortion_coefficients:
  rows: 1
  cols: 5
  data: [{values_d}]
rectification_matrix:
  rows: 3
  cols: 3
  data: [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
projection_matrix:
  rows: 3
  cols: 4
  data: [{values_p}]
"""
    output = Path(path).expanduser()
    output.parent.mkdir(parents=True, exist_ok=True)
    if output.exists():
        backup = output.with_suffix(output.suffix + '.pre_single_aruco')
        shutil.copy2(output, backup)
    temporary = output.with_suffix(output.suffix + '.tmp')
    temporary.write_text(content, encoding='utf-8')
    temporary.replace(output)


class SingleArucoCalibrator(Node):
    """Collect diverse marker observations and write camera intrinsics."""

    def __init__(self):
        super().__init__('arm2_single_aruco_calibrator')
        self.declare_parameter('image_topic', '/arm2/gripper_camera/image_raw')
        self.declare_parameter('marker_id', 0)
        self.declare_parameter('marker_size_m', 0.026)
        self.declare_parameter('dictionary', 'DICT_5X5_50')
        self.declare_parameter('minimum_samples', 30)
        self.declare_parameter('camera_name', 'arm2_gripper_camera')
        self.declare_parameter(
            'output_file',
            '/home/rsj/poter_ws/config/arm2/arm2_gripper_camera_info.yaml',
        )

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.marker_id = int(self.get_parameter('marker_id').value)
        self.marker_size = float(self.get_parameter('marker_size_m').value)
        self.minimum_samples = int(self.get_parameter('minimum_samples').value)
        self.camera_name = str(self.get_parameter('camera_name').value)
        self.output_file = str(self.get_parameter('output_file').value)
        dictionary_name = str(self.get_parameter('dictionary').value)
        if self.marker_size <= 0.0:
            raise ValueError('marker_size_m must be positive')
        if self.minimum_samples < 15:
            raise ValueError('minimum_samples must be at least 15')
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
        self.samples = []
        self.current_corners = None
        self.image_size = None
        self.last_message = 'Show the 26 mm marker, then press SPACE'
        self.window_name = 'arm2 single ArUco camera calibration'
        self.create_subscription(
            Image, self.image_topic, self.on_image, qos_profile_sensor_data
        )
        self.get_logger().info(
            f'Waiting for ArUco id={self.marker_id} on {self.image_topic}; '
            'SPACE=sample, C=calibrate/save, Q=quit'
        )

    def on_image(self, message):
        try:
            frame = self.bridge.imgmsg_to_cv2(message, desired_encoding='bgr8')
        except CvBridgeError as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')
            return
        self.image_size = (int(frame.shape[1]), int(frame.shape[0]))
        if self.detector is not None:
            corners, ids, _rejected = self.detector.detectMarkers(frame)
        else:
            corners, ids, _rejected = cv2.aruco.detectMarkers(
                frame,
                self.dictionary,
                parameters=self.detector_parameters,
            )
        self.current_corners = None
        if ids is not None:
            for index, detected_id in enumerate(ids.flatten()):
                if int(detected_id) == self.marker_id:
                    self.current_corners = corners[index].reshape(4, 2)
                    cv2.aruco.drawDetectedMarkers(
                        frame, [corners[index]], ids[index:index + 1]
                    )
                    break

        color = (0, 255, 0) if self.current_corners is not None else (0, 0, 255)
        cv2.putText(
            frame,
            f'Samples: {len(self.samples)}/{self.minimum_samples}',
            (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2,
        )
        cv2.putText(
            frame, self.last_message, (12, 56),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1,
        )
        cv2.putText(
            frame, 'SPACE sample | C calibrate/save | Q quit', (12, 82),
            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1,
        )
        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1) & 0xff
        if key == ord(' '):
            self.add_sample()
        elif key in (ord('c'), ord('C')):
            self.calibrate_and_save()
        elif key in (ord('q'), ord('Q'), 27):
            rclpy.shutdown()

    def add_sample(self):
        if self.current_corners is None:
            self.last_message = 'REJECTED: marker id 0 is not detected'
            return
        candidate = self.current_corners.astype(np.float32).copy()
        if self.is_duplicate(candidate):
            self.last_message = 'REJECTED: move/tilt/resize marker more'
            return
        self.samples.append(candidate)
        self.last_message = f'Accepted sample {len(self.samples)}'
        self.get_logger().info(self.last_message)

    def is_duplicate(self, candidate):
        if not self.samples:
            return False
        center = np.mean(candidate, axis=0)
        area = abs(float(cv2.contourArea(candidate)))
        edge = candidate[1] - candidate[0]
        angle = float(np.degrees(np.arctan2(edge[1], edge[0])))
        for sample in self.samples:
            old_center = np.mean(sample, axis=0)
            old_area = abs(float(cv2.contourArea(sample)))
            old_edge = sample[1] - sample[0]
            old_angle = float(np.degrees(np.arctan2(old_edge[1], old_edge[0])))
            center_delta = float(np.linalg.norm(center - old_center))
            area_ratio = area / max(old_area, 1.0)
            angle_delta = abs((angle - old_angle + 180.0) % 360.0 - 180.0)
            if center_delta < 25.0 and 0.82 < area_ratio < 1.22 and angle_delta < 8.0:
                return True
        return False

    def calibrate_and_save(self):
        if len(self.samples) < self.minimum_samples:
            self.last_message = (
                f'Need {self.minimum_samples - len(self.samples)} more samples'
            )
            return
        if self.image_size is None:
            self.last_message = 'No image size available'
            return
        object_template = marker_object_points(self.marker_size)
        object_points = [object_template.copy() for _sample in self.samples]
        image_points = [sample.reshape(-1, 1, 2) for sample in self.samples]
        flags = cv2.CALIB_FIX_K3
        error, matrix, distortion, _rvecs, _tvecs = cv2.calibrateCamera(
            object_points, image_points, self.image_size, None, None,
            flags=flags,
        )
        if not np.isfinite(error) or error > 2.0:
            self.last_message = f'REJECTED: RMS error {error:.3f}px is too high'
            self.get_logger().error(self.last_message)
            return
        center = np.array([self.image_size[0] / 2.0, self.image_size[1] / 2.0])
        principal = np.array([matrix[0, 2], matrix[1, 2]])
        if float(np.linalg.norm(principal - center)) > 80.0:
            self.last_message = 'REJECTED: optical center is implausible'
            self.get_logger().error(
                f'{self.last_message}: cx={matrix[0, 2]:.1f}, '
                f'cy={matrix[1, 2]:.1f}'
            )
            return
        if np.max(np.abs(np.asarray(distortion).reshape(-1)[:2])) > 2.0:
            self.last_message = 'REJECTED: lens distortion is implausible'
            self.get_logger().error(self.last_message)
            return
        write_camera_yaml(
            self.output_file,
            self.camera_name,
            self.image_size[0],
            self.image_size[1],
            matrix,
            distortion,
        )
        self.last_message = f'SAVED RMS={error:.3f}px fx={matrix[0, 0]:.1f}'
        self.get_logger().info(
            f'{self.last_message} fy={matrix[1, 1]:.1f} -> {self.output_file}'
        )

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    """Run the single-marker calibration GUI."""
    rclpy.init(args=args)
    node = SingleArucoCalibrator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
