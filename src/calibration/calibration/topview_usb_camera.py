#!/usr/bin/env python3

import subprocess
import time
from pathlib import Path

import cv2
import yaml

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image


class TopviewUsbCamera(Node):
    def __init__(self):
        super().__init__('topview_usb_camera')

        self.declare_parameter('camera_name', 'ABKO APC925')
        self.declare_parameter('camera_device', 'auto')
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('image_topic', '/top_camera/image_raw')
        self.declare_parameter('camera_info_topic', '/top_camera/camera_info')
        self.declare_parameter(
            'camera_info_yaml',
            'config/top_camera/camera_info.yaml',
        )
        self.declare_parameter('frame_id', 'top_camera_optical_frame')
        self.declare_parameter('display', False)
        self.declare_parameter('use_gstreamer', True)
        self.declare_parameter('display_width', 640)
        self.declare_parameter('display_height', 480)

        self.camera_name = str(self.get_parameter('camera_name').value)
        self.camera_device = str(self.get_parameter('camera_device').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = int(self.get_parameter('fps').value)
        self.image_topic = str(self.get_parameter('image_topic').value)
        self.camera_info_topic = str(self.get_parameter('camera_info_topic').value)
        self.camera_info_yaml = self.resolve_path(
            str(self.get_parameter('camera_info_yaml').value)
        )
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.display = bool(self.get_parameter('display').value)
        self.use_gstreamer = bool(self.get_parameter('use_gstreamer').value)
        self.display_width = int(self.get_parameter('display_width').value)
        self.display_height = int(self.get_parameter('display_height').value)

        if self.camera_device == 'auto':
            self.camera_device = self.detect_camera_device(self.camera_name)

        if not self.camera_device:
            raise RuntimeError(f'Camera not found: {self.camera_name}')

        self.bridge = CvBridge()
        self.image_pub = self.create_publisher(Image, self.image_topic, 10)
        self.info_pub = self.create_publisher(CameraInfo, self.camera_info_topic, 10)
        self.camera_info = self.load_camera_info()

        self.capture = self.open_capture()

        if not self.capture.isOpened():
            raise RuntimeError(f'Failed to open camera: {self.camera_device}')

        interval = 1.0 / max(float(self.fps), 1.0)
        self.timer = self.create_timer(interval, self.publish_frame)
        self.last_warn_time = 0.0

        if self.display:
            cv2.namedWindow('topview_usb_camera', cv2.WINDOW_NORMAL)
            cv2.resizeWindow('topview_usb_camera', self.display_width, self.display_height)
            cv2.moveWindow('topview_usb_camera', 700, 0)

        self.get_logger().info(
            f'Publishing topview camera {self.camera_device} '
            f'({self.width}x{self.height}@{self.fps}) to {self.image_topic}'
        )

    def resolve_path(self, configured_path):
        path = Path(configured_path).expanduser()
        if path.is_absolute():
            return path
        return Path.home() / 'poter_ws' / path

    def load_camera_info(self):
        if not self.camera_info_yaml.is_file():
            self.get_logger().warn(
                f'Topview camera calibration not found: {self.camera_info_yaml}. '
                'Publishing uncalibrated CameraInfo.'
            )
            return None

        try:
            with open(self.camera_info_yaml, 'r') as stream:
                data = yaml.safe_load(stream) or {}

            info = CameraInfo()
            info.width = int(data['image_width'])
            info.height = int(data['image_height'])
            info.distortion_model = str(data.get('distortion_model', 'plumb_bob'))
            info.d = [float(value) for value in data['distortion_coefficients']['data']]
            info.k = [float(value) for value in data['camera_matrix']['data']]
            info.r = [float(value) for value in data['rectification_matrix']['data']]
            info.p = [float(value) for value in data['projection_matrix']['data']]
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as exc:
            raise RuntimeError(
                f'Invalid topview camera calibration {self.camera_info_yaml}: {exc}'
            ) from exc

        if info.width != self.width or info.height != self.height:
            raise RuntimeError(
                'Topview calibration resolution '
                f'{info.width}x{info.height} does not match camera '
                f'{self.width}x{self.height}'
            )

        self.get_logger().info(
            f'Loaded topview camera calibration: {self.camera_info_yaml}'
        )
        return info

    def open_capture(self):
        if self.use_gstreamer:
            pipeline = (
                f'v4l2src device={self.camera_device} do-timestamp=true ! '
                f'image/jpeg,width={self.width},height={self.height},framerate={self.fps}/1 ! '
                'queue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! '
                'jpegdec ! '
                'videoconvert ! '
                'video/x-raw,format=BGR ! '
                'appsink sync=false max-buffers=1 drop=true'
            )
            self.get_logger().info(f'Opening GStreamer pipeline:\n{pipeline}')
            capture = cv2.VideoCapture(pipeline, cv2.CAP_GSTREAMER)
            if capture.isOpened():
                return capture
            self.get_logger().warn('GStreamer open failed; falling back to V4L2')

        capture = cv2.VideoCapture(self.camera_device, cv2.CAP_V4L2)
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        return capture

    def detect_camera_device(self, camera_name):
        try:
            result = subprocess.run(
                ['v4l2-ctl', '--list-devices'],
                check=False,
                capture_output=True,
                text=True,
            )
        except FileNotFoundError:
            self.get_logger().warn('v4l2-ctl not found; fallback to /dev/video0')
            return '/dev/video0'

        lines = result.stdout.splitlines()
        found = False
        for line in lines:
            stripped = line.strip()
            if camera_name in line:
                found = True
                continue

            if found and stripped.startswith('/dev/video'):
                return stripped

            if line and not line.startswith((' ', '\t')):
                found = False

        return ''

    def publish_frame(self):
        ok, frame = self.capture.read()
        if not ok or frame is None:
            now = time.time()
            if now - self.last_warn_time > 2.0:
                self.get_logger().warn(f'Failed to read frame from {self.camera_device}')
                self.last_warn_time = now
            return

        stamp = self.get_clock().now().to_msg()

        image_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        image_msg.header.stamp = stamp
        image_msg.header.frame_id = self.frame_id

        info_msg = CameraInfo()
        if self.camera_info is not None:
            info_msg = self.camera_info
        info_msg.header.stamp = stamp
        info_msg.header.frame_id = self.frame_id
        info_msg.width = frame.shape[1]
        info_msg.height = frame.shape[0]

        self.image_pub.publish(image_msg)
        self.info_pub.publish(info_msg)

        if self.display:
            preview = cv2.resize(
                frame,
                (self.display_width, self.display_height),
                interpolation=cv2.INTER_AREA
            )
            cv2.imshow('topview_usb_camera', preview)
            cv2.waitKey(1)

    def destroy_node(self):
        if hasattr(self, 'capture') and self.capture is not None:
            self.capture.release()
        try:
            cv2.destroyWindow('topview_usb_camera')
        except cv2.error:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = TopviewUsbCamera()

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
