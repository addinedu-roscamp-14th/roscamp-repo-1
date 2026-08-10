"""Serve camera and composed SLAM views as low-latency MJPEG."""

import asyncio
from importlib.resources import files
import json
import threading
import time

import cv2
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException, SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener

from .frame_store import LatestFrameStore, LatestJpegStore
from .slam_renderer import (
    draw_laser_scan,
    draw_robot_pose,
    quaternion_to_yaw,
    render_occupancy_grid,
)


BOUNDARY = b'frame'


def _web_dependencies():
    """Load optional web dependencies with an actionable error message."""
    try:
        from fastapi import FastAPI
        from fastapi.responses import Response, StreamingResponse
        import uvicorn
    except ImportError as exc:
        raise RuntimeError(
            'FastAPI dependencies are missing. Install: '
            'sudo apt install python3-fastapi python3-uvicorn'
        ) from exc
    return (
        FastAPI,
        Response,
        StreamingResponse,
        uvicorn,
    )


class DashboardStreamNode(Node):
    """Receive images quickly and encode only the most recent frame."""

    def __init__(self):
        super().__init__('dashboard_stream_node')
        self.declare_parameter(
            'input_topic', '/central/yolo/image_annotated'
        )
        self.declare_parameter(
            'detection_topic', '/central/yolo/detections'
        )
        self.declare_parameter('host', '0.0.0.0')
        self.declare_parameter('port', 8000)
        self.declare_parameter('web_fps', 15.0)
        self.declare_parameter('jpeg_quality', 70)
        self.declare_parameter('output_width', 640)
        self.declare_parameter('output_height', 480)
        self.declare_parameter('stale_timeout_sec', 2.0)
        self.declare_parameter('slam_map_topic', '/map')
        self.declare_parameter('slam_base_frame', 'base_footprint')
        self.declare_parameter('slam_scan_topic', '/scan')
        self.declare_parameter('slam_pose_topic', '')
        self.declare_parameter('slam_enable_scan', True)
        self.declare_parameter('slam_scan_max_age_sec', 0.5)
        self.declare_parameter('slam_live_fps', 15.0)
        self.declare_parameter('slam_output_width', 720)
        self.declare_parameter('slam_output_height', 720)

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.detection_topic = str(
            self.get_parameter('detection_topic').value
        )
        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.web_fps = float(self.get_parameter('web_fps').value)
        self.jpeg_quality = int(
            self.get_parameter('jpeg_quality').value
        )
        self.output_width = int(
            self.get_parameter('output_width').value
        )
        self.output_height = int(
            self.get_parameter('output_height').value
        )
        self.stale_timeout = float(
            self.get_parameter('stale_timeout_sec').value
        )
        self.slam_map_topic = str(
            self.get_parameter('slam_map_topic').value
        )
        self.slam_base_frame = str(
            self.get_parameter('slam_base_frame').value
        )
        self.slam_scan_topic = str(
            self.get_parameter('slam_scan_topic').value
        )
        self.slam_pose_topic = str(
            self.get_parameter('slam_pose_topic').value
        )
        self.slam_enable_scan = bool(
            self.get_parameter('slam_enable_scan').value
        )
        self.slam_scan_max_age = float(
            self.get_parameter('slam_scan_max_age_sec').value
        )
        self.slam_live_fps = float(
            self.get_parameter('slam_live_fps').value
        )
        self.slam_output_width = int(
            self.get_parameter('slam_output_width').value
        )
        self.slam_output_height = int(
            self.get_parameter('slam_output_height').value
        )
        self._validate_parameters()

        self.bridge = CvBridge()
        self.frames = LatestFrameStore()
        self.jpegs = LatestJpegStore()
        self.slam_jpegs = LatestJpegStore()
        self.maps = LatestFrameStore()
        self.scans = LatestFrameStore()
        self.poses = LatestFrameStore()
        self.detection_lock = threading.Lock()
        self.latest_detection = None
        self.detection_received_at = 0.0
        self.detection_input_count = 0
        self.detection_last_error = ''
        self.stop_event = threading.Event()
        self.last_error = ''
        self.slam_last_error = ''
        self.slam_tf_error = ''
        self.slam_map_frame = ''
        self.slam_robot_visible = False
        self.slam_scan_points = 0
        self.slam_scan_tf_error = ''
        self.slam_map_image = None
        self.slam_map_png = None
        self.slam_map_layout = None
        self.slam_map_version = 0
        self.slam_map_encoded_at = 0.0
        self.slam_map_encode_duration_ms = 0.0
        self.slam_state_lock = threading.Lock()
        self.tf_buffer = None
        self.tf_listener = None
        if not self.slam_pose_topic or self.slam_enable_scan:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
        self.encoder_thread = threading.Thread(
            target=self._encode_frames,
            name='dashboard-jpeg-encoder',
            daemon=True,
        )
        self.slam_encoder_thread = threading.Thread(
            target=self._encode_slam_maps,
            name='dashboard-slam-encoder',
            daemon=True,
        )
        self.slam_stream_thread = threading.Thread(
            target=self._encode_slam_frames,
            name='dashboard-slam-jpeg-encoder',
            daemon=True,
        )

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.subscription = self.create_subscription(
            Image,
            self.input_topic,
            self._on_image,
            image_qos,
        )
        self.detection_subscription = self.create_subscription(
            String,
            self.detection_topic,
            self._on_detection,
            image_qos,
        )
        map_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_subscription = self.create_subscription(
            OccupancyGrid,
            self.slam_map_topic,
            self._on_map,
            map_qos,
        )
        scan_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pose_subscription = None
        if self.slam_pose_topic:
            self.pose_subscription = self.create_subscription(
                PoseWithCovarianceStamped,
                self.slam_pose_topic,
                self._on_pose,
                scan_qos,
            )
        self.scan_subscription = None
        if self.slam_enable_scan:
            self.scan_subscription = self.create_subscription(
                LaserScan,
                self.slam_scan_topic,
                self._on_scan,
                scan_qos,
            )
        self.encoder_thread.start()
        self.slam_encoder_thread.start()
        self.slam_stream_thread.start()
        self.get_logger().info(
            f'Dashboard stream input={self.input_topic}, '
            f'detections={self.detection_topic}, '
            f'web={self.host}:{self.port}, max_fps={self.web_fps:.1f}, '
            f'jpeg_quality={self.jpeg_quality}'
        )
        self.get_logger().info(
            f'SLAM stream input={self.slam_map_topic}, '
            f'pose={self.slam_pose_topic or "TF"}, '
            f'scan={self.slam_scan_topic if self.slam_enable_scan else "off"},'
            ' '
            f'base_frame={self.slam_base_frame}, '
            f'mjpeg_fps={self.slam_live_fps:.1f}'
        )

    def _validate_parameters(self):
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be within 1..65535')
        if self.web_fps <= 0.0 or self.web_fps > 60.0:
            raise ValueError('web_fps must be within (0, 60]')
        if not 1 <= self.jpeg_quality <= 100:
            raise ValueError('jpeg_quality must be within 1..100')
        if self.output_width < 0 or self.output_height < 0:
            raise ValueError('output dimensions must be non-negative')
        if self.stale_timeout <= 0.0:
            raise ValueError('stale_timeout_sec must be positive')
        if self.slam_live_fps <= 0.0 or self.slam_live_fps > 30.0:
            raise ValueError('slam_live_fps must be within (0, 30]')
        if self.slam_output_width <= 0 or self.slam_output_height <= 0:
            raise ValueError('SLAM output dimensions must be positive')
        if self.slam_scan_max_age <= 0.0:
            raise ValueError('slam_scan_max_age_sec must be positive')

    def _on_image(self, message):
        self.frames.put(message)

    def _on_detection(self, message):
        try:
            payload = json.loads(message.data)
            if not isinstance(payload, dict):
                raise ValueError('detection payload must be a JSON object')
        except (json.JSONDecodeError, ValueError) as exc:
            with self.detection_lock:
                self.detection_last_error = str(exc)
            return

        with self.detection_lock:
            self.latest_detection = payload
            self.detection_received_at = time.monotonic()
            self.detection_input_count += 1
            self.detection_last_error = ''

    def _on_map(self, message):
        self.maps.put(message)

    def _on_scan(self, message):
        self.scans.put(message)

    def _on_pose(self, message):
        self.poses.put(message)

    def _encode_frames(self):
        interval = 1.0 / self.web_fps
        previous_sequence = 0
        next_encode_at = 0.0

        while not self.stop_event.is_set():
            snapshot = self.frames.wait_for_new(
                previous_sequence,
                timeout=0.2,
            )
            if snapshot is None:
                continue

            remaining = next_encode_at - time.monotonic()
            if remaining > 0.0 and self.stop_event.wait(remaining):
                break

            # A newer frame may have arrived while enforcing the FPS limit.
            snapshot = self.frames.latest() or snapshot
            started = time.monotonic()
            try:
                frame = self.bridge.imgmsg_to_cv2(
                    snapshot.message,
                    desired_encoding='bgr8',
                )
                if self.output_width > 0 and self.output_height > 0:
                    if (
                        frame.shape[1] != self.output_width
                        or frame.shape[0] != self.output_height
                    ):
                        frame = cv2.resize(
                            frame,
                            (self.output_width, self.output_height),
                            interpolation=cv2.INTER_AREA,
                        )
                success, encoded = cv2.imencode(
                    '.jpg',
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not success:
                    raise RuntimeError('cv2.imencode returned false')
            except Exception as exc:
                error = str(exc)
                if error != self.last_error:
                    self.get_logger().error(
                        f'Failed to encode dashboard frame: {error}'
                    )
                    self.last_error = error
                previous_sequence = snapshot.sequence
                next_encode_at = time.monotonic() + interval
                continue

            elapsed_ms = (time.monotonic() - started) * 1000.0
            self.jpegs.put(
                encoded.tobytes(),
                snapshot.sequence,
                elapsed_ms,
            )
            self.last_error = ''
            previous_sequence = snapshot.sequence
            next_encode_at = started + interval

    def _encode_slam_maps(self):
        previous_sequence = 0
        while not self.stop_event.is_set():
            snapshot = self.maps.wait_for_new(
                previous_sequence,
                timeout=0.2,
            )
            if snapshot is None:
                continue
            started = time.monotonic()
            try:
                image, layout = render_occupancy_grid(
                    snapshot.message,
                    self.slam_output_width,
                    self.slam_output_height,
                )
                map_frame = snapshot.message.header.frame_id or 'map'
                success, encoded = cv2.imencode(
                    '.png',
                    image,
                    [cv2.IMWRITE_PNG_COMPRESSION, 3],
                )
                if not success:
                    raise RuntimeError('SLAM PNG encoding returned false')
            except Exception as exc:
                error = str(exc)
                with self.slam_state_lock:
                    previous_error = self.slam_last_error
                    self.slam_last_error = error
                if error != previous_error:
                    self.get_logger().error(
                        f'Failed to encode SLAM map: {error}'
                    )
                previous_sequence = snapshot.sequence
                continue

            elapsed_ms = (time.monotonic() - started) * 1000.0
            with self.slam_state_lock:
                self.slam_last_error = ''
                self.slam_map_frame = map_frame
                self.slam_map_image = image
                self.slam_map_png = encoded.tobytes()
                self.slam_map_layout = layout
                self.slam_map_version = snapshot.sequence
                self.slam_map_encoded_at = time.monotonic()
                self.slam_map_encode_duration_ms = elapsed_ms
            previous_sequence = snapshot.sequence

    def _encode_slam_frames(self):
        interval = 1.0 / self.slam_live_fps
        while not self.stop_event.is_set():
            started = time.monotonic()
            with self.slam_state_lock:
                map_image = self.slam_map_image
                layout = self.slam_map_layout
                map_frame = self.slam_map_frame
                map_version = self.slam_map_version
            if map_image is None or layout is None:
                if self.stop_event.wait(min(interval, 0.2)):
                    break
                continue

            frame = map_image.copy()
            robot_visible = False
            tf_error = ''
            try:
                pose_snapshot = self.poses.latest()
                if pose_snapshot is not None:
                    pose = pose_snapshot.message.pose.pose
                    world_x = float(pose.position.x)
                    world_y = float(pose.position.y)
                    world_yaw = quaternion_to_yaw(pose.orientation)
                elif self.tf_buffer is not None:
                    transform = self.tf_buffer.lookup_transform(
                        map_frame,
                        self.slam_base_frame,
                        Time(),
                    )
                    translation = transform.transform.translation
                    rotation = transform.transform.rotation
                    world_x = float(translation.x)
                    world_y = float(translation.y)
                    world_yaw = quaternion_to_yaw(rotation)
                else:
                    raise RuntimeError(
                        f'waiting for pose on {self.slam_pose_topic}'
                    )
                robot_visible = draw_robot_pose(
                    frame,
                    layout,
                    world_x,
                    world_y,
                    world_yaw,
                )
                cv2.putText(
                    frame,
                    f'x {world_x:.3f}  y {world_y:.3f}  '
                    f'yaw {world_yaw * 180.0 / 3.141592653589793:.1f} deg',
                    (12, 28),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (30, 30, 230),
                    2,
                    cv2.LINE_AA,
                )
            except (TransformException, RuntimeError) as exc:
                tf_error = str(exc)

            scan_points = 0
            scan_tf_error = ''
            scan = self.scans.latest() if self.slam_enable_scan else None
            if (
                scan is not None
                and time.monotonic() - scan.received_at
                <= self.slam_scan_max_age
            ):
                scan_frame = scan.message.header.frame_id
                if scan_frame:
                    try:
                        transform = self.tf_buffer.lookup_transform(
                            map_frame,
                            scan_frame,
                            Time(),
                        )
                        translation = transform.transform.translation
                        rotation = transform.transform.rotation
                        scan_points = draw_laser_scan(
                            frame,
                            layout,
                            scan.message.ranges,
                            float(scan.message.angle_min),
                            float(scan.message.angle_increment),
                            float(scan.message.range_min),
                            float(scan.message.range_max),
                            float(translation.x),
                            float(translation.y),
                            quaternion_to_yaw(rotation),
                        )
                    except TransformException as exc:
                        scan_tf_error = str(exc)

            try:
                success, encoded = cv2.imencode(
                    '.jpg',
                    frame,
                    [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality],
                )
                if not success:
                    raise RuntimeError('SLAM JPEG encoding returned false')
            except Exception as exc:
                error = str(exc)
                with self.slam_state_lock:
                    previous_error = self.slam_last_error
                    self.slam_last_error = error
                if error != previous_error:
                    self.get_logger().error(
                        f'Failed to encode SLAM stream: {error}'
                    )
            else:
                elapsed_ms = (time.monotonic() - started) * 1000.0
                source_sequence = (
                    scan.sequence if scan is not None else map_version
                )
                self.slam_jpegs.put(
                    encoded.tobytes(),
                    source_sequence,
                    elapsed_ms,
                )
                with self.slam_state_lock:
                    self.slam_last_error = ''
                    self.slam_tf_error = tf_error
                    self.slam_robot_visible = robot_visible
                    self.slam_scan_points = scan_points
                    self.slam_scan_tf_error = scan_tf_error

            remaining = interval - (time.monotonic() - started)
            if remaining > 0.0 and self.stop_event.wait(remaining):
                break

    def slam_map_response(self):
        """Return the immutable current map PNG and its version."""
        with self.slam_state_lock:
            return self.slam_map_png, self.slam_map_version

    def slam_map_metadata(self):
        """Return geometry needed to overlay live pixels on the map PNG."""
        with self.slam_state_lock:
            layout = self.slam_map_layout
            version = self.slam_map_version
            frame = self.slam_map_frame
        if layout is None:
            return {'status': 'waiting_for_map', 'map_version': 0}
        return {
            'status': 'ok',
            'map_version': version,
            'frame_id': frame,
            'image_width': self.slam_output_width,
            'image_height': self.slam_output_height,
            'grid_width': layout.grid_width,
            'grid_height': layout.grid_height,
            'resolution': layout.resolution,
            'origin_x': layout.origin_x,
            'origin_y': layout.origin_y,
            'origin_yaw': layout.origin_yaw,
            'scale': layout.scale,
            'offset_x': layout.offset_x,
            'offset_y': layout.offset_y,
        }

    def health(self):
        """Return JSON-serializable stream status without blocking ROS."""
        jpeg = self.jpegs.latest()
        metrics = self.jpegs.metrics()
        if jpeg is None:
            status = 'waiting_for_frame'
            frame_age = None
            encode_duration = None
        else:
            frame_age = max(0.0, time.monotonic() - jpeg.encoded_at)
            status = 'ok' if frame_age <= self.stale_timeout else 'stale'
            encode_duration = round(jpeg.encode_duration_ms, 2)
        return {
            'status': status,
            'input_topic': self.input_topic,
            'input_count': self.frames.input_count,
            'encoded_count': metrics['encoded_count'],
            'client_count': metrics['client_count'],
            'frame_age_sec': (
                None if frame_age is None else round(frame_age, 3)
            ),
            'encode_duration_ms': encode_duration,
            'max_web_fps': self.web_fps,
            'jpeg_quality': self.jpeg_quality,
            'last_error': self.last_error,
            'detection_topic': self.detection_topic,
            'detection_input_count': self.detection_input_count,
        }

    def detection_status(self):
        """Return the latest structured YOLO result without old-frame queues."""
        with self.detection_lock:
            payload = self.latest_detection
            received_at = self.detection_received_at
            input_count = self.detection_input_count
            last_error = self.detection_last_error
        if payload is None:
            return {
                'status': 'waiting_for_detections',
                'topic': self.detection_topic,
                'input_count': input_count,
                'age_sec': None,
                'last_error': last_error,
                'detection_count': 0,
                'detections': [],
            }
        age = max(0.0, time.monotonic() - received_at)
        result = dict(payload)
        result.update({
            'status': 'ok' if age <= self.stale_timeout else 'stale',
            'topic': self.detection_topic,
            'input_count': input_count,
            'age_sec': round(age, 3),
            'last_error': last_error,
        })
        return result

    def slam_health(self):
        """Return status for the cached map and composed MJPEG stream."""
        source = self.maps.latest()
        scan_source = self.scans.latest()
        stream = self.slam_jpegs.latest()
        stream_metrics = self.slam_jpegs.metrics()
        with self.slam_state_lock:
            map_ready = self.slam_map_png is not None
            map_frame = self.slam_map_frame
            map_version = self.slam_map_version
            encode_duration = self.slam_map_encode_duration_ms
            robot_visible = self.slam_robot_visible
            tf_error = self.slam_tf_error
            scan_points = self.slam_scan_points
            scan_tf_error = self.slam_scan_tf_error
            last_error = self.slam_last_error
        if source is None or not map_ready:
            status = 'waiting_for_map'
            map_age = None
            width = None
            height = None
            resolution = None
        else:
            map_age = max(0.0, time.monotonic() - source.received_at)
            status = 'ok'
            width = int(source.message.info.width)
            height = int(source.message.info.height)
            resolution = float(source.message.info.resolution)
        scan_age = (
            None
            if scan_source is None
            else max(0.0, time.monotonic() - scan_source.received_at)
        )
        stream_age = (
            None
            if stream is None
            else max(0.0, time.monotonic() - stream.encoded_at)
        )
        return {
            'status': status,
            'map_topic': self.slam_map_topic,
            'map_frame': map_frame,
            'base_frame': self.slam_base_frame,
            'scan_topic': self.slam_scan_topic,
            'map_count': self.maps.input_count,
            'scan_count': self.scans.input_count,
            'map_version': map_version,
            'stream_encoded_count': stream_metrics['encoded_count'],
            'stream_client_count': stream_metrics['client_count'],
            'map_age_sec': (
                None if map_age is None else round(map_age, 3)
            ),
            'stream_age_sec': (
                None if stream_age is None else round(stream_age, 3)
            ),
            'scan_age_sec': (
                None if scan_age is None else round(scan_age, 3)
            ),
            'map_encode_duration_ms': round(encode_duration, 2),
            'width': width,
            'height': height,
            'resolution': resolution,
            'robot_visible': robot_visible,
            'tf_error': tf_error,
            'scan_points_visible': scan_points,
            'scan_tf_error': scan_tf_error,
            'last_error': last_error,
        }

    def stop(self):
        self.stop_event.set()
        self.encoder_thread.join(timeout=2.0)
        self.slam_encoder_thread.join(timeout=2.0)
        self.slam_stream_thread.join(timeout=2.0)


def create_app(
    node,
    fastapi_class,
    response_class,
    streaming_response_class,
):
    """Create the API while keeping ROS state owned by the node."""
    app = fastapi_class(title='Port-ER Top-down Camera API')

    @app.get('/health')
    async def health():
        return node.health()

    def stream_response(store, fps):
        async def generate_mjpeg():
            previous_sequence = 0
            store.add_client()
            try:
                while True:
                    snapshot = store.latest()
                    if (
                        snapshot is not None
                        and snapshot.sequence != previous_sequence
                    ):
                        previous_sequence = snapshot.sequence
                        yield (
                            b'--' + BOUNDARY + b'\r\n'
                            b'Content-Type: image/jpeg\r\n'
                            b'Cache-Control: no-cache\r\n'
                            + (
                                f'Content-Length: {len(snapshot.data)}\r\n\r\n'
                            ).encode('ascii')
                            + snapshot.data
                            + b'\r\n'
                        )
                    await asyncio.sleep(1.0 / fps)
            finally:
                store.remove_client()

        return streaming_response_class(
            generate_mjpeg(),
            media_type='multipart/x-mixed-replace; boundary=frame',
            headers={
                'Cache-Control': 'no-store, no-cache, must-revalidate',
                'Pragma': 'no-cache',
                'X-Accel-Buffering': 'no',
                'Connection': 'keep-alive',
            },
        )

    @app.get('/video')
    async def video():
        return stream_response(node.jpegs, node.web_fps)

    @app.get('/detections')
    async def detections():
        return node.detection_status()

    @app.get('/slam/health')
    async def slam_health():
        return node.slam_health()

    @app.get('/slam/video')
    async def slam_video():
        return stream_response(node.slam_jpegs, node.slam_live_fps)

    @app.get('/slam/view')
    async def slam_view():
        html = (
            files('dashboard')
            .joinpath('web/slam_view.html')
            .read_text(encoding='utf-8')
        )
        return response_class(
            content=html,
            media_type='text/html',
            headers={
                'Cache-Control': 'no-store, no-cache, must-revalidate',
                'Pragma': 'no-cache',
            },
        )

    @app.get('/slam/map.png')
    async def slam_map_png():
        data, version = node.slam_map_response()
        if data is None:
            return response_class(
                content=b'Map is not available yet',
                status_code=503,
                media_type='text/plain',
                headers={'Cache-Control': 'no-store'},
            )
        return response_class(
            content=data,
            media_type='image/png',
            headers={
                'Cache-Control': 'no-cache',
                'ETag': f'"map-{version}"',
                'X-Map-Version': str(version),
            },
        )

    @app.get('/slam/map/metadata')
    async def slam_map_metadata():
        return node.slam_map_metadata()

    return app


def main(args=None):
    (
        FastAPI,
        Response,
        StreamingResponse,
        uvicorn,
    ) = _web_dependencies()
    rclpy.init(args=args)
    node = DashboardStreamNode()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(
        target=executor.spin,
        name='dashboard-ros-executor',
        daemon=True,
    )
    ros_thread.start()

    app = create_app(
        node,
        FastAPI,
        Response,
        StreamingResponse,
    )
    config = uvicorn.Config(
        app,
        host=node.host,
        port=node.port,
        log_level='info',
        access_log=False,
        lifespan='off',
    )
    server = uvicorn.Server(config)
    try:
        server.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.stop()
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
