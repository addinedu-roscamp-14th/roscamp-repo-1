#!/usr/bin/env python3

import json
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
import torch
from ultralytics import YOLO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class YoloNode(Node):
    # BGR colors keyed by color keyword expected to appear in OBB class labels
    # (e.g. 'container_red', 'blue_container'). Matched by substring, case-insensitive.
    CONTAINER_COLOR_MAP = {
        'black': (60, 60, 60),
        'gray': (160, 160, 160),
        'brown': (19, 69, 139),
    }

    def __init__(self):
        super().__init__('yolo_node')

        self.declare_parameter('input_topic', '/image_rect/compressed')
        self.declare_parameter('input_is_compressed', True)
        self.declare_parameter('annotated_topic', '/central/yolo/image_annotated')
        self.declare_parameter('detection_topic', '/central/yolo/detections')
        self.declare_parameter('weights_path', 'config/weights/best.pt')
        self.declare_parameter('confidence_threshold', 0.6)
        self.declare_parameter('device', '')
        self.declare_parameter('publish_annotated_image', True)
        self.declare_parameter('show_heading_annotation', False)
        self.declare_parameter('enable_obb', True)
        self.declare_parameter('obb_weights_path', 'config/weights/best1.pt')
        self.declare_parameter('obb_confidence_threshold', 0.6)
        self.declare_parameter(
            'class_names',
            [
                'trailer',
                'car_yellow',
                'car_blue',
                'A-1',
                'A-2',
                'A-3',
                'B-1',
            ],
        )

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.input_is_compressed = bool(self.get_parameter('input_is_compressed').value)
        self.annotated_topic = str(self.get_parameter('annotated_topic').value)
        self.detection_topic = str(self.get_parameter('detection_topic').value)
        self.weights_path = self.resolve_weights_path(str(self.get_parameter('weights_path').value))
        self.confidence_threshold = float(self.get_parameter('confidence_threshold').value)
        self.device = str(self.get_parameter('device').value).strip()
        self.publish_annotated_image = bool(self.get_parameter('publish_annotated_image').value)
        self.show_heading_annotation = bool(
            self.get_parameter('show_heading_annotation').value
        )
        self.expected_class_names = [str(name) for name in self.get_parameter('class_names').value]
        self.enable_obb = bool(self.get_parameter('enable_obb').value)
        self.obb_weights_path = self.resolve_weights_path(str(self.get_parameter('obb_weights_path').value))
        self.obb_confidence_threshold = float(self.get_parameter('obb_confidence_threshold').value)

        if not self.weights_path.exists():
            raise FileNotFoundError(f'YOLO weights not found: {self.weights_path}')

        self.bridge = CvBridge()
        self.model = YOLO(str(self.weights_path))
        self.class_names = self.model.names

        self.get_logger().info(f'Expected segmentation classes: {", ".join(self.expected_class_names)}')
        self.get_logger().info(f'Model class map: {self.class_names}')

        self.obb_model = None
        self.obb_stream = None
        self.obb_executor = None
        if self.enable_obb:
            if self.obb_weights_path.exists():
                self.obb_model = YOLO(str(self.obb_weights_path))
                self.get_logger().info(
                    f'OBB model loaded: {self.obb_weights_path} (classes: {self.obb_model.names})'
                )
                if torch.cuda.is_available():
                    self.obb_stream = torch.cuda.Stream()
                    self.get_logger().info('OBB inference will run concurrently on a separate CUDA stream')
                self.obb_executor = ThreadPoolExecutor(max_workers=1)
            else:
                self.get_logger().warn(
                    f'OBB weights not found at {self.obb_weights_path}; OBB inference disabled'
                )

        self.annotated_pub = None
        if self.publish_annotated_image:
            self.annotated_pub = self.create_publisher(Image, self.annotated_topic, 10)
        self.detection_pub = self.create_publisher(String, self.detection_topic, 10)

        if self.input_is_compressed:
            self.subscription = self.create_subscription(
                CompressedImage,
                self.input_topic,
                self.on_compressed_image,
                10,
            )
        else:
            self.subscription = self.create_subscription(
                Image,
                self.input_topic,
                self.on_image,
                10,
            )

        self.get_logger().info(f'YOLO node started with weights: {self.weights_path}')
        self.get_logger().info(
            f'Subscribing input topic: {self.input_topic} '
            f'({"compressed" if self.input_is_compressed else "raw"})'
        )
        self.get_logger().info(f'Publishing detections to: {self.detection_topic}')

    def destroy_node(self):
        if self.obb_executor is not None:
            self.obb_executor.shutdown(wait=False)
        super().destroy_node()

    def resolve_weights_path(self, configured_path: str) -> Path:
        path = Path(configured_path)
        if path.is_absolute():
            return path

        candidates = [Path.cwd() / path]
        module_path = Path(__file__).resolve()

        for parent in module_path.parents:
            candidates.append(parent / path)

        for candidate in candidates:
            if candidate.exists():
                return candidate

        return candidates[0]

    def on_image(self, msg: Image):
        try:
            frame_bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Failed to convert image message: {exc}')
            return

        self.process_frame(frame_bgr, msg.header.frame_id, msg.header.stamp)

    def on_compressed_image(self, msg: CompressedImage):
        try:
            buffer = np.frombuffer(msg.data, dtype=np.uint8)
            frame_array = cv2.imdecode(buffer, cv2.IMREAD_COLOR)
        except Exception as exc:
            self.get_logger().error(f'Failed to decode compressed image: {exc}')
            return

        if frame_array is None:
            self.get_logger().error('Compressed image decode returned None')
            return

        self.process_frame(frame_array, msg.header.frame_id, msg.header.stamp)

    def process_frame(self, frame_bgr, frame_id, stamp):
        obb_future = None
        if self.obb_model is not None:
            obb_future = self.obb_executor.submit(self.infer_obb, frame_bgr)

        results = self.model.predict(
            source=frame_bgr,
            conf=self.confidence_threshold,
            device=self.device or None,
            verbose=False,
        )

        if not results:
            if obb_future is not None:
                obb_future.result()
            return

        result = results[0]
        annotated_frame = result.plot()

        detections = []
        boxes = result.boxes
        masks = result.masks

        if boxes is not None:
            for index, box in enumerate(boxes):
                class_id = int(box.cls[0].item())
                confidence = float(box.conf[0].item())
                label = self.class_names.get(class_id, str(class_id))

                x1, y1, x2, y2 = [float(value) for value in box.xyxy[0].tolist()]
                detection = {
                    'class_id': class_id,
                    'label': label,
                    'confidence': round(confidence, 4),
                    'bbox_xyxy': [round(x1, 2), round(y1, 2), round(x2, 2), round(y2, 2)],
                }

                if masks is not None and masks.xy is not None and index < len(masks.xy):
                    polygon = masks.xy[index]
                    detection['mask_xy'] = [[round(float(x), 2), round(float(y), 2)] for x, y in polygon.tolist()]
                    heading_deg = self.compute_heading_deg(polygon)
                    if heading_deg is not None:
                        detection['heading_deg'] = round(heading_deg, 2)
                        if self.show_heading_annotation:
                            self.draw_heading_label(
                                annotated_frame,
                                x1,
                                y1,
                                heading_deg,
                            )

                if not self.expected_class_names or label in self.expected_class_names:
                    detections.append(detection)

        obb_detections = []
        if obb_future is not None:
            obb = obb_future.result()
            obb_detections = self.postprocess_obb(obb, annotated_frame)

        summary = {
            'frame_id': frame_id,
            'stamp': {
                'sec': stamp.sec,
                'nanosec': stamp.nanosec,
            },
            'detection_count': len(detections),
            'segmentation': masks is not None,
            'detections': detections,
            'obb_detection_count': len(obb_detections),
            'obb_detections': obb_detections,
        }

        detection_msg = String()
        detection_msg.data = json.dumps(summary, ensure_ascii=False)
        self.detection_pub.publish(detection_msg)

        if self.annotated_pub is not None:
            annotated_msg = Image()
            annotated_msg.header.frame_id = frame_id
            annotated_msg.header.stamp = stamp
            annotated_msg.height = int(annotated_frame.shape[0])
            annotated_msg.width = int(annotated_frame.shape[1])
            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = 0
            annotated_msg.step = int(annotated_frame.shape[1] * annotated_frame.shape[2])
            annotated_msg.data = annotated_frame.tobytes()
            self.annotated_pub.publish(annotated_msg)

    def infer_obb(self, frame_bgr):
        """Run OBB inference, optionally on its own CUDA stream so it overlaps
        with the segmentation model's inference happening on the main thread."""
        if self.obb_stream is not None:
            with torch.cuda.stream(self.obb_stream):
                results = self.obb_model.predict(
                    source=frame_bgr,
                    conf=self.obb_confidence_threshold,
                    device=self.device or None,
                    verbose=False,
                )
            self.obb_stream.synchronize()
        else:
            results = self.obb_model.predict(
                source=frame_bgr,
                conf=self.obb_confidence_threshold,
                device=self.device or None,
                verbose=False,
            )

        if not results:
            return None

        return results[0].obb

    def postprocess_obb(self, obb, annotated_frame):
        detections = []

        if obb is None or obb.xyxyxyxy is None:
            return detections

        obb_class_names = self.obb_model.names

        for index in range(len(obb)):
            class_id = int(obb.cls[index].item())
            confidence = float(obb.conf[index].item())
            label = obb_class_names.get(class_id, str(class_id))
            corners = obb.xyxyxyxy[index].tolist()
            heading_deg = float(np.degrees(obb.xywhr[index][4].item()))

            detections.append({
                'class_id': class_id,
                'label': label,
                'confidence': round(confidence, 4),
                'corners_xy': [[round(float(x), 2), round(float(y), 2)] for x, y in corners],
                'heading_deg': round(heading_deg, 2),
            })

            self.draw_obb_box(annotated_frame, corners, label, confidence)

        return detections

    def resolve_container_color(self, label):
        normalized = label.lower()
        for keyword, color in self.CONTAINER_COLOR_MAP.items():
            if keyword in normalized:
                return color
        return (0, 255, 255)

    def draw_obb_box(self, image, corners, label, confidence):
        points = np.asarray(corners, dtype=np.int32).reshape((-1, 1, 2))
        color = self.resolve_container_color(label)
        cv2.polylines(image, [points], True, color, 2)

        text = f'{label} {confidence:.2f}'
        origin_x = int(corners[0][0])
        origin_y = max(18, int(corners[0][1]) - 8)
        cv2.putText(
            image,
            text,
            (origin_x, origin_y),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            color,
            2,
            cv2.LINE_AA,
        )

    def compute_heading_deg(self, polygon):
        if polygon is None or len(polygon) < 3:
            return None

        points = np.asarray(polygon, dtype=np.float32)
        rect = cv2.minAreaRect(points)
        (width, height) = rect[1]

        if width <= 0.0 or height <= 0.0:
            return None

        angle = float(rect[2])
        if width < height:
            angle += 90.0

        while angle >= 90.0:
            angle -= 180.0
        while angle < -90.0:
            angle += 180.0

        return angle

    def draw_heading_label(self, image, x1, y1, heading_deg):
        text = f'heading={heading_deg:.1f} deg'
        origin_x = max(0, int(x1))
        origin_y = max(18, int(y1) - 28)
        font = cv2.FONT_HERSHEY_SIMPLEX
        scale = 0.55
        thickness = 2

        (text_width, text_height), baseline = cv2.getTextSize(text, font, scale, thickness)
        top_left = (origin_x, max(0, origin_y - text_height - baseline - 4))
        bottom_right = (
            min(image.shape[1] - 1, origin_x + text_width + 8),
            min(image.shape[0] - 1, origin_y + baseline + 4),
        )

        cv2.rectangle(image, top_left, bottom_right, (20, 20, 20), -1)
        cv2.putText(
            image,
            text,
            (origin_x + 4, origin_y),
            font,
            scale,
            (255, 255, 255),
            thickness,
            cv2.LINE_AA,
        )


def main(args=None):
    rclpy.init(args=args)
    node = YoloNode()

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
