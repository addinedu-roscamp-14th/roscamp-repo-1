#!/usr/bin/env python3

import json
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import String


class YoloNode(Node):
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

        if not self.weights_path.exists():
            raise FileNotFoundError(f'YOLO weights not found: {self.weights_path}')

        self.bridge = CvBridge()
        self.model = YOLO(str(self.weights_path))
        self.class_names = self.model.names

        self.get_logger().info(f'Expected segmentation classes: {", ".join(self.expected_class_names)}')
        self.get_logger().info(f'Model class map: {self.class_names}')

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
        results = self.model.predict(
            source=frame_bgr,
            conf=self.confidence_threshold,
            device=self.device or None,
            verbose=False,
        )

        if not results:
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

        summary = {
            'frame_id': frame_id,
            'stamp': {
                'sec': stamp.sec,
                'nanosec': stamp.nanosec,
            },
            'detection_count': len(detections),
            'segmentation': masks is not None,
            'detections': detections,
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
