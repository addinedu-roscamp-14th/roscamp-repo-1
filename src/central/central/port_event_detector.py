#!/usr/bin/env python3

"""Debounce vessel arrival/departure from YOLO OBB detections in an ROI."""

from __future__ import annotations

from collections import deque
import json
import time
import uuid

from porter_interfaces.msg import PortEvent
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


DEFAULT_ARRIVAL_ROI = [
    0.8424657534246576,
    0.6316695352839932,
    0.9575342465753425,
    0.8399311531841652,
]


class PortEventDetector(Node):
    def __init__(self):
        super().__init__('port_event_detector')
        self.declare_parameter('detection_topic', '/central/yolo/detections')
        self.declare_parameter(
            'event_topic', '/central/autonomy/port_events'
        )
        self.declare_parameter(
            'roi_config_topic', '/central/autonomy/arrival_roi_config'
        )
        self.declare_parameter('roi_normalized', DEFAULT_ARRIVAL_ROI)
        self.declare_parameter(
            'vessel_label_keywords',
            ['container', 'black', 'gray', 'brown', 'vessel', 'ship'],
        )
        self.declare_parameter('minimum_confidence', 0.65)
        self.declare_parameter('minimum_roi_overlap', 0.30)
        self.declare_parameter('history_size', 5)
        self.declare_parameter('required_hits', 3)
        self.declare_parameter('cargo_addition_required_hits', 3)
        self.declare_parameter('departure_absence_sec', 10.0)

        self.roi = self._valid_roi(
            list(self.get_parameter('roi_normalized').value)
        )
        self.label_keywords = tuple(
            str(item).lower()
            for item in self.get_parameter('vessel_label_keywords').value
        )
        self.minimum_confidence = float(
            self.get_parameter('minimum_confidence').value
        )
        self.minimum_overlap = float(
            self.get_parameter('minimum_roi_overlap').value
        )
        history_size = max(1, int(self.get_parameter('history_size').value))
        self.required_hits = max(
            1, min(history_size, int(self.get_parameter('required_hits').value))
        )
        self.cargo_addition_required_hits = max(
            1,
            min(
                history_size,
                int(
                    self.get_parameter(
                        'cargo_addition_required_hits'
                    ).value
                ),
            ),
        )
        self.departure_absence_sec = float(
            self.get_parameter('departure_absence_sec').value
        )
        self.history = deque(maxlen=history_size)
        self.match_count_history = deque(maxlen=history_size)
        self.vessel_present = False
        self.maximum_stable_match_count = 0
        self.last_positive_at = None
        self.last_confidence = 0.0

        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            PortEvent, str(self.get_parameter('event_topic').value), qos
        )
        self.status_publisher = self.create_publisher(
            String, '/central/autonomy/port_status', qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter('detection_topic').value),
            self._on_detections,
            10,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('roi_config_topic').value),
            self._on_roi_config,
            qos,
        )
        self.create_timer(0.5, self._check_departure)
        self._publish_status('WAITING_FOR_VESSEL')

    @staticmethod
    def _valid_roi(values):
        if len(values) != 4:
            raise ValueError('roi_normalized must contain four values')
        x1, y1, x2, y2 = (float(value) for value in values)
        if not (0.0 <= x1 < x2 <= 1.0 and 0.0 <= y1 < y2 <= 1.0):
            raise ValueError('ROI must satisfy 0<=x1<x2<=1 and 0<=y1<y2<=1')
        return (x1, y1, x2, y2)

    def _on_roi_config(self, message):
        try:
            payload = json.loads(message.data)
            roi = self._valid_roi([
                payload['x_min'], payload['y_min'],
                payload['x_max'], payload['y_max'],
            ])
        except (KeyError, TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Invalid arrival ROI ignored: {exc}')
            return
        self.roi = roi
        self.history.clear()
        self.match_count_history.clear()
        self.maximum_stable_match_count = 0
        self.get_logger().info(f'Arrival ROI updated: {self.roi}')
        self._publish_status('ROI_UPDATED')

    def _on_detections(self, message):
        try:
            payload = json.loads(message.data)
            detections = payload.get('obb_detections', [])
        except (TypeError, json.JSONDecodeError):
            return
        width, height = self._image_size(payload)
        matched = []
        for detection in detections:
            label = str(detection.get('label', '')).lower()
            confidence = float(detection.get('confidence', 0.0))
            if confidence < self.minimum_confidence:
                continue
            if not any(keyword in label for keyword in self.label_keywords):
                continue
            corners = detection.get('corners_xy') or []
            overlap = self._roi_overlap(corners, width, height)
            if overlap >= self.minimum_overlap:
                matched.append((confidence, label, overlap))
        positive = bool(matched)
        self.history.append(positive)
        self.match_count_history.append(len(matched))
        if positive:
            self.last_positive_at = time.monotonic()
            self.last_confidence = max(item[0] for item in matched)
        if not self.vessel_present and sum(self.history) >= self.required_hits:
            self.vessel_present = True
            # Use the accepted frame as the baseline. A transient earlier
            # over-count must not suppress a real cargo addition later.
            self.maximum_stable_match_count = max(1, len(matched))
            details = {
                'change_type': 'VESSEL_ARRIVED',
                'roi_normalized': self.roi,
                'matched_object_count': len(matched),
                'matches': [
                    {'confidence': conf, 'label': label, 'overlap': overlap}
                    for conf, label, overlap in matched
                ],
            }
            self._publish_event(PortEvent.VESSEL_ARRIVED, details)
            self._publish_status('VESSEL_PRESENT')
        elif self.vessel_present:
            increased_count = self._stable_count_increase(
                self.match_count_history,
                self.maximum_stable_match_count,
                self.cargo_addition_required_hits,
            )
            if increased_count is not None:
                previous_count = self.maximum_stable_match_count
                self.maximum_stable_match_count = increased_count
                details = {
                    'change_type': 'CARGO_ADDED',
                    'roi_normalized': self.roi,
                    'previous_object_count': previous_count,
                    'matched_object_count': increased_count,
                    'matches': [
                        {
                            'confidence': conf,
                            'label': label,
                            'overlap': overlap,
                        }
                        for conf, label, overlap in matched
                    ],
                }
                self._publish_event(PortEvent.VESSEL_ARRIVED, details)
                self._publish_status('CARGO_ADDED')

    @staticmethod
    def _stable_count_increase(counts, baseline, required_hits):
        """Return a conservative new count after a stable count increase."""
        recent = list(counts)[-required_hits:]
        if len(recent) < required_hits:
            return None
        if not all(count > baseline for count in recent):
            return None
        return min(recent)

    @staticmethod
    def _image_size(payload):
        width = float(payload.get('image_width', 640) or 640)
        height = float(payload.get('image_height', 480) or 480)
        return width, height

    def _roi_overlap(self, corners, width, height):
        if not corners:
            return 0.0
        xs = [float(point[0]) / width for point in corners]
        ys = [float(point[1]) / height for point in corners]
        x1, x2 = max(0.0, min(xs)), min(1.0, max(xs))
        y1, y2 = max(0.0, min(ys)), min(1.0, max(ys))
        area = max(0.0, x2 - x1) * max(0.0, y2 - y1)
        if area <= 0.0:
            return 0.0
        rx1, ry1, rx2, ry2 = self.roi
        intersection = (
            max(0.0, min(x2, rx2) - max(x1, rx1))
            * max(0.0, min(y2, ry2) - max(y1, ry1))
        )
        return intersection / area

    def _check_departure(self):
        if not self.vessel_present or self.last_positive_at is None:
            return
        absent_for = time.monotonic() - self.last_positive_at
        if absent_for < self.departure_absence_sec:
            return
        self.vessel_present = False
        self.history.clear()
        self.match_count_history.clear()
        self.maximum_stable_match_count = 0
        self._publish_event(
            PortEvent.VESSEL_DEPARTED, {'absent_for_sec': absent_for}
        )
        self._publish_status('WAITING_FOR_VESSEL')

    def _publish_event(self, event_type, details):
        message = PortEvent()
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = 'topdown_camera'
        message.event_id = str(uuid.uuid4())
        message.event_type = event_type
        message.event_type_text = (
            'VESSEL_ARRIVED'
            if event_type == PortEvent.VESSEL_ARRIVED
            else 'VESSEL_DEPARTED'
        )
        message.active = event_type == PortEvent.VESSEL_ARRIVED
        message.confidence = float(self.last_confidence)
        message.details_json = json.dumps(details, ensure_ascii=False)
        self.publisher.publish(message)
        self.get_logger().info(
            f'Port event: {message.event_type_text} ({message.event_id})'
        )

    def _publish_status(self, state):
        message = String()
        message.data = json.dumps({
            'state': state,
            'vessel_present': self.vessel_present,
            'roi_normalized': self.roi,
            'recent_hits': int(sum(self.history)),
            'history_size': len(self.history),
            'matched_object_count': (
                self.match_count_history[-1]
                if self.match_count_history else 0
            ),
            'maximum_stable_match_count': self.maximum_stable_match_count,
        })
        self.status_publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = PortEventDetector()
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
