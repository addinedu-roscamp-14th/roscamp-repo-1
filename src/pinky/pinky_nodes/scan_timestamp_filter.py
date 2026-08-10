#!/usr/bin/env python3

from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def classify_stamp(age_sec, max_age_sec, max_future_sec):
    if age_sec > max_age_sec:
        return 'stale'
    if age_sec < -max_future_sec:
        return 'future'
    return 'valid'


class ScanTimestampFilter(Node):
    """Forward only the newest scan that already has matching odom TF."""

    def __init__(self):
        super().__init__('scan_timestamp_filter')
        self.declare_parameter('input_topic', 'scan_raw')
        self.declare_parameter('output_topic', 'scan')
        self.declare_parameter('target_frame', 'odom')
        self.declare_parameter('max_age_sec', 0.5)
        self.declare_parameter('max_future_sec', 0.1)
        self.declare_parameter('retry_rate_hz', 50.0)

        input_topic = str(self.get_parameter('input_topic').value)
        output_topic = str(self.get_parameter('output_topic').value)
        self.target_frame = str(self.get_parameter('target_frame').value)
        self.max_age_sec = float(self.get_parameter('max_age_sec').value)
        self.max_future_sec = float(
            self.get_parameter('max_future_sec').value
        )
        retry_rate_hz = float(self.get_parameter('retry_rate_hz').value)
        if self.max_age_sec <= 0.0:
            raise ValueError('max_age_sec must be positive')
        if self.max_future_sec < 0.0:
            raise ValueError('max_future_sec must be non-negative')
        if retry_rate_hz <= 0.0:
            raise ValueError('retry_rate_hz must be positive')

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.publisher = self.create_publisher(
            LaserScan, output_topic, sensor_qos
        )
        self.subscription = self.create_subscription(
            LaserScan, input_topic, self._on_scan, sensor_qos
        )
        self.tf_buffer = Buffer(cache_time=Duration(seconds=2.0))
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.pending_scan = None
        self.received_count = 0
        self.published_count = 0
        self.stale_count = 0
        self.future_count = 0
        self.tf_timeout_count = 0
        self.last_reported_drop_count = 0
        self.pending_waiting_for_tf = False
        self.timer = self.create_timer(1.0 / retry_rate_hz, self._flush_latest)
        self.status_timer = self.create_timer(5.0, self._report_status)

        self.get_logger().info(
            f'Scan timestamp filter: {input_topic} -> {output_topic}, '
            f'target_frame={self.target_frame}, '
            f'max_age={self.max_age_sec:.2f}s'
        )

    def _on_scan(self, message):
        self.received_count += 1
        self.pending_scan = message
        self.pending_waiting_for_tf = False
        self._flush_latest()

    def _flush_latest(self):
        message = self.pending_scan
        if message is None:
            return

        stamp = Time.from_msg(message.header.stamp)
        if stamp.nanoseconds == 0:
            self.pending_scan = None
            self.stale_count += 1
            self.pending_waiting_for_tf = False
            return

        age_sec = (
            self.get_clock().now().nanoseconds - stamp.nanoseconds
        ) / 1e9
        stamp_state = classify_stamp(
            age_sec, self.max_age_sec, self.max_future_sec
        )
        if stamp_state == 'stale':
            self.pending_scan = None
            if self.pending_waiting_for_tf:
                self.tf_timeout_count += 1
            else:
                self.stale_count += 1
            self.pending_waiting_for_tf = False
            return
        if stamp_state == 'future':
            self.pending_scan = None
            self.future_count += 1
            self.pending_waiting_for_tf = False
            return

        if not self.tf_buffer.can_transform(
            self.target_frame,
            message.header.frame_id,
            stamp,
            timeout=Duration(seconds=0.0),
        ):
            self.pending_waiting_for_tf = True
            return

        self.publisher.publish(message)
        self.published_count += 1
        self.pending_scan = None
        self.pending_waiting_for_tf = False

    def _report_status(self):
        pending = self.pending_scan
        if pending is not None:
            stamp = Time.from_msg(pending.header.stamp)
            age_sec = (
                self.get_clock().now().nanoseconds - stamp.nanoseconds
            ) / 1e9
            if age_sec > self.max_age_sec:
                self.pending_scan = None
                if self.pending_waiting_for_tf:
                    self.tf_timeout_count += 1
                else:
                    self.stale_count += 1
                self.pending_waiting_for_tf = False

        drop_count = (
            self.stale_count + self.future_count + self.tf_timeout_count
        )
        if drop_count > self.last_reported_drop_count:
            self.get_logger().warn(
                'Scan filter drops: '
                f'stale={self.stale_count}, future={self.future_count}, '
                f'tf_timeout={self.tf_timeout_count}; '
                f'received={self.received_count}, '
                f'published={self.published_count}'
            )
            self.last_reported_drop_count = drop_count


def main(args=None):
    rclpy.init(args=args)
    node = ScanTimestampFilter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
