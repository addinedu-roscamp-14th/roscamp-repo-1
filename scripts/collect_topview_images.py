#!/usr/bin/env python3

import argparse
from datetime import datetime
from pathlib import Path

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class TopviewImageCollector(Node):
    def __init__(self, topic, output_dir, video_output_dir, fps):
        super().__init__('topview_image_collector')
        self.output_dir = output_dir
        self.output_dir.mkdir(parents=True, exist_ok=True)
        self.video_output_dir = video_output_dir
        self.video_output_dir.mkdir(parents=True, exist_ok=True)
        self.fps = fps
        self.video_writer = None
        self.video_file = None
        self.bridge = CvBridge()
        self.latest_frame = None
        self.saved_count = 0
        self.subscription = self.create_subscription(Image, topic, self.on_image, 10)
        self.get_logger().info(f'Collecting {topic} into {self.output_dir}')

    def on_image(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().error(f'Image conversion failed: {exc}')

    def save(self):
        if self.latest_frame is None:
            self.get_logger().warn('No topview image received yet')
            return

        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S_%f')
        output_file = self.output_dir / f'topview_{timestamp}.jpg'
        if not cv2.imwrite(str(output_file), self.latest_frame):
            self.get_logger().error(f'Failed to save {output_file}')
            return

        self.saved_count += 1
        self.get_logger().info(f'Saved #{self.saved_count}: {output_file.name}')

    def toggle_recording(self):
        if self.video_writer is not None:
            self.stop_recording()
            return

        if self.latest_frame is None:
            self.get_logger().warn('No topview image received yet')
            return

        height, width = self.latest_frame.shape[:2]
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.video_file = self.video_output_dir / f'topview_{timestamp}.mp4'
        self.video_writer = cv2.VideoWriter(
            str(self.video_file),
            cv2.VideoWriter_fourcc(*'mp4v'),
            self.fps,
            (width, height),
        )
        if not self.video_writer.isOpened():
            self.video_writer.release()
            self.video_writer = None
            self.get_logger().error(f'Failed to start video recording: {self.video_file}')
            return
        self.get_logger().info(f'Recording started: {self.video_file}')

    def write_video_frame(self):
        if self.video_writer is not None and self.latest_frame is not None:
            self.video_writer.write(self.latest_frame)

    def stop_recording(self):
        if self.video_writer is None:
            return
        self.video_writer.release()
        self.video_writer = None
        self.get_logger().info(f'Recording saved: {self.video_file}')

    def destroy_node(self):
        self.stop_recording()
        super().destroy_node()


def parse_args():
    parser = argparse.ArgumentParser(description='Save ROS topview images for YOLO labeling')
    parser.add_argument('--topic', default='/top_camera/image_raw')
    parser.add_argument(
        '--output',
        default=str(Path.home() / 'poter_ws/datasets/topview/unlabeled'),
    )
    parser.add_argument(
        '--video-output',
        default=str(Path.home() / 'poter_ws/datasets/topview/videos'),
    )
    parser.add_argument('--fps', type=float, default=30.0)
    return parser.parse_known_args()


def main():
    args, ros_args = parse_args()
    rclpy.init(args=ros_args)
    node = TopviewImageCollector(
        args.topic,
        Path(args.output).expanduser(),
        Path(args.video_output).expanduser(),
        args.fps,
    )

    cv2.namedWindow('Topview dataset collector', cv2.WINDOW_AUTOSIZE)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if node.latest_frame is None:
                continue

            display = node.latest_frame.copy()
            node.write_video_frame()
            status = 'RECORDING' if node.video_writer is not None else 'READY'
            cv2.putText(
                display,
                f'{status}   R: record/stop   S: photo   Q/ESC: quit',
                (15, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 255),
                2,
            )
            cv2.imshow('Topview dataset collector', display)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('s'):
                node.save()
            elif key == ord('r'):
                node.toggle_recording()
            elif key in (ord('q'), 27):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
