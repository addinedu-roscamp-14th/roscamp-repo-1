#!/usr/bin/env python3

import cv2

import rclpy
from cv_bridge import CvBridge
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image


class ImageViewer(Node):
    def __init__(self):
        super().__init__('image_viewer')

        self.declare_parameter('image_topic', '/top_camera/yolo/image_annotated')
        self.declare_parameter('window_name', 'Topview YOLO')
        self.declare_parameter('display_width', 640)
        self.declare_parameter('display_height', 480)
        self.declare_parameter('window_x', 700)
        self.declare_parameter('window_y', 520)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.window_name = str(self.get_parameter('window_name').value)
        self.display_width = int(self.get_parameter('display_width').value)
        self.display_height = int(self.get_parameter('display_height').value)
        self.window_x = int(self.get_parameter('window_x').value)
        self.window_y = int(self.get_parameter('window_y').value)

        self.bridge = CvBridge()
        self.latest_frame = None

        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(self.window_name, self.display_width, self.display_height)
        cv2.moveWindow(self.window_name, self.window_x, self.window_y)

        self.sub = self.create_subscription(Image, self.image_topic, self.on_image, 10)
        self.timer = self.create_timer(0.03, self.spin_window)
        self.status_timer = self.create_timer(5.0, self.log_status)

        self.get_logger().info(f'Subscribed image: {self.image_topic}')

    def on_image(self, msg):
        try:
            self.latest_frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:
            self.get_logger().warn(f'Failed to convert image: {exc}')

    def spin_window(self):
        if self.latest_frame is not None:
            frame = cv2.resize(
                self.latest_frame,
                (self.display_width, self.display_height),
                interpolation=cv2.INTER_AREA,
            )
        else:
            frame = self.build_waiting_image()

        cv2.imshow(self.window_name, frame)
        key = cv2.waitKey(1)
        if key in (27, ord('q')):
            rclpy.shutdown()

    def build_waiting_image(self):
        frame = cv2.UMat(self.display_height, self.display_width, cv2.CV_8UC3).get()
        frame[:] = (0, 0, 0)
        cv2.putText(
            frame,
            f'Waiting for {self.image_topic}',
            (30, max(40, self.display_height // 2)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        return frame

    def log_status(self):
        if self.latest_frame is None:
            self.get_logger().warn(f'Waiting for image frames on {self.image_topic}')

    def destroy_node(self):
        try:
            cv2.destroyWindow(self.window_name)
        except cv2.error:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ImageViewer()

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
