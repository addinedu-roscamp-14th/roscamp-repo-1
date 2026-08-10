#!/usr/bin/env python3

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class YuyvToBgrNode(Node):
    def __init__(self):
        super().__init__("yuyv_to_bgr_node")

        self.bridge = CvBridge()

        self.publisher = self.create_publisher(
            Image,
            "/camera/image_color",
            10,
        )

        self.subscription = self.create_subscription(
            Image,
            "/camera/image_raw",
            self.image_callback,
            10,
        )

        self.get_logger().info(
            "/camera/image_raw → /camera/image_color 변환 시작"
        )

    def image_callback(self, msg: Image) -> None:
        try:
            # cv_bridge가 YUYV/YUV422를 BGR8로 변환
            bgr_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding="bgr8",
            )

            output = self.bridge.cv2_to_imgmsg(
                bgr_image,
                encoding="bgr8",
            )

            output.header = msg.header
            self.publisher.publish(output)

        except Exception as error:
            self.get_logger().error(
                f"영상 변환 실패: {type(error).__name__}: {error}"
            )


def main():
    rclpy.init()
    node = YuyvToBgrNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
