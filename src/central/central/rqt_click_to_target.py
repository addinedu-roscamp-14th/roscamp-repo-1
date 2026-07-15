#!/usr/bin/env python3

from geometry_msgs.msg import Point, PointStamped
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node


class RqtClickToTarget(Node):
    def __init__(self):
        super().__init__('rqt_click_to_target')

        self.declare_parameter('mouse_topic', '/central/yolo/image_annotated_mouse_left')
        self.declare_parameter('target_pixel_topic', '/central/target_pixel')
        self.declare_parameter('frame_id', 'camera')

        self.mouse_topic = str(self.get_parameter('mouse_topic').value)
        self.target_pixel_topic = str(self.get_parameter('target_pixel_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)

        self.publisher = self.create_publisher(PointStamped, self.target_pixel_topic, 10)
        self.create_subscription(Point, self.mouse_topic, self.on_mouse_click, 10)

        self.get_logger().info(f'Subscribing rqt mouse clicks: {self.mouse_topic}')
        self.get_logger().info(f'Publishing target pixels: {self.target_pixel_topic}')

    def on_mouse_click(self, msg):
        target = PointStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = self.frame_id
        target.point.x = float(msg.x)
        target.point.y = float(msg.y)
        target.point.z = float(msg.z)

        self.publisher.publish(target)
        self.get_logger().info(
            f'Published target pixel: x={target.point.x:.1f}, y={target.point.y:.1f}'
        )


def main(args=None):
    rclpy.init(args=args)
    node = RqtClickToTarget()

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
