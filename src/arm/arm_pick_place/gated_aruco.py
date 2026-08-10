"""Gate the package-local dual ArUco detector to stationary search windows."""

from .dual_aruco_pose_publisher import DualArucoPosePublisher

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)

from std_msgs.msg import Bool


class GatedPickPlaceAruco(DualArucoPosePublisher):
    """Publish marker TF only while the coordinator enables detection."""

    def __init__(self):
        """Create the detector with a dedicated enable topic."""
        self.detection_enabled = False
        super().__init__()
        self.create_subscription(
            Bool,
            '/arm/pick_place/detection_enabled',
            self.on_detection_enabled,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.get_logger().info('Homography Pick/Place detector is gated OFF')

    def on_detection_enabled(self, message):
        """Apply the stationary-search detection gate."""
        enabled = bool(message.data)
        if enabled == self.detection_enabled:
            return
        self.detection_enabled = enabled
        self.last_detected_ids = None
        self.get_logger().info(
            f'ArUco detection {"ENABLED" if enabled else "DISABLED"}'
        )

    def on_image(self, message):
        """Discard moving-camera frames before any vision computation."""
        if self.detection_enabled:
            super().on_image(message)


def main(args=None):
    """Run the gated detector."""
    rclpy.init(args=args)
    node = None
    try:
        node = GatedPickPlaceAruco()
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
