"""Run the existing dual ArUco detector only when explicitly enabled."""

from arm.dual_aruco_pose_publisher import DualArucoPosePublisher
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.qos import (
    DurabilityPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool


class GatedDualArucoPosePublisher(DualArucoPosePublisher):
    """Suppress image processing and TF publication during robot motion."""

    def __init__(self):
        self.detection_enabled = False
        super().__init__()
        self.create_subscription(
            Bool,
            '/arm/simple_pick_place/detection_enabled',
            self.on_detection_enabled,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self.get_logger().info(
            'ArUco gate ready; detection starts disabled'
        )

    def on_detection_enabled(self, message):
        """Apply the coordinator's observation-only detection state."""
        enabled = bool(message.data)
        if enabled == self.detection_enabled:
            return
        self.detection_enabled = enabled
        self.last_detected_ids = None
        state = 'ENABLED' if enabled else 'DISABLED'
        self.get_logger().info(f'ArUco detection {state}')

    def on_image(self, message):
        """Ignore images unless the robot has reached an observation pose."""
        if not self.detection_enabled:
            return
        super().on_image(message)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = GatedDualArucoPosePublisher()
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
