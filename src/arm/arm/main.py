"""ROS 2 node for camera-click based JetCobot pick and place."""

import math
import time

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

from ._config import BAUD, CAMERA, PORT
from ._container_task import pick_clicked_container, place_clicked_position
from ._robot_utils import connect_robot, move_photo_pose


def capture_one_frame(camera_path):
    """Capture one warmed-up frame from an OpenCV camera device."""
    cap = cv2.VideoCapture(camera_path)
    if not cap.isOpened():
        return None

    frame = None
    for _ in range(5):
        ret, candidate = cap.read()
        if ret:
            frame = candidate
        time.sleep(0.05)

    cap.release()
    return frame


class JetCobotClickNode(Node):
    """Drive the JetCobot from left/right clicks on a captured camera frame."""

    def __init__(self):
        super().__init__("jetcobot_click_control")

        self.declare_parameter("camera_path", CAMERA)
        self.declare_parameter("serial_port", PORT)
        self.declare_parameter("baud_rate", BAUD)
        self.declare_parameter("window_name", "captured robot camera")

        self.camera_path = self.get_parameter("camera_path").value
        serial_port = self.get_parameter("serial_port").value
        baud_rate = self.get_parameter("baud_rate").value
        self.window_name = self.get_parameter("window_name").value

        self.get_logger().info(
            f"Connecting to JetCobot: port={serial_port}, baud={baud_rate}"
        )
        self.mc = connect_robot(serial_port, baud_rate)
        move_photo_pose(self.mc)

        self.frame = capture_one_frame(self.camera_path)
        if self.frame is None:
            raise RuntimeError(f"Failed to capture a frame from {self.camera_path}")

        self.robot_busy = False
        self.holding = False
        self._closed = False

        self.joint_state_pub = self.create_publisher(JointState, "/joint_states", 10)
        self.create_timer(1.0, self._publish_joint_states)

        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self._mouse_callback)
        self.create_timer(0.03, self._update_window)

        self.get_logger().info(
            "Left click: pick, right click: place, Q: refresh frame, ESC: quit"
        )

    def _publish_joint_states(self):
        """Publish hardware angles for the URDF robot_state_publisher."""
        if self.robot_busy:
            return

        try:
            angles = self.mc.get_angles()
            if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                self.get_logger().warning(f"Invalid joint angles: {angles}")
                return

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = [
                "1_Joint", "2_Joint", "3_Joint",
                "4_Joint", "5_Joint", "6_Joint",
            ]
            message.position = [math.radians(angle) for angle in angles]
            self.joint_state_pub.publish(message)
        except Exception as exc:
            self.get_logger().error(f"Failed to publish joint states: {exc}")

    def _mouse_callback(self, event, x, y, _flags, _param):
        if self.robot_busy:
            self.get_logger().warning("Robot is already moving")
            return

        if event == cv2.EVENT_LBUTTONDOWN:
            self.robot_busy = True
            try:
                if pick_clicked_container(self.mc, self.frame.copy(), x, y):
                    self.holding = True
                    self._refresh_frame("pick")
            except Exception as exc:
                self.get_logger().error(f"Pick failed: {exc}")
            finally:
                self.robot_busy = False

        elif event == cv2.EVENT_RBUTTONDOWN:
            if not self.holding:
                self.get_logger().warning("Pick a container before placing it")
                return

            self.robot_busy = True
            try:
                if place_clicked_position(self.mc, x, y):
                    self.holding = False
                    self._refresh_frame("place")
            except Exception as exc:
                self.get_logger().error(f"Place failed: {exc}")
            finally:
                self.robot_busy = False

    def _refresh_frame(self, operation):
        self.get_logger().info(f"Capturing a new frame after {operation}")
        new_frame = capture_one_frame(self.camera_path)
        if new_frame is None:
            self.get_logger().error(
                f"Frame refresh failed after {operation}; keeping the previous frame"
            )
            return False
        self.frame = new_frame
        self.get_logger().info("Camera frame refreshed")
        return True

    def _update_window(self):
        display = self.frame.copy()
        lines = (
            "L-click: pick container",
            "R-click: place position",
            "Q: refresh camera frame",
            f"Holding: {'YES' if self.holding else 'NO'}",
        )
        for index, line in enumerate(lines, start=1):
            cv2.putText(
                display, line, (20, 30 * index), cv2.FONT_HERSHEY_SIMPLEX,
                0.7, (0, 255, 255) if index == 4 else (0, 255, 0), 2,
            )

        cv2.imshow(self.window_name, display)
        key = cv2.waitKey(1) & 0xFF
        if key == 27:
            self.get_logger().info("ESC pressed; shutting down")
            rclpy.shutdown()
        elif key in (ord("q"), ord("Q")) and not self.robot_busy:
            self.robot_busy = True
            try:
                self._refresh_frame("manual request")
            finally:
                self.robot_busy = False

    def close(self):
        if not self._closed:
            cv2.destroyAllWindows()
            self._closed = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JetCobotClickNode()
        rclpy.spin(node)
    except (KeyboardInterrupt, RuntimeError) as exc:
        if node is not None:
            node.get_logger().error(str(exc))
        else:
            print(f"JetCobot startup failed: {exc}")
    finally:
        if node is not None:
            node.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
