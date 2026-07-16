"""Publish measured JetCobot joint angles for robot_state_publisher."""

import math

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from sensor_msgs.msg import JointState

from ._config import BAUD, PORT
from ._robot_utils import connect_robot


class HardwareJointStatePublisher(Node):
    """Read six hardware joints and publish them in radians."""

    JOINT_NAMES = [
        '1_Joint',
        '2_Joint',
        '3_Joint',
        '4_Joint',
        '5_Joint',
        '6_Joint',
    ]

    def __init__(self):
        """Connect to the robot and configure periodic state publication."""
        super().__init__('hardware_joint_state_publisher')

        self.declare_parameter('serial_port', PORT)
        self.declare_parameter('baud_rate', BAUD)
        self.declare_parameter('publish_rate', 10.0)
        self.declare_parameter('joint_states_topic', '/arm2/joint_states')

        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        publish_rate = float(self.get_parameter('publish_rate').value)
        joint_states_topic = str(
            self.get_parameter('joint_states_topic').value
        )
        if publish_rate <= 0.0:
            raise ValueError('publish_rate must be greater than zero')

        self.robot = connect_robot(serial_port, baud_rate)
        self.publisher = self.create_publisher(
            JointState, joint_states_topic, 10
        )
        self.create_timer(1.0 / publish_rate, self._publish_joint_states)
        self.get_logger().info(
            f'Publishing measured joints: {joint_states_topic} at '
            f'{publish_rate:g} Hz'
        )

    def _publish_joint_states(self):
        try:
            angles = self.robot.get_angles()
            if not isinstance(angles, (list, tuple)) or len(angles) != 6:
                self.get_logger().warning(f'Invalid joint angles: {angles}')
                return

            message = JointState()
            message.header.stamp = self.get_clock().now().to_msg()
            message.name = self.JOINT_NAMES
            message.position = [math.radians(float(value)) for value in angles]
            message.velocity = [0.0] * 6
            message.effort = [0.0] * 6
            self.publisher.publish(message)
        except Exception as exc:
            self.get_logger().error(f'Failed to read joint angles: {exc}')


def main(args=None):
    """Run the hardware joint state publisher node."""
    rclpy.init(args=args)
    node = None
    try:
        node = HardwareJointStatePublisher()
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
