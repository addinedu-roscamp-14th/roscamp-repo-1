"""Forward arm2 JSON transfer events to a remote computer over UDP."""

import json
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class Arm2JsonUdpBridge(Node):
    """Validate and forward transfer event JSON without modifying it."""

    def __init__(self):
        super().__init__('arm2_json_udp_bridge')
        self.declare_parameter('remote_host', '127.0.0.1')
        self.declare_parameter('remote_port', 15002)
        self.remote_host = str(self.get_parameter('remote_host').value)
        self.remote_port = int(self.get_parameter('remote_port').value)
        if not 1 <= self.remote_port <= 65535:
            raise ValueError('remote_port must be between 1 and 65535')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.subscription = self.create_subscription(
            String, '/arm2/transfer_events', self.forward_event, qos
        )
        self.get_logger().info(
            f'Forwarding /arm2/transfer_events to '
            f'{self.remote_host}:{self.remote_port}/udp'
        )

    def forward_event(self, message):
        """Validate an event and send its UTF-8 JSON datagram."""
        try:
            json.loads(message.data)
            payload = message.data.encode('utf-8')
            self.socket.sendto(payload, (self.remote_host, self.remote_port))
        except (OSError, UnicodeError, json.JSONDecodeError) as exc:
            self.get_logger().error(f'JSON UDP forwarding failed: {exc}')

    def destroy_node(self):
        self.socket.close()
        return super().destroy_node()


def main(args=None):
    """Run the JSON UDP bridge."""
    rclpy.init(args=args)
    node = Arm2JsonUdpBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
