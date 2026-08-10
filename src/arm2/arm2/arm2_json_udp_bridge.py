"""Forward arm2 JSON transfer events to a remote computer over UDP."""

import json
import socket

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


OPERATION_NAMES = {
    'transfer': '컨테이너를 적재 구역에',
    'id_transfer': '컨테이너를 컨테이너에',
    'trailer_load': '컨테이너를 트레일러에',
}

TOP_LEVEL_OPERATIONS = set(OPERATION_NAMES)
TERMINAL_PHASES = {'COMPLETED', 'STOPPED', 'FAILED'}


def is_terminal_event(event):
    """Return whether one of the three top-level sequences has ended."""
    return (
        event.get('operation') in TOP_LEVEL_OPERATIONS
        and event.get('phase') in TERMINAL_PHASES
    )


def localized_status(event):
    """Return one of the only two statuses exposed to remote computers."""
    return '실패' if event.get('phase') == 'FAILED' else '성공'


def localize_event(event):
    """Create the Korean, arm2-prefixed payload sent to the remote PC."""
    operation = OPERATION_NAMES.get(
        event.get('operation'), str(event.get('operation') or '알 수 없는 명령')
    )
    return {
        '로봇': 'arm2',
        '명령어': f'arm2 {operation}',
        '메시지': event.get('message'),
        '상태': localized_status(event),
    }


class Arm2JsonUdpBridge(Node):
    """Translate transfer events for Korean display and forward them."""

    def __init__(self):
        super().__init__('arm2_json_udp_bridge')
        self.declare_parameter('remote_host', '127.0.0.1')
        self.declare_parameter('remote_port', 15002)
        self.remote_host = str(self.get_parameter('remote_host').value)
        self.remote_port = int(self.get_parameter('remote_port').value)
        if not 1 <= self.remote_port <= 65535:
            raise ValueError('remote_port must be between 1 and 65535')
        self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sent_operation_ids = set()
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
        """Translate an event and send its UTF-8 JSON datagram."""
        try:
            event = json.loads(message.data)
            if not is_terminal_event(event):
                return
            operation_id = event.get('operation_id')
            if operation_id and operation_id in self.sent_operation_ids:
                return
            payload = json.dumps(
                localize_event(event), ensure_ascii=False, separators=(',', ':')
            )
            payload = f'{payload}\n'.encode('utf-8')
            self.socket.sendto(payload, (self.remote_host, self.remote_port))
            if operation_id:
                self.sent_operation_ids.add(operation_id)
                if len(self.sent_operation_ids) > 1000:
                    self.sent_operation_ids.clear()
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
