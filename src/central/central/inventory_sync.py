#!/usr/bin/env python3

"""ROS node owning all writes from normalized ARM movements to inventory."""

from __future__ import annotations

import json
import os

from central.inventory_sync_core import (
    InventorySynchronizer,
    PostgresInventoryWriter,
    SQLiteMovementOutbox,
)
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


def _psycopg_connect(**kwargs):
    import psycopg2
    return psycopg2.connect(**kwargs)


class InventorySyncNode(Node):
    """Persist movement events and expose a latched synchronization gate."""

    def __init__(self):
        super().__init__('inventory_sync')
        self.declare_parameter(
            'movement_topic', '/central/inventory/movements'
        )
        self.declare_parameter(
            'status_topic', '/central/inventory/sync_status'
        )
        self.declare_parameter('retry_interval_sec', 2.0)
        outbox_path = os.environ.get(
            'PORT_INVENTORY_OUTBOX_PATH',
            os.path.expanduser('~/.local/state/port_control/inventory.sqlite3'),
        )
        self.synchronizer = InventorySynchronizer(
            SQLiteMovementOutbox(outbox_path),
            PostgresInventoryWriter.from_environment(_psycopg_connect),
        )
        qos = QoSProfile(
            depth=100,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.publisher = self.create_publisher(
            String, str(self.get_parameter('status_topic').value), qos
        )
        self.create_subscription(
            String,
            str(self.get_parameter('movement_topic').value),
            self._on_movement,
            qos,
        )
        self.create_timer(
            max(0.5, float(self.get_parameter('retry_interval_sec').value)),
            self._retry,
        )
        self._publish_status()

    def _on_movement(self, message):
        try:
            event = json.loads(message.data)
            self.synchronizer.submit(event)
        except Exception as exc:
            self.synchronizer.last_error = str(exc)
            self.get_logger().error(f'Inventory movement rejected: {exc}')
        self._publish_status()

    def _retry(self):
        self.synchronizer.flush()
        self._publish_status()

    def _publish_status(self):
        message = String()
        message.data = json.dumps(
            self.synchronizer.status().to_dict(), ensure_ascii=False
        )
        self.publisher.publish(message)

    def destroy_node(self):
        self.synchronizer.outbox.close()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = InventorySyncNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
