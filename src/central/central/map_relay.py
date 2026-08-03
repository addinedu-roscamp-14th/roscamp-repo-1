#!/usr/bin/env python3

"""
Relay the available AGV map and keepout mask onto fleet-wide topics.

This keeps one central RViz display populated when only one vehicle is
connected.
"""

from __future__ import annotations

from map_msgs.msg import OccupancyGridUpdate
from nav_msgs.msg import OccupancyGrid
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


def _map_qos():
    # Matches nav2_map_server's default map QoS (reliable + transient_local)
    # so a late-joining subscriber (e.g. RViz opened after the map is
    # published) still receives the latched last message.
    return QoSProfile(
        reliability=ReliabilityPolicy.RELIABLE,
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        durability=DurabilityPolicy.TRANSIENT_LOCAL,
    )


class MapRelay(Node):
    """Forward each vehicle's map/keepout-mask topics onto shared ones."""

    def __init__(self):
        super().__init__('map_relay')
        self.declare_parameter('vehicle_ids', ['agv1', 'agv2'])
        self.declare_parameter('map_topic', 'map')
        self.declare_parameter('map_updates_topic', 'map_updates')
        self.declare_parameter('keepout_mask_topic', 'keepout_filter_mask')
        self.declare_parameter('output_namespace', '/central/fleet')

        vehicle_ids = [
            str(value) for value in self.get_parameter('vehicle_ids').value
        ]
        map_topic = str(self.get_parameter('map_topic').value)
        map_updates_topic = str(
            self.get_parameter('map_updates_topic').value
        )
        keepout_mask_topic = str(
            self.get_parameter('keepout_mask_topic').value
        )
        output_namespace = str(
            self.get_parameter('output_namespace').value
        ).rstrip('/')

        qos = _map_qos()
        self.map_pub = self.create_publisher(
            OccupancyGrid, f'{output_namespace}/shared_map', qos
        )
        self.map_updates_pub = self.create_publisher(
            OccupancyGridUpdate,
            f'{output_namespace}/shared_map_updates',
            qos,
        )
        self.keepout_pub = self.create_publisher(
            OccupancyGrid, f'{output_namespace}/shared_keepout_mask', qos
        )

        for vehicle_id in vehicle_ids:
            self.create_subscription(
                OccupancyGrid,
                f'/{vehicle_id}/{map_topic}',
                self.map_pub.publish,
                qos,
            )
            self.create_subscription(
                OccupancyGridUpdate,
                f'/{vehicle_id}/{map_updates_topic}',
                self.map_updates_pub.publish,
                qos,
            )
            self.create_subscription(
                OccupancyGrid,
                f'/{vehicle_id}/{keepout_mask_topic}',
                self.keepout_pub.publish,
                qos,
            )

        self.get_logger().info(
            'Relaying map/keepout mask from '
            f'{", ".join(vehicle_ids)} -> {output_namespace}/shared_*'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapRelay()
    try:
        rclpy.spin(node)
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()
        else:
            node.destroy_node()


if __name__ == '__main__':
    main()
