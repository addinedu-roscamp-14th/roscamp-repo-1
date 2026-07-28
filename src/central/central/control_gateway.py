#!/usr/bin/env python3

"""HTTP-to-ROS gateway for central navigation and telemetry."""

from __future__ import annotations

import json
import math
import secrets
import threading
import time
import uuid
from collections import deque

from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry

import rclpy
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
)
from rclpy.node import Node

from std_msgs.msg import Float32, String

from .control_protocol import (
    CommandValidationError,
    validate_pixel_goal,
)


def _web_dependencies():
    try:
        from fastapi import FastAPI, Header, HTTPException
        import uvicorn
    except ImportError as exc:
        raise RuntimeError(
            'FastAPI dependencies are missing. Install: '
            'sudo apt install python3-fastapi python3-uvicorn'
        ) from exc
    return FastAPI, Header, HTTPException, uvicorn


class CentralControlGateway(Node):
    """Accept validated AI commands and expose current ROS telemetry."""

    def __init__(self):
        """Configure ROS interfaces and HTTP gateway parameters."""
        super().__init__('central_control_gateway')
        self.declare_parameter('host', '127.0.0.1')
        self.declare_parameter('port', 8100)
        self.declare_parameter('api_token', '')
        self.declare_parameter(
            'target_pixel_topic', '/central/target_pixel'
        )
        self.declare_parameter(
            'target_map_json_topic', '/central/target_map_json'
        )
        self.declare_parameter(
            'command_status_topic', '/central/control/status'
        )
        self.declare_parameter('battery_percent_topic', '/battery/percent')
        self.declare_parameter('battery_voltage_topic', '/battery/voltage')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('minimum_heading_distance_px', 10.0)
        self.declare_parameter('telemetry_stale_timeout_sec', 2.0)

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.api_token = str(self.get_parameter('api_token').value)
        self.target_pixel_topic = str(
            self.get_parameter('target_pixel_topic').value
        )
        self.target_map_json_topic = str(
            self.get_parameter('target_map_json_topic').value
        )
        self.command_status_topic = str(
            self.get_parameter('command_status_topic').value
        )
        self.camera_frame_id = str(
            self.get_parameter('camera_frame_id').value
        )
        self.image_width = int(self.get_parameter('image_width').value)
        self.image_height = int(self.get_parameter('image_height').value)
        self.minimum_heading_distance_px = float(
            self.get_parameter('minimum_heading_distance_px').value
        )
        self.telemetry_stale_timeout = float(
            self.get_parameter('telemetry_stale_timeout_sec').value
        )
        self._validate_parameters()

        self._lock = threading.Lock()
        self._dispatch_lock = threading.Lock()
        self._recent_command_ids = deque(maxlen=200)
        self._telemetry = {
            'battery_percent': None,
            'battery_voltage': None,
            'odom': None,
            'last_map_target': None,
            'last_command': None,
        }
        self._received_at = {
            'battery_percent': None,
            'battery_voltage': None,
            'odom': None,
            'last_map_target': None,
        }

        self.target_pixel_publisher = self.create_publisher(
            PointStamped,
            self.target_pixel_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            self.command_status_topic,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('battery_percent_topic').value),
            self._on_battery_percent,
            10,
        )
        self.create_subscription(
            Float32,
            str(self.get_parameter('battery_voltage_topic').value),
            self._on_battery_voltage,
            10,
        )
        self.create_subscription(
            Odometry,
            str(self.get_parameter('odom_topic').value),
            self._on_odom,
            10,
        )
        self.create_subscription(
            String,
            self.target_map_json_topic,
            self._on_map_target,
            10,
        )

        self.get_logger().info(
            f'Central control API: http://{self.host}:{self.port}'
        )
        self.get_logger().info(
            f'AI pixel goals -> {self.target_pixel_topic} '
            f'({self.image_width}x{self.image_height})'
        )

    def _validate_parameters(self):
        if not 1 <= self.port <= 65535:
            raise ValueError('port must be within 1..65535')
        if self.host not in ('127.0.0.1', 'localhost', '::1'):
            if not self.api_token:
                raise ValueError(
                    'api_token is required when host is not loopback'
                )
        if self.image_width <= 0 or self.image_height <= 0:
            raise ValueError('image dimensions must be positive')
        if self.minimum_heading_distance_px <= 0.0:
            raise ValueError(
                'minimum_heading_distance_px must be positive'
            )
        if self.telemetry_stale_timeout <= 0.0:
            raise ValueError('telemetry_stale_timeout_sec must be positive')

    def _set_telemetry(self, key, value):
        with self._lock:
            self._telemetry[key] = value
            self._received_at[key] = time.monotonic()

    def _on_battery_percent(self, message):
        self._set_telemetry('battery_percent', float(message.data))

    def _on_battery_voltage(self, message):
        self._set_telemetry('battery_voltage', float(message.data))

    def _on_odom(self, message):
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        siny = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        self._set_telemetry(
            'odom',
            {
                'x': float(position.x),
                'y': float(position.y),
                'yaw_deg': math.degrees(math.atan2(siny, cosy)),
            },
        )

    def _on_map_target(self, message):
        try:
            value = json.loads(message.data)
        except json.JSONDecodeError:
            value = {'raw': message.data}
        self._set_telemetry('last_map_target', value)

    def dispatch_pixel_goal(self, payload):
        """Validate and publish a target/heading pixel pair exactly once."""
        goal = validate_pixel_goal(
            payload,
            self.image_width,
            self.image_height,
            self.minimum_heading_distance_px,
        )
        command_id = goal.command_id or str(uuid.uuid4())

        with self._dispatch_lock:
            if command_id in self._recent_command_ids:
                return {
                    'accepted': True,
                    'duplicate': True,
                    'command_id': command_id,
                }
            subscribers = (
                self.target_pixel_publisher.get_subscription_count()
            )
            if subscribers == 0:
                raise RuntimeError(
                    f'no subscriber on {self.target_pixel_topic}; '
                    'start camera_to_map_bridge first'
                )

            stamp = self.get_clock().now().to_msg()
            self.target_pixel_publisher.publish(
                self._point_message(
                    goal.target.x,
                    goal.target.y,
                    stamp,
                    goal.mode,
                )
            )
            self.target_pixel_publisher.publish(
                self._point_message(
                    goal.heading.x,
                    goal.heading.y,
                    stamp,
                    goal.mode,
                )
            )
            self._recent_command_ids.append(command_id)

        command = {
            'state': 'PIXEL_GOAL_PUBLISHED',
            'command_id': command_id,
            'mode': goal.mode,
            'target': {'x': goal.target.x, 'y': goal.target.y},
            'heading': {'x': goal.heading.x, 'y': goal.heading.y},
            'target_pixel_topic': self.target_pixel_topic,
        }
        with self._lock:
            self._telemetry['last_command'] = command
        status = String()
        status.data = json.dumps(command, ensure_ascii=False)
        self.status_publisher.publish(status)
        self.get_logger().info(
            f'Accepted AI goal {command_id}: '
            f'mode={goal.mode}, '
            f'target=({goal.target.x:.1f}, {goal.target.y:.1f}), '
            f'heading=({goal.heading.x:.1f}, {goal.heading.y:.1f})'
        )
        return {
            'accepted': True,
            'duplicate': False,
            'command_id': command_id,
            'published_points': 2,
        }

    def _point_message(self, x, y, stamp, mode='direct'):
        message = PointStamped()
        message.header.stamp = stamp
        message.header.frame_id = (
            f'{self.camera_frame_id}/parking_b1'
            if mode == 'parking_b1'
            else self.camera_frame_id
        )
        message.point.x = x
        message.point.y = y
        return message

    def status(self):
        """Return the latest command, telemetry, and interface health."""
        now = time.monotonic()
        with self._lock:
            telemetry = dict(self._telemetry)
            received_at = dict(self._received_at)

        ages = {}
        for key, received in received_at.items():
            ages[key] = (
                None if received is None else round(now - received, 3)
            )
        stale = {
            key: age is None or age > self.telemetry_stale_timeout
            for key, age in ages.items()
        }
        return {
            'status': 'ready',
            'interfaces': {
                'target_pixel_topic': self.target_pixel_topic,
                'target_pixel_subscribers': (
                    self.target_pixel_publisher.get_subscription_count()
                ),
                'command_status_topic': self.command_status_topic,
            },
            'telemetry': telemetry,
            'age_sec': ages,
            'stale': stale,
        }


def create_app(
    node,
    fastapi_class,
    header_factory,
    http_exception_class,
):
    """Create the central-control HTTP API."""
    app = fastapi_class(title='Port-ER Central Control API')

    def authorize(token):
        if not node.api_token:
            return
        if token is None or not secrets.compare_digest(
            token,
            node.api_token,
        ):
            raise http_exception_class(
                status_code=401,
                detail='invalid control token',
            )

    @app.get('/health')
    async def health():
        status = node.status()
        return {
            'status': status['status'],
            'target_pixel_subscribers': status['interfaces'][
                'target_pixel_subscribers'
            ],
        }

    @app.get('/api/v1/status')
    async def status(
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        return node.status()

    @app.post('/api/v1/navigation/pixel-goal')
    async def navigation_pixel_goal(
        payload: dict,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.dispatch_pixel_goal(payload)
        except CommandValidationError as exc:
            raise http_exception_class(
                status_code=422,
                detail=str(exc),
            ) from exc
        except RuntimeError as exc:
            raise http_exception_class(
                status_code=503,
                detail=str(exc),
            ) from exc

    return app


def main(args=None):
    """Run the ROS executor and FastAPI server together."""
    FastAPI, Header, HTTPException, uvicorn = _web_dependencies()
    rclpy.init(args=args)
    node = CentralControlGateway()
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    ros_thread = threading.Thread(
        target=executor.spin,
        name='central-control-ros-executor',
        daemon=True,
    )
    ros_thread.start()
    app = create_app(node, FastAPI, Header, HTTPException)
    config = uvicorn.Config(
        app,
        host=node.host,
        port=node.port,
        log_level='info',
        access_log=True,
        lifespan='off',
    )
    server = uvicorn.Server(config)
    try:
        server.run()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        ros_thread.join(timeout=2.0)


if __name__ == '__main__':
    main()
