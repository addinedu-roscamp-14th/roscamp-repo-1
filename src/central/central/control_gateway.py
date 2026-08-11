#!/usr/bin/env python3

"""HTTP-to-ROS gateway for central navigation and telemetry."""

from __future__ import annotations

from collections import deque
import json
import math
import secrets
import threading
import time
import uuid

from geometry_msgs.msg import PointStamped

from nav_msgs.msg import Odometry
from porter_interfaces.action import DispatchArmCommand
from porter_interfaces.msg import ArmState, PixelNavigationCommand, VehicleState

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import (
    ExternalShutdownException,
    SingleThreadedExecutor,
)
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger

from .control_protocol import (
    CommandValidationError,
    validate_park_request,
    validate_pixel_goal,
)


DEFAULT_ARRIVAL_ROI = [0.78, 0.55, 0.99, 0.98]


def _json_safe(value):
    """
    Replace NaN/Infinity floats with None (JSON null).

    Starlette's JSONResponse renders with allow_nan=False, so an
    uninitialized telemetry field (e.g. battery_percent defaults to
    math.nan before a vehicle reports it) would otherwise raise
    ValueError and turn the whole response into a 500.
    """
    if isinstance(value, float):
        return None if not math.isfinite(value) else value
    if isinstance(value, dict):
        return {key: _json_safe(item) for key, item in value.items()}
    if isinstance(value, (list, tuple)):
        return [_json_safe(item) for item in value]
    return value


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
            'fleet_pixel_command_topic',
            '/central/fleet/pixel_navigation_command',
        )
        self.declare_parameter(
            'target_map_json_topic', '/central/target_map_json'
        )
        self.declare_parameter(
            'command_status_topic', '/central/control/status'
        )
        self.declare_parameter(
            'park_request_topic', '/central/fleet/park_request'
        )
        self.declare_parameter('battery_percent_topic', '/battery/percent')
        self.declare_parameter('battery_voltage_topic', '/battery/voltage')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('camera_frame_id', 'camera')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('minimum_heading_distance_px', 10.0)
        self.declare_parameter('telemetry_stale_timeout_sec', 2.0)
        self.declare_parameter(
            'arrival_roi_normalized', DEFAULT_ARRIVAL_ROI
        )

        self.host = str(self.get_parameter('host').value)
        self.port = int(self.get_parameter('port').value)
        self.api_token = str(self.get_parameter('api_token').value)
        self.target_pixel_topic = str(
            self.get_parameter('target_pixel_topic').value
        )
        self.fleet_pixel_command_topic = str(
            self.get_parameter('fleet_pixel_command_topic').value
        )
        self.target_map_json_topic = str(
            self.get_parameter('target_map_json_topic').value
        )
        self.command_status_topic = str(
            self.get_parameter('command_status_topic').value
        )
        self.park_request_topic = str(
            self.get_parameter('park_request_topic').value
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
            'vehicles': {},
            'arms': {},
            'last_arm_result': None,
            'arrival_roi': list(
                self.get_parameter('arrival_roi_normalized').value
            ),
            'b1_zone': 'B-1:UNKNOWN',
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
        self.fleet_pixel_publisher = self.create_publisher(
            PixelNavigationCommand,
            self.fleet_pixel_command_topic,
            10,
        )
        self.status_publisher = self.create_publisher(
            String,
            self.command_status_topic,
            10,
        )
        config_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.arrival_roi_publisher = self.create_publisher(
            String, '/central/autonomy/arrival_roi_config', config_qos
        )
        self.park_request_publisher = self.create_publisher(
            String,
            self.park_request_topic,
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
        for vehicle_id in ('agv1', 'agv2'):
            self.create_subscription(
                VehicleState,
                f'/central/fleet/{vehicle_id}/state',
                self._on_vehicle_state,
                10,
            )
        self.create_subscription(
            String,
            '/central/fleet/zones',
            self._on_zone_state,
            10,
        )
        for arm_id in ('arm1', 'arm2'):
            self.create_subscription(
                ArmState,
                f'/central/arms/{arm_id}/state',
                self._on_arm_state,
                10,
            )
        self.create_subscription(
            String,
            '/central/arms/results',
            self._on_arm_result,
            10,
        )
        self._arm_dispatch_client = ActionClient(
            self,
            DispatchArmCommand,
            '/central/arms/dispatch',
        )
        self._arm_stop_clients = {
            arm_id: self.create_client(
                Trigger, f'/central/arms/{arm_id}/stop'
            )
            for arm_id in ('arm1', 'arm2')
        }
        self._fleet_emergency_client = self.create_client(
            SetBool, '/central/fleet/emergency_stop'
        )
        self._vehicle_emergency_clients = {
            vehicle_id: self.create_client(
                SetBool,
                f'/central/fleet/{vehicle_id}/emergency_stop',
            )
            for vehicle_id in ('agv1', 'agv2')
        }
        self._clear_zone_clients = {
            'B-1': self.create_client(
                Trigger, '/central/fleet/clear_b1_lock'
            ),
            'A': self.create_client(
                Trigger, '/central/fleet/clear_a_lock'
            ),
        }

        self.get_logger().info(
            f'Central control API: http://{self.host}:{self.port}'
        )
        self.publish_arrival_roi(
            list(self.get_parameter('arrival_roi_normalized').value)
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

    def _on_vehicle_state(self, message):
        position = message.pose.pose.position
        value = {
            'vehicle_id': message.vehicle_id,
            'state': message.state_text,
            'battery_percent': float(message.battery_percent),
            'battery_voltage': float(message.battery_voltage),
            'pose': {
                'x': float(position.x),
                'y': float(position.y),
            },
            'current_command_id': message.current_command_id,
            'nav2_ready': bool(message.nav2_ready),
            'emergency_stopped': bool(message.emergency_stopped),
            'locked_zone': message.locked_zone,
            'telemetry_age_sec': float(message.telemetry_age_sec),
        }
        with self._lock:
            self._telemetry['vehicles'][message.vehicle_id] = value

    def _on_zone_state(self, message):
        with self._lock:
            self._telemetry['b1_zone'] = message.data

    def _on_arm_state(self, message):
        value = {
            'arm_id': message.arm_id,
            'state': int(message.state),
            'state_text': message.state_text,
            'ready': bool(message.ready),
            'current_command_id': message.current_command_id,
            'current_mission_id': message.current_mission_id,
            'current_operation': message.current_operation,
            'operation_id': message.operation_id,
            'phase': message.phase,
            'progress': float(message.progress),
            'last_error': message.last_error,
            'telemetry_age_sec': float(message.telemetry_age_sec),
        }
        with self._lock:
            self._telemetry['arms'][message.arm_id] = value

    def _on_arm_result(self, message):
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError:
            result = {'raw': message.data}
        with self._lock:
            self._telemetry['last_arm_result'] = result

    def publish_arrival_roi(self, values):
        if len(values) != 4:
            raise CommandValidationError('ROI requires four normalized values')
        x_min, y_min, x_max, y_max = (float(value) for value in values)
        if not (
            0.0 <= x_min < x_max <= 1.0
            and 0.0 <= y_min < y_max <= 1.0
        ):
            raise CommandValidationError(
                'ROI must satisfy 0<=x_min<x_max<=1 and '
                '0<=y_min<y_max<=1'
            )
        roi = [x_min, y_min, x_max, y_max]
        message = String()
        message.data = json.dumps({
            'x_min': x_min,
            'y_min': y_min,
            'x_max': x_max,
            'y_max': y_max,
        })
        self.arrival_roi_publisher.publish(message)
        with self._lock:
            self._telemetry['arrival_roi'] = roi
        return {'accepted': True, 'roi_normalized': roi}

    def dispatch_arm_command(self, payload, timeout_sec=3.0):
        """Queue an ARM action and return after the action server accepts it."""
        command_id = str(payload.get('command_id') or uuid.uuid4())
        arm_id = str(payload.get('arm_id') or 'arm2').lower()
        operation = str(payload.get('operation') or '').lower()
        if arm_id not in {'arm1', 'arm2'}:
            raise CommandValidationError('arm_id must be arm1 or arm2')
        if not operation:
            raise CommandValidationError('operation is required')
        with self._dispatch_lock:
            if command_id in self._recent_command_ids:
                return {
                    'accepted': True,
                    'duplicate': True,
                    'command_id': command_id,
                }
            if not self._arm_dispatch_client.wait_for_server(
                timeout_sec=0.5
            ):
                raise RuntimeError('central ARM dispatcher is unavailable')
            goal = DispatchArmCommand.Goal()
            goal.command_id = command_id
            goal.mission_id = str(payload.get('mission_id') or '')
            goal.arm_id = arm_id
            goal.operation = operation
            goal.destination_slot = str(payload.get('destination_slot') or '')
            goal.source_id = int(payload.get('source_id', -1))
            goal.destination_id = int(payload.get('destination_id', -1))
            goal.vehicle_id = str(payload.get('vehicle_id') or '')
            goal.final_for_vehicle = bool(
                payload.get('final_for_vehicle', False)
            )
            future = self._arm_dispatch_client.send_goal_async(goal)
            deadline = time.monotonic() + timeout_sec
            while not future.done() and time.monotonic() < deadline:
                time.sleep(0.01)
            if not future.done():
                raise RuntimeError('ARM dispatcher goal acceptance timed out')
            goal_handle = future.result()
            if goal_handle is None or not goal_handle.accepted:
                raise RuntimeError('ARM dispatcher rejected the command')
            result_future = goal_handle.get_result_async()
            result_future.add_done_callback(
                lambda done, cid=command_id: self._log_arm_result(cid, done)
            )
            self._recent_command_ids.append(command_id)
        return {
            'accepted': True,
            'duplicate': False,
            'command_id': command_id,
            'mission_id': goal.mission_id,
            'arm_id': arm_id,
            'operation': operation,
            'queued': True,
        }

    def _log_arm_result(self, command_id, future):
        try:
            wrapped = future.result()
            result = wrapped.result
            level = self.get_logger().info if result.success else self.get_logger().error
            level(
                f'ARM command {command_id} completed: '
                f'success={result.success}, message={result.message}'
            )
        except Exception as exc:
            self.get_logger().error(
                f'ARM command {command_id} result failed: {exc}'
            )

    def stop_arm(self, arm_id, timeout_sec=3.0):
        arm_id = str(arm_id).lower()
        if arm_id not in self._arm_stop_clients:
            raise CommandValidationError('arm_id must be arm1 or arm2')
        client = self._arm_stop_clients[arm_id]
        if not client.wait_for_service(timeout_sec=0.5):
            raise RuntimeError(
                f'central {arm_id.upper()} stop service is unavailable'
            )
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f'{arm_id.upper()} stop timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return {
            'accepted': True,
            'arm_id': arm_id,
            'message': response.message,
        }

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
            fleet_subscribers = (
                self.fleet_pixel_publisher.get_subscription_count()
            )
            legacy_subscribers = (
                self.target_pixel_publisher.get_subscription_count()
            )
            if fleet_subscribers == 0 and legacy_subscribers == 0:
                raise RuntimeError(
                    'no subscriber on fleet or legacy pixel command topics; '
                    'start camera_to_map_bridge first'
                )

            stamp = self.get_clock().now().to_msg()
            if fleet_subscribers > 0:
                message = PixelNavigationCommand()
                message.header.stamp = stamp
                message.header.frame_id = self.camera_frame_id
                message.command_id = command_id
                message.predecessor_command_id = (
                    goal.predecessor_command_id
                )
                message.requested_vehicle_id = goal.requested_vehicle_id
                message.zone_id = goal.zone_id
                message.mode = goal.mode
                message.queue_if_busy = goal.queue_if_busy
                message.zone_visually_empty = goal.zone_visually_empty
                message.target_pixel.x = goal.target.x
                message.target_pixel.y = goal.target.y
                message.heading_pixel.x = goal.heading.x
                message.heading_pixel.y = goal.heading.y
                self.fleet_pixel_publisher.publish(message)
            else:
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
            'predecessor_command_id': goal.predecessor_command_id,
            'queue_if_busy': goal.queue_if_busy,
            'mode': goal.mode,
            'requested_vehicle_id': goal.requested_vehicle_id or 'AUTO',
            'zone_id': goal.zone_id,
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
            'transport': 'fleet' if fleet_subscribers > 0 else 'legacy',
        }

    def set_emergency(self, vehicle_id, enabled, timeout_sec=2.0):
        if vehicle_id in ('', 'all', 'fleet'):
            client = self._fleet_emergency_client
            target = 'fleet'
        elif vehicle_id in self._vehicle_emergency_clients:
            client = self._vehicle_emergency_clients[vehicle_id]
            target = vehicle_id
        else:
            raise ValueError('vehicle_id must be agv1, agv2, or fleet')
        if not client.wait_for_service(timeout_sec=0.25):
            raise RuntimeError(f'{target} emergency service is unavailable')
        request = SetBool.Request()
        request.data = bool(enabled)
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f'{target} emergency service timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return {'accepted': True, 'target': target, 'enabled': bool(enabled)}

    _ZONE_URL_ALIASES = {'b1': 'B-1', 'a': 'A'}

    def clear_zone_lock(self, zone_id, timeout_sec=2.0):
        normalized = self._ZONE_URL_ALIASES.get(
            zone_id.lower(), zone_id.upper()
        )
        client = self._clear_zone_clients.get(normalized)
        if client is None:
            raise ValueError(f'unknown zone_id: {zone_id}')
        if not client.wait_for_service(timeout_sec=0.25):
            raise RuntimeError(f'{normalized} clear service is unavailable')
        future = client.call_async(Trigger.Request())
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.01)
        if not future.done():
            raise RuntimeError(f'{normalized} clear service timed out')
        response = future.result()
        if not response.success:
            raise RuntimeError(response.message)
        return {
            'accepted': True,
            'zone_id': normalized,
            'message': response.message,
        }

    def dispatch_park(self, payload):
        """Validate and publish one auto-park request exactly once."""
        request = validate_park_request(payload)
        command_id = request.command_id or str(uuid.uuid4())

        with self._dispatch_lock:
            if command_id in self._recent_command_ids:
                return {
                    'accepted': True,
                    'duplicate': True,
                    'command_id': command_id,
                }
            if self.park_request_publisher.get_subscription_count() == 0:
                raise RuntimeError(
                    'no subscriber on the park request topic; '
                    'start fleet_dispatcher first'
                )
            message = String()
            message.data = json.dumps({
                'vehicle_id': request.requested_vehicle_id,
                'predecessor_command_id': request.predecessor_command_id,
            }, ensure_ascii=False)
            self.park_request_publisher.publish(message)
            self._recent_command_ids.append(command_id)

        self.get_logger().info(
            f'Accepted park request {command_id}: '
            f'vehicle_id={request.requested_vehicle_id or "AUTO"}'
        )
        return {
            'accepted': True,
            'duplicate': False,
            'command_id': command_id,
            'vehicle_id': request.requested_vehicle_id or 'AUTO',
            'predecessor_command_id': request.predecessor_command_id,
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
        return _json_safe({
            'status': 'ready',
            'interfaces': {
                'target_pixel_topic': self.target_pixel_topic,
                'target_pixel_subscribers': (
                    self.target_pixel_publisher.get_subscription_count()
                ),
                'fleet_pixel_command_topic': self.fleet_pixel_command_topic,
                'fleet_pixel_subscribers': (
                    self.fleet_pixel_publisher.get_subscription_count()
                ),
                'command_status_topic': self.command_status_topic,
            },
            'telemetry': telemetry,
            'age_sec': ages,
            'stale': stale,
        })


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

    @app.post('/api/v1/navigation/park')
    async def navigation_park(
        payload: dict,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.dispatch_park(payload)
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

    @app.post('/api/v1/arms/commands')
    async def arm_command(
        payload: dict,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.dispatch_arm_command(payload)
        except CommandValidationError as exc:
            raise http_exception_class(status_code=422, detail=str(exc)) from exc
        except (RuntimeError, TypeError, ValueError) as exc:
            raise http_exception_class(status_code=503, detail=str(exc)) from exc

    @app.post('/api/v1/arms/{arm_id}/stop')
    async def arm_stop(
        arm_id: str,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.stop_arm(arm_id)
        except CommandValidationError as exc:
            raise http_exception_class(status_code=422, detail=str(exc)) from exc
        except RuntimeError as exc:
            raise http_exception_class(status_code=503, detail=str(exc)) from exc

    @app.put('/api/v1/autonomy/arrival-roi')
    async def update_arrival_roi(
        payload: dict,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.publish_arrival_roi([
                payload['x_min'], payload['y_min'],
                payload['x_max'], payload['y_max'],
            ])
        except (KeyError, TypeError, ValueError, CommandValidationError) as exc:
            raise http_exception_class(status_code=422, detail=str(exc)) from exc

    @app.post('/api/v1/emergency-stop')
    async def emergency_stop(
        payload: dict,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.set_emergency(
                str(payload.get('vehicle_id', 'fleet')),
                bool(payload.get('enabled', True)),
            )
        except (RuntimeError, ValueError) as exc:
            raise http_exception_class(
                status_code=503,
                detail=str(exc),
            ) from exc

    @app.post('/api/v1/zones/{zone_id}/clear')
    async def clear_zone(
        zone_id: str,
        x_control_token: str | None = header_factory(default=None),
    ):
        authorize(x_control_token)
        try:
            return node.clear_zone_lock(zone_id)
        except ValueError as exc:
            raise http_exception_class(
                status_code=404,
                detail=str(exc),
            ) from exc
        except RuntimeError as exc:
            raise http_exception_class(
                status_code=409,
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
