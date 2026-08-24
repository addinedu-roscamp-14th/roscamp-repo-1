#!/usr/bin/env python3

"""Queue robot-arm commands and correlate them with structured ARM events."""

from __future__ import annotations

from collections import deque
from datetime import datetime, timezone
import json
import threading
import time

from arm2_interfaces.srv import TransferById, TransferToSlot
from porter_interfaces.action import DispatchArmCommand
from porter_interfaces.msg import ArmState, VehicleState
from porter_interfaces.srv import ExecutePickPlace
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


TERMINAL_PHASES = {'COMPLETED', 'FAILED', 'STOPPED'}
EVENT_OPERATIONS = {
    'scan_destinations': 'destination_scan',
    'transfer_to_slot': 'transfer',
    'load_to_trailer': 'trailer_load',
    'transfer_by_id': 'id_transfer',
}
ARM1_TERMINAL_STATES = {'WORK_COMPLETED', 'FAILED', 'STOPPED'}
ARM1_PROGRESS = {
    'IDLE': 0.0,
    'WORK_STARTED': 5.0,
    'SEARCHING': 15.0,
    'PICK_STARTED': 30.0,
    'PICK_COMPLETED': 50.0,
    'PLACE_STARTED': 65.0,
    'PLACE_COMPLETED': 90.0,
    'WORK_COMPLETED': 100.0,
    'STOP_REQUESTED': 0.0,
    'STOPPED': 0.0,
    'FAILED': 0.0,
}
MARKER_LOCATIONS = {
    9: 'AMR2', 10: 'AMR1',
    11: 'A-1-1', 12: 'A-1-2', 13: 'A-2-1', 14: 'A-2-2',
    15: 'A-3-1', 16: 'A-3-2',
    19: '선박-2', 20: '선박-3', 21: '선박-4',
    22: '선박-5', 23: '선박-6',
}


def is_terminal_event(event):
    """
    Return true only for an operation-level terminal phase.

    ARM2 uses ``state=COMPLETED`` for successful intermediate phases such as
    SOURCE_LOCKED. Therefore state must never terminate the central command.
    """
    return str(event.get('phase', '')).upper() in TERMINAL_PHASES


def is_terminal_arm1_state(state):
    """Return whether ARM1's fixed work-state contract has terminated."""
    return str(state or '').strip().upper() in ARM1_TERMINAL_STATES


def movement_from_goal(goal, success, operation_id, vehicle_cargo=None):
    """Normalize one physical transfer into the inventory event contract."""
    operation = str(goal.operation or '').lower()
    arm_id = str(goal.arm_id or 'arm2').lower()
    vehicle_id = str(goal.vehicle_id or '').lower()
    trailer_location = {'agv1': 'AMR1', 'agv2': 'AMR2'}.get(vehicle_id, '')
    carried = str((vehicle_cargo or {}).get(vehicle_id, '') or '')
    source_location = ''
    destination_location = ''
    container_id = ''

    if arm_id == 'arm1' and operation == 'pick_place':
        source_id = int(goal.source_id)
        destination_id = int(goal.destination_id)
        if 0 <= source_id <= 8:
            container_id = str(source_id)
        else:
            container_id = carried
            source_location = MARKER_LOCATIONS.get(source_id, '')
        destination_location = MARKER_LOCATIONS.get(destination_id, '')
        if (
            trailer_location
            and destination_location.startswith('선박-')
        ):
            # Trailer-to-ship commands pick the exposed container marker
            # (0..8), not the trailer marker hidden below the cargo.
            source_location = trailer_location
    elif arm_id == 'arm2' and operation == 'load_to_trailer':
        container_id = str(int(goal.source_id))
        destination_location = trailer_location
    elif arm_id == 'arm2' and operation == 'transfer_to_slot':
        container_id = carried
        source_location = trailer_location
        destination_location = str(goal.destination_slot or '').upper()
    elif arm_id == 'arm2' and operation == 'transfer_by_id':
        container_id = str(int(goal.source_id))
        destination_location = (
            str(getattr(goal, 'destination_slot', '') or '').upper()
            or MARKER_LOCATIONS.get(int(goal.destination_id), '')
        )
    else:
        return None
    explicit_container_id = str(
        getattr(goal, 'container_id', '') or ''
    ).strip()
    if explicit_container_id:
        container_id = explicit_container_id
    if not container_id or not destination_location:
        return None
    return {
        'schema_version': '1.0',
        'operation_id': str(operation_id or f'arm-{goal.command_id}'),
        'command_id': str(goal.command_id),
        'mission_id': str(goal.mission_id),
        'arm_id': arm_id,
        'container_id': container_id,
        'source_location': source_location,
        'source_floor': None,
        'source_base_aruco_id': '',
        'destination_location': destination_location,
        'destination_floor': (
            int(getattr(goal, 'destination_floor', 0) or 0) or None
            if arm_id == 'arm2' and operation in {
                'transfer_to_slot', 'transfer_by_id'
            } else None
        ),
        'destination_base_aruco_id': (
            str(int(goal.destination_id))
            if arm_id == 'arm2'
            and operation == 'transfer_by_id'
            and int(getattr(goal, 'destination_floor', 0) or 0) > 1
            and 0 <= int(goal.destination_id) <= 8
            else ''
        ),
        'success': bool(success),
        'completed_at': datetime.now(timezone.utc).isoformat(),
        'error': '',
    }


class ArmDispatcher(Node):
    """Expose one action endpoint with an independent queue for each ARM."""

    ERROR_INVALID_REQUEST = 1
    ERROR_UNCONFIGURED = 2
    ERROR_SERVICE_UNAVAILABLE = 3
    ERROR_REJECTED = 4
    ERROR_OPERATION_FAILED = 5
    ERROR_TIMEOUT = 6
    ERROR_CANCELED = 7

    def __init__(self):
        super().__init__('arm_dispatcher')
        self.declare_parameter(
            'dispatch_action', '/central/arms/dispatch'
        )
        self.declare_parameter('arm2_event_topic', '/arm2/transfer_events')
        self.declare_parameter(
            'arm1_work_state_topic', '/arm/pick_place/work_state'
        )
        self.declare_parameter(
            'arm1_status_topic', '/arm/pick_place/status'
        )
        self.declare_parameter('state_publish_rate_hz', 2.0)
        self.declare_parameter('service_wait_timeout_sec', 5.0)
        self.declare_parameter('service_retry_count', 3)
        self.declare_parameter('operation_timeout_sec', 600.0)
        self.declare_parameter('scan_timeout_sec', 240.0)
        # A transfer only makes sense once the trailer is actually parked in
        # front of the arm. Without this the arm reached for a vehicle that
        # was still driving. Empty zone disables the wait.
        self.declare_parameter('arm1_vehicle_arrival_zone', 'B-1')
        self.declare_parameter('arm2_vehicle_arrival_zone', 'A')
        self.declare_parameter('vehicle_arrival_timeout_sec', 300.0)
        self.declare_parameter('vehicle_state_max_age_sec', 5.0)

        self.callback_group = ReentrantCallbackGroup()
        self.condition = threading.Condition()
        self.queues = {'arm1': deque(), 'arm2': deque()}
        # Legacy aggregate attributes remain for lightweight diagnostics and
        # old test doubles. Scheduling uses the per-arm dictionaries below.
        self.queue = deque()
        # token -> vehicle_id for every queued or running command that has
        # promised to gate a vehicle's departure. Derived from live queue
        # state rather than edge events so a dropped message self-heals.
        self.vehicle_commitments = {}
        self.vehicle_arrival_zones = {
            arm_id: str(
                self.get_parameter(
                    f'{arm_id}_vehicle_arrival_zone'
                ).value or ''
            ).strip('/')
            for arm_id in ('arm1', 'arm2')
        }
        self.vehicle_arrival_timeout_sec = float(
            self.get_parameter('vehicle_arrival_timeout_sec').value
        )
        if self.vehicle_arrival_timeout_sec <= 0.0:
            raise ValueError('vehicle_arrival_timeout_sec must be positive')
        self.vehicle_state_max_age_sec = float(
            self.get_parameter('vehicle_state_max_age_sec').value
        )
        if self.vehicle_state_max_age_sec <= 0.0:
            raise ValueError('vehicle_state_max_age_sec must be positive')
        self.vehicle_states = {}
        self.events = deque(maxlen=500)
        self.latest_sequence = 0
        self.latest_event_at = None
        self.active_goal = None
        self.active_command = None
        self.active_goals = {'arm1': None, 'arm2': None}
        self.active_commands = {'arm1': None, 'arm2': None}
        self.stop_generations = {'arm1': 0, 'arm2': 0}
        self.failed_missions = set()
        # Cargo currently known to be on each trailer. This bridges the two
        # independent ARM operations in one physical movement without asking
        # either robot controller to write the database directly.
        self.vehicle_cargo = {}
        self.arm2_last_error = ''
        self.arm2_state = ArmState.OFFLINE
        self.arm2_state_text = 'waiting for ARM2 event or service'
        self.arm1_last_error = ''
        self.arm1_status_text = ''
        self.arm1_state = ArmState.OFFLINE
        self.arm1_state_text = 'waiting for ARM1 work_state or service'
        self.arm1_latest_state_at = None
        self.arm1_event_sequence = 0
        self.arm1_events = deque(maxlen=100)

        event_qos = QoSProfile(
            depth=50,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('arm2_event_topic').value),
            self._on_arm2_event,
            event_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('arm1_work_state_topic').value),
            self._on_arm1_work_state,
            event_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            str(self.get_parameter('arm1_status_topic').value),
            self._on_arm1_status,
            10,
            callback_group=self.callback_group,
        )
        self.result_publisher = self.create_publisher(
            String, '/central/arms/results', event_qos
        )
        self.movement_publisher = self.create_publisher(
            String, '/central/inventory/movements', event_qos
        )
        # Volatile on purpose: a periodic snapshot must never be replayed to
        # a late joiner, or a restarted fleet dispatcher would hold a vehicle
        # against a command that finished long ago.
        self.vehicle_hold_publisher = self.create_publisher(
            String, '/central/arms/vehicle_holds', 10
        )
        for vehicle_id in ('agv1', 'agv2'):
            self.create_subscription(
                VehicleState,
                f'/central/fleet/{vehicle_id}/state',
                lambda message, vid=vehicle_id: self._on_vehicle_state(
                    vid, message
                ),
                10,
                callback_group=self.callback_group,
            )
        self.arm1_state_publisher = self.create_publisher(
            ArmState, '/central/arms/arm1/state', 10
        )
        self.arm2_state_publisher = self.create_publisher(
            ArmState, '/central/arms/arm2/state', 10
        )

        self.trigger_clients = {}
        for name in (
            'scan_destinations',
            'stop_pick',
            'reset_stack_level',
            'go_initial_pose',
            'go_a1_pose',
            'go_a2_pose',
            'go_a3_pose',
        ):
            self.trigger_clients[name] = self.create_client(
                Trigger,
                f'/arm2/{name}',
                callback_group=self.callback_group,
            )
        for slot in ('a1_1', 'a1_2', 'a2_1', 'a2_2', 'a3_1', 'a3_2'):
            self.trigger_clients[f'transfer_{slot}'] = self.create_client(
                Trigger,
                f'/arm2/transfer_to_{slot}',
                callback_group=self.callback_group,
            )
        for source_id in range(9):
            key = f'load_id{source_id}_to_trailer'
            self.trigger_clients[key] = self.create_client(
                Trigger,
                f'/arm2/{key}',
                callback_group=self.callback_group,
            )
        self.transfer_by_id_client = self.create_client(
            TransferById,
            '/arm2/transfer_by_id',
            callback_group=self.callback_group,
        )
        self.transfer_to_slot_client = self.create_client(
            TransferToSlot,
            '/arm2/transfer_to_slot',
            callback_group=self.callback_group,
        )
        self.arm1_execute_client = self.create_client(
            ExecutePickPlace,
            '/arm/pick_place/execute',
            callback_group=self.callback_group,
        )
        self.arm1_stop_client = self.create_client(
            Trigger,
            '/arm/pick_place/stop',
            callback_group=self.callback_group,
        )
        self.arm1_ship_scan_client = self.create_client(
            Trigger,
            '/arm/pick_place/scan_ship_destinations',
            callback_group=self.callback_group,
        )
        self.arm1_inbound_scan_client = self.create_client(
            Trigger,
            '/arm/pick_place/scan_inbound',
            callback_group=self.callback_group,
        )

        self.create_service(
            Trigger,
            '/central/arms/arm1/stop',
            self._stop_arm1,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/central/arms/arm2/stop',
            self._stop_arm2,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/central/arms/arm1/resume',
            self._resume_arm1,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/central/arms/arm2/resume',
            self._resume_arm2,
            callback_group=self.callback_group,
        )
        self.action_server = ActionServer(
            self,
            DispatchArmCommand,
            str(self.get_parameter('dispatch_action').value),
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self.callback_group,
        )
        rate = max(
            0.2, float(self.get_parameter('state_publish_rate_hz').value)
        )
        self.create_timer(1.0 / rate, self._publish_states)
        self.get_logger().info(
            'ARM dispatcher ready: ARM1=/arm/pick_place '
            f'(arrival={self.vehicle_arrival_zones["arm1"]}), '
            'ARM2=/arm2 services '
            f'(arrival={self.vehicle_arrival_zones["arm2"]})'
        )

    def _accept_goal(self, goal_request):
        if not str(goal_request.command_id).strip():
            self.get_logger().warning('Rejected ARM command without command_id')
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_goal(self, _goal_handle):
        return CancelResponse.ACCEPT

    def _on_arm2_event(self, message):
        try:
            event = json.loads(message.data)
            event['_received_monotonic'] = time.monotonic()
            sequence = int(event.get('sequence', 0))
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Invalid ARM2 event ignored: {exc}')
            return
        with self.condition:
            self.latest_sequence = max(self.latest_sequence, sequence)
            self.latest_event_at = time.monotonic()
            self.events.append(event)
            phase = str(event.get('phase', '')).upper()
            state = str(event.get('state', '')).upper()
            if is_terminal_event(event):
                if phase == 'COMPLETED' and state == 'COMPLETED':
                    self.arm2_state = ArmState.READY
                elif phase == 'STOPPED':
                    self.arm2_state = ArmState.STOPPED
                else:
                    self.arm2_state = ArmState.ERROR
                self.arm2_state_text = phase or state
                self.arm2_last_error = str(event.get('error') or '')
            else:
                self.arm2_state = ArmState.BUSY
                self.arm2_state_text = phase or state or 'RUNNING'
            self.condition.notify_all()

    def _on_arm1_work_state(self, message):
        state = str(message.data or '').strip().upper()
        if state not in ARM1_PROGRESS:
            self.get_logger().warning(
                f'Invalid ARM1 work_state ignored: {state!r}'
            )
            return
        with self.condition:
            self.arm1_event_sequence += 1
            self.arm1_latest_state_at = time.monotonic()
            self.arm1_events.append((self.arm1_event_sequence, state))
            self.arm1_state_text = state
            if state in {'IDLE', 'WORK_COMPLETED'}:
                self.arm1_state = ArmState.READY
                self.arm1_last_error = ''
            elif state in {'STOP_REQUESTED', 'STOPPED'}:
                self.arm1_state = ArmState.STOPPED
            elif state == 'FAILED':
                self.arm1_state = ArmState.ERROR
                self.arm1_last_error = (
                    getattr(self, 'arm1_status_text', '')
                    or 'ARM1 pick/place reported FAILED'
                )
            else:
                self.arm1_state = ArmState.BUSY
            self.condition.notify_all()

    def _on_arm1_status(self, message):
        """Keep ARM1's human-readable failure detail beside work_state."""
        status = str(message.data or '').strip()
        if not status:
            return
        with self.condition:
            self.arm1_status_text = status
            if self.arm1_state == ArmState.ERROR:
                self.arm1_last_error = status
            self.condition.notify_all()

    @staticmethod
    def _service_is_ready(client):
        """Read one ROS service graph state without making snapshots fragile."""
        try:
            return client is not None and bool(client.service_is_ready())
        except Exception:
            return False

    def _arm2_service_is_ready(self):
        """Return whether at least one persistent ARM2 endpoint is visible."""
        clients = [
            self.trigger_clients.get('stop_pick'),
            self.trigger_clients.get('scan_destinations'),
            self.transfer_by_id_client,
        ]
        return any(self._service_is_ready(client) for client in clients)

    def _active_command_for(self, arm_id):
        active = getattr(self, 'active_commands', None)
        if isinstance(active, dict):
            return active.get(str(arm_id).lower())
        legacy = getattr(self, 'active_command', None)
        if (
            legacy is not None
            and str(legacy.arm_id or 'arm2').lower() == str(arm_id).lower()
        ):
            return legacy
        return None

    def _refresh_arm2_idle_connectivity(self):
        """Derive idle ARM2 connectivity from services before any event exists."""
        if self._active_command_for('arm2') is not None:
            return
        service_ready = self._arm2_service_is_ready()
        if service_ready and self.arm2_state == ArmState.OFFLINE:
            self.arm2_state = ArmState.READY
            self.arm2_state_text = 'SERVICE_CONNECTED'
            self.arm2_last_error = ''
        elif (
            not service_ready
            and self.arm2_state == ArmState.READY
            and self.arm2_state_text == 'SERVICE_CONNECTED'
        ):
            self.arm2_state = ArmState.OFFLINE
            self.arm2_state_text = 'waiting for ARM2 event or service'

    def _publish_states(self):
        self._refresh_arm2_idle_connectivity()
        self._publish_vehicle_holds()
        now = self.get_clock().now().to_msg()
        arm1 = ArmState()
        arm1.header.stamp = now
        arm1.header.frame_id = 'arm1'
        arm1.arm_id = 'arm1'
        arm1.state = self.arm1_state
        arm1.state_text = self.arm1_state_text
        arm1.ready = self.arm1_state == ArmState.READY
        arm1.last_error = self.arm1_last_error
        arm1.telemetry_age_sec = (
            float('inf')
            if self.arm1_latest_state_at is None
            else float(time.monotonic() - self.arm1_latest_state_at)
        )
        command = self._active_command_for('arm1')
        if command is not None:
            arm1.current_command_id = str(command.command_id)
            arm1.current_mission_id = str(command.mission_id)
            arm1.current_operation = str(command.operation)
            arm1.operation_id = f'arm1-{command.command_id}'
            arm1.phase = self.arm1_state_text
            arm1.progress = float(
                ARM1_PROGRESS.get(self.arm1_state_text, 0.0)
            )
        self.arm1_state_publisher.publish(arm1)

        arm2 = ArmState()
        arm2.header.stamp = now
        arm2.header.frame_id = 'arm2'
        arm2.arm_id = 'arm2'
        arm2.state = self.arm2_state
        arm2.state_text = self.arm2_state_text
        arm2.ready = self.arm2_state == ArmState.READY
        arm2.last_error = self.arm2_last_error
        if (
            self.latest_event_at is None
            and self.arm2_state_text == 'SERVICE_CONNECTED'
        ):
            arm2.telemetry_age_sec = 0.0
        elif self.latest_event_at is None:
            arm2.telemetry_age_sec = float('inf')
        else:
            arm2.telemetry_age_sec = float(
                time.monotonic() - self.latest_event_at
            )
        command = self._active_command_for('arm2')
        if command is not None:
            arm2.current_command_id = str(command.command_id)
            arm2.current_mission_id = str(command.mission_id)
            arm2.current_operation = str(command.operation)
            if self.events:
                arm2.operation_id = str(
                    self.events[-1].get('operation_id', '')
                )
                arm2.phase = str(self.events[-1].get('phase', ''))
                progress = self.events[-1].get('progress')
                arm2.progress = float(progress or 0.0)
        self.arm2_state_publisher.publish(arm2)

    def _validate(self, goal):
        arm_id = str(goal.arm_id or 'arm2').lower()
        operation = str(goal.operation).lower()
        if arm_id not in {'arm1', 'arm2'}:
            return None, f'unknown arm_id: {arm_id}'
        if arm_id == 'arm1':
            if operation not in {
                'pick_place', 'scan_ship_destinations', 'scan_inbound', 'stop'
            }:
                return None, f'unsupported ARM1 operation: {operation}'
            if operation == 'pick_place' and (
                not 0 <= goal.source_id <= 49
                or not 0 <= goal.destination_id <= 49
                or goal.source_id == goal.destination_id
            ):
                return None, (
                    'ARM1 pick_place requires different source_id and '
                    'destination_id values within 0..49'
                )
            return operation, ''
        if operation not in {
            'scan_destinations', 'transfer_to_slot', 'load_to_trailer',
            'transfer_by_id', 'go_pose', 'reset_stack_level', 'stop',
        }:
            return None, f'unsupported ARM2 operation: {operation}'
        if operation == 'transfer_to_slot':
            slot = str(goal.destination_slot).upper()
            if slot not in {
                'A-1-1', 'A-1-2', 'A-2-1', 'A-2-2', 'A-3-1', 'A-3-2'
            }:
                return None, f'invalid destination_slot: {slot}'
            destination_floor = int(
                getattr(goal, 'destination_floor', 0) or 0
            )
            if destination_floor not in {0, 1, 2, 3}:
                return None, (
                    'destination_floor must be 1..3 '
                    '(or 0 for legacy automatic stacking)'
                )
        if operation == 'load_to_trailer' and not 0 <= goal.source_id <= 8:
            return None, 'source_id must be 0..8'
        if operation == 'transfer_by_id' and (
            not 0 <= goal.source_id <= 8
            or goal.destination_id not in set(range(9)) | set(range(11, 17))
            or goal.source_id == goal.destination_id
        ):
            return None, 'invalid source_id/destination_id pair'
        if operation == 'go_pose' and str(goal.destination_slot).lower() not in {
            'initial', 'a-1', 'a-2', 'a-3'
        }:
            return None, 'go_pose destination_slot must be initial/A-1/A-2/A-3'
        return operation, ''

    def _service_for_goal(self, goal, operation):
        if str(goal.arm_id).lower() == 'arm1':
            if operation == 'stop':
                return self.arm1_stop_client, Trigger.Request(), 'success'
            if operation == 'scan_ship_destinations':
                return (
                    self.arm1_ship_scan_client, Trigger.Request(), 'success'
                )
            if operation == 'scan_inbound':
                return self.arm1_inbound_scan_client, Trigger.Request(), 'success'
            request = ExecutePickPlace.Request()
            request.pick_id = int(goal.source_id)
            request.place_id = int(goal.destination_id)
            return self.arm1_execute_client, request, 'accepted'
        if operation == 'transfer_by_id':
            request = TransferById.Request()
            request.source_id = int(goal.source_id)
            request.destination_id = int(goal.destination_id)
            return self.transfer_by_id_client, request, 'accepted'
        if operation == 'transfer_to_slot':
            request = TransferToSlot.Request()
            request.destination_slot = str(goal.destination_slot).upper()
            request.destination_floor = int(
                getattr(goal, 'destination_floor', 0) or 0
            )
            return self.transfer_to_slot_client, request, 'success'
        elif operation == 'load_to_trailer':
            key = f'load_id{int(goal.source_id)}_to_trailer'
        elif operation == 'go_pose':
            suffix = str(goal.destination_slot).lower().replace('-', '')
            key = 'go_initial_pose' if suffix == 'initial' else f'go_{suffix}_pose'
        elif operation == 'stop':
            key = 'stop_pick'
        else:
            key = operation
        return self.trigger_clients[key], Trigger.Request(), 'success'

    def _call_service(self, client, request, accepted_field):
        wait_timeout = float(
            self.get_parameter('service_wait_timeout_sec').value
        )
        retries = max(1, int(self.get_parameter('service_retry_count').value))
        last_error = 'service unavailable'
        for _attempt in range(retries):
            if not client.wait_for_service(timeout_sec=wait_timeout):
                last_error = f'{client.srv_name} is unavailable'
                continue
            done = threading.Event()
            future = client.call_async(request)
            future.add_done_callback(lambda _future: done.set())
            if not done.wait(wait_timeout):
                last_error = f'{client.srv_name} call timed out'
                continue
            try:
                response = future.result()
            except Exception as exc:  # ROS transport exception
                last_error = str(exc)
                continue
            accepted = bool(getattr(response, accepted_field, False))
            return accepted, str(getattr(response, 'message', ''))
        return False, last_error

    def _wait_for_terminal_event(
        self,
        goal_handle,
        operation,
        baseline_sequence,
        expected_operation,
        command_generation,
    ):
        timeout = float(
            self.get_parameter(
                'scan_timeout_sec'
                if operation == 'scan_destinations'
                else 'operation_timeout_sec'
            ).value
        )
        deadline = time.monotonic() + timeout
        operation_id = ''
        consumed_sequence = baseline_sequence
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return None, operation_id, 'command canceled'
            with self.condition:
                if command_generation != self.stop_generations['arm2']:
                    return None, operation_id, 'command stopped by operator'
                candidates = [
                    event for event in self.events
                    if int(event.get('sequence', 0)) > consumed_sequence
                ]
                for event in candidates:
                    consumed_sequence = max(
                        consumed_sequence, int(event.get('sequence', 0))
                    )
                    if str(event.get('operation', '')) != expected_operation:
                        continue
                    candidate_id = str(event.get('operation_id', ''))
                    if not operation_id:
                        operation_id = candidate_id
                    if candidate_id != operation_id:
                        continue
                    feedback = DispatchArmCommand.Feedback()
                    feedback.arm_id = 'arm2'
                    feedback.phase = str(event.get('phase', ''))
                    feedback.progress = float(event.get('progress') or 0.0)
                    feedback.message = str(event.get('message') or '')
                    goal_handle.publish_feedback(feedback)
                    if is_terminal_event(event):
                        return event, operation_id, ''
                self.condition.wait(timeout=min(0.25, deadline - time.monotonic()))
        return None, operation_id, f'{operation} timed out after {timeout:.1f}s'

    def _wait_for_arm1_terminal(
        self, goal_handle, baseline_sequence, command_generation
    ):
        """Wait for a new terminal work_state from ARM1's coordinator."""
        timeout = float(self.get_parameter('operation_timeout_sec').value)
        deadline = time.monotonic() + timeout
        consumed_sequence = baseline_sequence
        work_started = False
        operation_id = f'arm1-{goal_handle.request.command_id}'
        while time.monotonic() < deadline:
            if goal_handle.is_cancel_requested:
                return None, operation_id, 'command canceled'
            with self.condition:
                if command_generation != self.stop_generations['arm1']:
                    return None, operation_id, 'command stopped by operator'
                candidates = [
                    event for event in self.arm1_events
                    if event[0] > consumed_sequence
                ]
                for sequence, state in candidates:
                    consumed_sequence = max(consumed_sequence, sequence)
                    if state == 'WORK_STARTED':
                        work_started = True
                    feedback = DispatchArmCommand.Feedback()
                    feedback.arm_id = 'arm1'
                    feedback.phase = state
                    feedback.progress = float(ARM1_PROGRESS.get(state, 0.0))
                    feedback.message = f'ARM1 {state}'
                    goal_handle.publish_feedback(feedback)
                    # Require a fresh WORK_STARTED before accepting a terminal
                    # state, so a transient-local replay from an old run can
                    # never complete a new central command.
                    if work_started and is_terminal_arm1_state(state):
                        if state == 'FAILED':
                            # arm_v2 publishes the detailed status immediately
                            # after FAILED. Briefly release the condition so the
                            # status callback can replace the generic error.
                            self.condition.wait(timeout=0.25)
                        return state, operation_id, ''
                self.condition.wait(
                    timeout=min(0.25, max(0.0, deadline - time.monotonic()))
                )
        return None, operation_id, f'pick_place timed out after {timeout:.1f}s'

    def _set_runtime_state(self, arm_id, state, state_text, error=None):
        """Update the selected arm without corrupting the other arm's card."""
        if str(arm_id).lower() == 'arm1':
            self.arm1_state = state
            self.arm1_state_text = state_text
            if error is not None:
                self.arm1_last_error = error
        else:
            self.arm2_state = state
            self.arm2_state_text = state_text
            if error is not None:
                self.arm2_last_error = error

    def _on_vehicle_state(self, vehicle_id, message):
        with self.condition:
            self.vehicle_states[vehicle_id] = (message, time.monotonic())
            self.condition.notify_all()

    def _arrival_zone_for_arm(self, arm_id):
        """Return the physical vehicle work zone for one robot arm."""
        zones = getattr(self, 'vehicle_arrival_zones', None)
        if isinstance(zones, dict):
            return str(zones.get(str(arm_id).lower(), '')).strip('/')
        # Compatibility for lightweight test doubles and old serialized state.
        return str(getattr(self, 'vehicle_arrival_zone', '')).strip('/')

    def _vehicle_has_arrived(self, vehicle_id, arm_id='arm2'):
        """Return true once the vehicle is parked in the arm's work zone."""
        arrival_zone = self._arrival_zone_for_arm(arm_id)
        with self.condition:
            entry = self.vehicle_states.get(vehicle_id)
            if entry is None:
                return False
            message, received_at = entry
        if time.monotonic() - received_at > self.vehicle_state_max_age_sec:
            # Stale telemetry says nothing about where the vehicle is now.
            return False
        if message.state != VehicleState.READY:
            return False
        return (
            str(message.locked_zone).strip('/') == arrival_zone
        )

    def _wait_for_vehicle_arrival(
        self, goal_handle, goal, generation, arm_id
    ):
        """Hold the command until its vehicle is parked in the work zone."""
        vehicle_id = str(goal.vehicle_id)
        arrival_zone = self._arrival_zone_for_arm(arm_id)
        if not arrival_zone or not vehicle_id:
            return True, ''
        deadline = time.monotonic() + self.vehicle_arrival_timeout_sec
        announced = False
        while not self._vehicle_has_arrived(vehicle_id, arm_id):
            if goal_handle.is_cancel_requested:
                return False, f'canceled while waiting for {vehicle_id}'
            with self.condition:
                stopped = generation != self.stop_generations[arm_id]
            if stopped:
                return False, 'stopped by operator while waiting'
            if time.monotonic() >= deadline:
                return False, (
                    f'{vehicle_id} did not reach '
                    f'{arrival_zone} within '
                    f'{self.vehicle_arrival_timeout_sec:.0f}s'
                )
            if not announced:
                announced = True
                self.get_logger().info(
                    f'Waiting for {vehicle_id} to reach '
                    f'{arrival_zone} before starting '
                    f'{goal.operation}'
                )
            with self.condition:
                self.condition.wait(timeout=0.2)
        return True, ''

    @staticmethod
    def _gates_vehicle(goal, operation):
        """
        Return true when this command must gate the vehicle's departure.

        The same predicate decides both the hold taken while the command is
        pending and the release reported when it finishes, so the two can
        never disagree.
        """
        return bool(
            goal.final_for_vehicle
            and str(goal.vehicle_id)
            and operation in {
                'transfer_to_slot', 'load_to_trailer', 'pick_place'
            }
        )

    def _publish_vehicle_holds(self):
        """Broadcast which vehicles are still owed a cargo operation."""
        with self.condition:
            held = sorted(set(self.vehicle_commitments.values()))
        message = String()
        message.data = json.dumps(
            {'held_vehicles': held}, ensure_ascii=False
        )
        self.vehicle_hold_publisher.publish(message)

    def _publish_result(self, goal, success, operation_id, message, release):
        payload = {
            'command_id': str(goal.command_id),
            'mission_id': str(goal.mission_id),
            'arm_id': str(goal.arm_id or 'arm2'),
            'vehicle_id': str(goal.vehicle_id),
            'operation': str(goal.operation),
            'operation_id': operation_id,
            'success': bool(success),
            'final_for_vehicle': bool(goal.final_for_vehicle),
            'vehicle_release_allowed': bool(release),
            'message': message,
            'source_id': int(goal.source_id),
            'destination_id': int(goal.destination_id),
            'destination_slot': str(goal.destination_slot),
            'destination_floor': int(
                getattr(goal, 'destination_floor', 0) or 0
            ),
            'container_id': str(getattr(goal, 'container_id', '') or ''),
        }
        message_out = String()
        message_out.data = json.dumps(payload, ensure_ascii=False)
        self.result_publisher.publish(message_out)
        movement = movement_from_goal(
            goal, success, operation_id, self.vehicle_cargo
        )
        if movement is not None:
            if not success:
                movement['error'] = str(message or 'ARM operation failed')
            movement_out = String()
            movement_out.data = json.dumps(movement, ensure_ascii=False)
            self.movement_publisher.publish(movement_out)
            if success:
                destination = movement['destination_location']
                source = movement['source_location']
                vehicle_id = str(goal.vehicle_id or '').lower()
                if destination in {'AMR1', 'AMR2'} and vehicle_id:
                    self.vehicle_cargo[vehicle_id] = movement['container_id']
                elif source in {'AMR1', 'AMR2'} and vehicle_id:
                    self.vehicle_cargo.pop(vehicle_id, None)

    def _finish_error(self, goal_handle, goal, code, message, operation_id=''):
        result = DispatchArmCommand.Result()
        result.success = False
        result.error_code = code
        result.message = message
        result.operation_id = operation_id
        result.vehicle_release_allowed = False
        if goal_handle.is_active:
            goal_handle.abort()
        if str(goal.mission_id):
            self.failed_missions.add(str(goal.mission_id))
        self._set_runtime_state(
            goal.arm_id, ArmState.ERROR, 'FAILED', str(message)
        )
        self._publish_result(goal, False, operation_id, message, False)
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        arm_id = str(goal.arm_id or 'arm2').lower()
        operation, error = self._validate(goal)
        if operation is None:
            return self._finish_error(
                goal_handle, goal, self.ERROR_INVALID_REQUEST, error
            )
        if str(goal.mission_id) in self.failed_missions:
            return self._finish_error(
                goal_handle,
                goal,
                self.ERROR_OPERATION_FAILED,
                f'mission {goal.mission_id} already failed; command skipped',
            )

        token = object()
        with self.condition:
            command_generation = self.stop_generations[arm_id]
            queues = getattr(self, 'queues', None)
            arm_queue = (
                queues.setdefault(arm_id, deque())
                if isinstance(queues, dict)
                else self.queue
            )
            arm_queue.append(token)
            while arm_queue and arm_queue[0] is not token:
                if (
                    goal_handle.is_cancel_requested
                    or command_generation != self.stop_generations[arm_id]
                ):
                    arm_queue.remove(token)
                    message = (
                        'command canceled while queued'
                        if goal_handle.is_cancel_requested
                        else 'command stopped by operator while queued'
                    )
                    goal_handle.canceled()
                    result = DispatchArmCommand.Result()
                    result.success = False
                    result.error_code = self.ERROR_CANCELED
                    result.message = message
                    self._publish_result(goal, False, '', message, False)
                    if str(goal.mission_id):
                        self.failed_missions.add(str(goal.mission_id))
                    self.condition.notify_all()
                    return result
                self.condition.wait(timeout=0.2)
            active_goals = getattr(self, 'active_goals', None)
            active_commands = getattr(self, 'active_commands', None)
            if isinstance(active_goals, dict):
                active_goals[arm_id] = goal_handle
            if isinstance(active_commands, dict):
                active_commands[arm_id] = goal
            self.active_goal = goal_handle
            self.active_command = goal
            self._set_runtime_state(arm_id, ArmState.BUSY, 'DISPATCHING')
            baseline_sequence = (
                self.arm1_event_sequence
                if arm_id == 'arm1'
                else self.latest_sequence
            )

        try:
            if str(goal.mission_id) in self.failed_missions:
                return self._finish_error(
                    goal_handle,
                    goal,
                    self.ERROR_OPERATION_FAILED,
                    f'mission {goal.mission_id} failed while command was queued',
                )
            if self._gates_vehicle(goal, operation):
                # Order matters: wait for the vehicle first, claim it second.
                # Claiming before the wait would hold the vehicle in place
                # while the arm waited for that same vehicle to arrive.
                with self.condition:
                    self._set_runtime_state(
                        arm_id, ArmState.BUSY, 'WAITING_FOR_VEHICLE'
                    )
                arrived, arrival_message = self._wait_for_vehicle_arrival(
                    goal_handle, goal, command_generation, arm_id
                )
                if not arrived:
                    return self._finish_error(
                        goal_handle,
                        goal,
                        (
                            self.ERROR_CANCELED
                            if goal_handle.is_cancel_requested
                            else self.ERROR_TIMEOUT
                        ),
                        arrival_message,
                    )
                with self.condition:
                    self.vehicle_commitments[id(token)] = str(goal.vehicle_id)
                    self._set_runtime_state(
                        arm_id, ArmState.BUSY, 'DISPATCHING'
                    )
            client, request, accepted_field = self._service_for_goal(
                goal, operation
            )
            if arm_id == 'arm1' and operation in {
                'pick_place', 'scan_ship_destinations', 'scan_inbound'
            }:
                # A latched-looking status string from an earlier operation
                # must never become the failure reason for this command.
                with self.condition:
                    self.arm1_status_text = ''
                    self.arm1_last_error = ''
            accepted, service_message = self._call_service(
                client, request, accepted_field
            )
            with self.condition:
                stopped = command_generation != self.stop_generations[arm_id]
            if stopped:
                return self._finish_error(
                    goal_handle,
                    goal,
                    self.ERROR_CANCELED,
                    'command stopped by operator',
                )
            if not accepted:
                self._set_runtime_state(
                    arm_id,
                    ArmState.ERROR,
                    'REJECTED',
                    service_message,
                )
                return self._finish_error(
                    goal_handle,
                    goal,
                    self.ERROR_REJECTED,
                    service_message or f'{arm_id.upper()} rejected the command',
                )

            operation_id = ''
            if arm_id == 'arm1' and operation in {
                'pick_place', 'scan_ship_destinations', 'scan_inbound'
            }:
                terminal_state, operation_id, wait_error = (
                    self._wait_for_arm1_terminal(
                        goal_handle,
                        baseline_sequence,
                        command_generation,
                    )
                )
                if terminal_state is None:
                    with self.condition:
                        stopped = (
                            command_generation
                            != self.stop_generations[arm_id]
                        )
                    if goal_handle.is_cancel_requested and not stopped:
                        self._call_service(
                            self.arm1_stop_client,
                            Trigger.Request(),
                            'success',
                        )
                    return self._finish_error(
                        goal_handle,
                        goal,
                        (
                            self.ERROR_CANCELED
                            if goal_handle.is_cancel_requested or stopped
                            else self.ERROR_TIMEOUT
                        ),
                        wait_error,
                        operation_id,
                    )
                success = terminal_state == 'WORK_COMPLETED'
                terminal_message = (
                    self.arm1_last_error
                    if terminal_state == 'FAILED' and self.arm1_last_error
                    else f'ARM1 {terminal_state}'
                )
                if not success:
                    return self._finish_error(
                        goal_handle,
                        goal,
                        self.ERROR_OPERATION_FAILED,
                        terminal_message,
                        operation_id,
                    )
            elif operation in EVENT_OPERATIONS:
                event, operation_id, wait_error = self._wait_for_terminal_event(
                    goal_handle,
                    operation,
                    baseline_sequence,
                    EVENT_OPERATIONS[operation],
                    command_generation,
                )
                if event is None:
                    with self.condition:
                        stopped = (
                            command_generation
                            != self.stop_generations[arm_id]
                        )
                    code = (
                        self.ERROR_CANCELED
                        if goal_handle.is_cancel_requested or stopped
                        else self.ERROR_TIMEOUT
                    )
                    if goal_handle.is_cancel_requested or stopped:
                        if goal_handle.is_cancel_requested and not stopped:
                            self._call_service(
                                self.trigger_clients['stop_pick'],
                                Trigger.Request(),
                                'success',
                            )
                        goal_handle.canceled()
                        result = DispatchArmCommand.Result()
                        result.success = False
                        result.error_code = code
                        result.message = wait_error
                        result.operation_id = operation_id
                        self._publish_result(
                            goal, False, operation_id, wait_error, False
                        )
                        if str(goal.mission_id):
                            self.failed_missions.add(str(goal.mission_id))
                        return result
                    return self._finish_error(
                        goal_handle, goal, code, wait_error, operation_id
                    )
                success = (
                    str(event.get('phase', '')).upper() == 'COMPLETED'
                    and str(event.get('state', '')).upper() == 'COMPLETED'
                )
                terminal_message = str(
                    event.get('message') or event.get('error') or ''
                )
                if not success:
                    self._set_runtime_state(
                        arm_id,
                        ArmState.WAITING_OPERATOR,
                        'WAITING_OPERATOR',
                        str(event.get('error') or terminal_message),
                    )
                    return self._finish_error(
                        goal_handle,
                        goal,
                        self.ERROR_OPERATION_FAILED,
                        terminal_message or f'{arm_id.upper()} operation failed',
                        operation_id,
                    )
            else:
                success = True
                terminal_message = service_message

            release = self._gates_vehicle(goal, operation)
            result = DispatchArmCommand.Result()
            result.success = True
            result.error_code = 0
            result.message = (
                terminal_message or f'{arm_id.upper()} operation completed'
            )
            result.operation_id = operation_id
            result.vehicle_release_allowed = release
            self._set_runtime_state(arm_id, ArmState.READY, 'READY', '')
            goal_handle.succeed()
            self._publish_result(
                goal, True, operation_id, result.message, release
            )
            return result
        finally:
            with self.condition:
                active_goals = getattr(self, 'active_goals', None)
                active_commands = getattr(self, 'active_commands', None)
                if isinstance(active_goals, dict):
                    active_goals[arm_id] = None
                if isinstance(active_commands, dict):
                    active_commands[arm_id] = None
                if self.active_goal is goal_handle:
                    self.active_goal = None
                if self.active_command is goal:
                    self.active_command = None
                if arm_queue and arm_queue[0] is token:
                    arm_queue.popleft()
                elif token in arm_queue:
                    arm_queue.remove(token)
                # Released on failure as well as success: a stuck vehicle is
                # worse than an early release, and the operation result
                # already tells the operator the transfer did not finish.
                self.vehicle_commitments.pop(id(token), None)
                self.condition.notify_all()

    def _stop_arm2(self, _request, response):
        return self._stop_arm(
            'arm2', self.trigger_clients['stop_pick'], response
        )

    def _stop_arm1(self, _request, response):
        return self._stop_arm('arm1', self.arm1_stop_client, response)

    def _resume_arm1(self, _request, response):
        return self._resume_arm('arm1', response)

    def _resume_arm2(self, _request, response):
        return self._resume_arm('arm2', response)

    def _resume_arm(self, arm_id, response):
        """Clear the central STOPPED latch without commanding robot motion."""
        with self.condition:
            active_goals = getattr(self, 'active_goals', {})
            active = (
                active_goals.get(arm_id)
                if isinstance(active_goals, dict) else None
            )
            if active is not None and getattr(active, 'is_active', False):
                response.success = False
                response.message = f'{arm_id.upper()} stop is still settling'
                return response
            self._set_runtime_state(arm_id, ArmState.READY, 'READY', '')
            self.condition.notify_all()
        response.success = True
        response.message = f'{arm_id.upper()} dispatcher resumed'
        return response

    def _stop_arm(self, arm_id, client, response):
        # Invalidate running and queued commands before waiting for the remote
        # controller.  Otherwise a slow/unavailable stop service leaves a
        # window in which the next queued job can begin.
        with self.condition:
            self.stop_generations[arm_id] += 1
            self.vehicle_commitments.clear()
            self._set_runtime_state(
                arm_id, ArmState.STOPPED, 'STOP_REQUESTED', ''
            )
            self.condition.notify_all()
        accepted, message = self._call_service(
            client, Trigger.Request(), 'success'
        )
        response.success = accepted
        response.message = message
        with self.condition:
            if accepted:
                self._set_runtime_state(
                    arm_id, ArmState.STOPPED, 'STOPPED', ''
                )
            else:
                self._set_runtime_state(
                    arm_id, ArmState.ERROR, 'STOP_FAILED', message
                )
            self.condition.notify_all()
        return response


def main(args=None):
    rclpy.init(args=args)
    node = ArmDispatcher()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        executor.shutdown(timeout_sec=2.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
