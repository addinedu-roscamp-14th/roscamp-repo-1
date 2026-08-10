#!/usr/bin/env python3

"""Queue robot-arm commands and correlate them with structured ARM events."""

from __future__ import annotations

from collections import deque
import json
import threading
import time

from arm2_interfaces.srv import TransferById
from porter_interfaces.action import DispatchArmCommand
from porter_interfaces.msg import ArmState
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


def is_terminal_event(event):
    """Return true only for an operation-level terminal phase.

    ARM2 uses ``state=COMPLETED`` for successful intermediate phases such as
    SOURCE_LOCKED. Therefore state must never terminate the central command.
    """
    return str(event.get('phase', '')).upper() in TERMINAL_PHASES


class ArmDispatcher(Node):
    """Expose one central action while keeping physical ARM calls serialized."""

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
        self.declare_parameter('state_publish_rate_hz', 2.0)
        self.declare_parameter('service_wait_timeout_sec', 5.0)
        self.declare_parameter('service_retry_count', 3)
        self.declare_parameter('operation_timeout_sec', 600.0)
        self.declare_parameter('scan_timeout_sec', 240.0)

        self.callback_group = ReentrantCallbackGroup()
        self.condition = threading.Condition()
        self.queue = deque()
        self.events = deque(maxlen=500)
        self.latest_sequence = 0
        self.latest_event_at = None
        self.active_goal = None
        self.active_command = None
        self.stop_generation = 0
        self.failed_missions = set()
        self.arm2_last_error = ''
        self.arm2_state = ArmState.OFFLINE
        self.arm2_state_text = 'waiting for ARM2 event or service'

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
        self.result_publisher = self.create_publisher(
            String, '/central/arms/results', event_qos
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

        self.create_service(
            Trigger,
            '/central/arms/arm2/stop',
            self._stop_arm2,
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
            'ARM dispatcher ready: ARM1=UNCONFIGURED, ARM2=/arm2 services'
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

    def _publish_states(self):
        now = self.get_clock().now().to_msg()
        arm1 = ArmState()
        arm1.header.stamp = now
        arm1.header.frame_id = 'arm1'
        arm1.arm_id = 'arm1'
        arm1.state = ArmState.UNCONFIGURED
        arm1.state_text = 'ARM1 service contract is not configured'
        arm1.ready = False
        arm1.last_error = arm1.state_text
        arm1.telemetry_age_sec = float('inf')
        self.arm1_state_publisher.publish(arm1)

        arm2 = ArmState()
        arm2.header.stamp = now
        arm2.header.frame_id = 'arm2'
        arm2.arm_id = 'arm2'
        arm2.state = self.arm2_state
        arm2.state_text = self.arm2_state_text
        arm2.ready = self.arm2_state == ArmState.READY
        arm2.last_error = self.arm2_last_error
        if self.latest_event_at is None:
            arm2.telemetry_age_sec = float('inf')
        else:
            arm2.telemetry_age_sec = float(
                time.monotonic() - self.latest_event_at
            )
        if self.active_command is not None:
            command = self.active_command
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
            return None, 'ARM1 service contract is not configured'
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
        if operation == 'transfer_by_id':
            request = TransferById.Request()
            request.source_id = int(goal.source_id)
            request.destination_id = int(goal.destination_id)
            return self.transfer_by_id_client, request, 'accepted'
        if operation == 'transfer_to_slot':
            slot = str(goal.destination_slot).lower().replace('-', '')
            key = f'transfer_{slot[0:2]}_{slot[2]}'
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
                if command_generation != self.stop_generation:
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
                    phase = feedback.phase.upper()
                    if is_terminal_event(event):
                        return event, operation_id, ''
                self.condition.wait(timeout=min(0.25, deadline - time.monotonic()))
        return None, operation_id, f'{operation} timed out after {timeout:.1f}s'

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
        }
        message_out = String()
        message_out.data = json.dumps(payload, ensure_ascii=False)
        self.result_publisher.publish(message_out)

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
        self._publish_result(goal, False, operation_id, message, False)
        return result

    def _execute(self, goal_handle):
        goal = goal_handle.request
        operation, error = self._validate(goal)
        if operation is None:
            code = (
                self.ERROR_UNCONFIGURED
                if str(goal.arm_id).lower() == 'arm1'
                else self.ERROR_INVALID_REQUEST
            )
            return self._finish_error(goal_handle, goal, code, error)
        if str(goal.mission_id) in self.failed_missions:
            return self._finish_error(
                goal_handle,
                goal,
                self.ERROR_OPERATION_FAILED,
                f'mission {goal.mission_id} already failed; command skipped',
            )

        token = object()
        with self.condition:
            command_generation = self.stop_generation
            self.queue.append(token)
            while self.queue and self.queue[0] is not token:
                if (
                    goal_handle.is_cancel_requested
                    or command_generation != self.stop_generation
                ):
                    self.queue.remove(token)
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
            self.active_goal = goal_handle
            self.active_command = goal
            self.arm2_state = ArmState.BUSY
            self.arm2_state_text = 'DISPATCHING'
            baseline_sequence = self.latest_sequence

        try:
            client, request, accepted_field = self._service_for_goal(
                goal, operation
            )
            accepted, service_message = self._call_service(
                client, request, accepted_field
            )
            with self.condition:
                stopped = command_generation != self.stop_generation
            if stopped:
                return self._finish_error(
                    goal_handle,
                    goal,
                    self.ERROR_CANCELED,
                    'command stopped by operator',
                )
            if not accepted:
                self.arm2_state = ArmState.ERROR
                self.arm2_last_error = service_message
                return self._finish_error(
                    goal_handle,
                    goal,
                    self.ERROR_REJECTED,
                    service_message or 'ARM2 rejected the command',
                )

            operation_id = ''
            if operation in EVENT_OPERATIONS:
                event, operation_id, wait_error = self._wait_for_terminal_event(
                    goal_handle,
                    operation,
                    baseline_sequence,
                    EVENT_OPERATIONS[operation],
                    command_generation,
                )
                if event is None:
                    with self.condition:
                        stopped = command_generation != self.stop_generation
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
                    self.arm2_state = ArmState.WAITING_OPERATOR
                    self.arm2_last_error = str(event.get('error') or terminal_message)
                    return self._finish_error(
                        goal_handle,
                        goal,
                        self.ERROR_OPERATION_FAILED,
                        terminal_message or 'ARM2 operation failed',
                        operation_id,
                    )
            else:
                success = True
                terminal_message = service_message

            release = bool(
                goal.final_for_vehicle
                and str(goal.vehicle_id)
                and operation in {'transfer_to_slot', 'load_to_trailer'}
            )
            result = DispatchArmCommand.Result()
            result.success = True
            result.error_code = 0
            result.message = terminal_message or 'ARM2 operation completed'
            result.operation_id = operation_id
            result.vehicle_release_allowed = release
            self.arm2_state = ArmState.READY
            self.arm2_state_text = 'READY'
            self.arm2_last_error = ''
            goal_handle.succeed()
            self._publish_result(
                goal, True, operation_id, result.message, release
            )
            return result
        finally:
            with self.condition:
                self.active_goal = None
                self.active_command = None
                if self.queue and self.queue[0] is token:
                    self.queue.popleft()
                elif token in self.queue:
                    self.queue.remove(token)
                self.condition.notify_all()

    def _stop_arm2(self, _request, response):
        client = self.trigger_clients['stop_pick']
        accepted, message = self._call_service(
            client, Trigger.Request(), 'success'
        )
        response.success = accepted
        response.message = message
        if accepted:
            with self.condition:
                self.stop_generation += 1
                self.arm2_state = ArmState.STOPPED
                self.arm2_state_text = 'STOP_REQUESTED'
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
