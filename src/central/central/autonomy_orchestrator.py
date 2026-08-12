#!/usr/bin/env python3

"""Start a port mission on vessel arrival and expose vehicle release events."""

from __future__ import annotations

import json
import threading
import uuid

from porter_interfaces.action import DispatchArmCommand
from porter_interfaces.msg import PortEvent
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String


class AutonomyOrchestrator(Node):
    def __init__(self):
        super().__init__('autonomy_orchestrator')
        self.declare_parameter('auto_scan_arm2_on_arrival', True)
        self.declare_parameter('auto_scan_arm1_ship_on_start', True)
        self.callback_group = ReentrantCallbackGroup()
        qos = QoSProfile(
            depth=20,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.status_publisher = self.create_publisher(
            String, '/central/autonomy/mission_state', qos
        )
        self.release_publisher = self.create_publisher(
            String, '/central/autonomy/vehicle_release', qos
        )
        self.create_subscription(
            PortEvent,
            '/central/autonomy/port_events',
            self._on_port_event,
            qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            '/central/arms/results',
            self._on_arm_result,
            qos,
            callback_group=self.callback_group,
        )
        self.arm_client = ActionClient(
            self,
            DispatchArmCommand,
            '/central/arms/dispatch',
            callback_group=self.callback_group,
        )
        self.lock = threading.Lock()
        self.active_mission_id = ''
        self.arrival_event_id = ''
        self.scan_requested = False
        self.scan_retry_pending = False
        self.arm1_scan_requested = False
        self.arm1_cache_ready = False
        self.inbound_scan_requested = False
        self.inbound_scan_pending = False
        self.create_timer(1.0, self._retry_scan_if_needed)
        self.create_timer(10.0, self._retry_arm1_ship_scan)
        self._publish_status('WAITING_FOR_VESSEL')

    def _retry_arm1_ship_scan(self):
        if not bool(self.get_parameter('auto_scan_arm1_ship_on_start').value):
            return
        with self.lock:
            if self.arm1_cache_ready or self.arm1_scan_requested:
                return
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            self._publish_status('WAITING_FOR_ARM1_SHIP_SCAN')
            return
        goal = DispatchArmCommand.Goal()
        goal.command_id = f'arm1-ship-scan-{uuid.uuid4().hex[:8]}'
        goal.mission_id = goal.command_id
        goal.arm_id = 'arm1'
        goal.operation = 'scan_ship_destinations'
        with self.lock:
            self.arm1_scan_requested = True
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(self._on_arm1_scan_accepted)

    def _on_arm1_scan_accepted(self, future):
        try:
            handle = future.result()
        except Exception as exc:
            handle = None
            self._publish_status('ARM1_SHIP_SCAN_FAILED', error=str(exc))
        if handle is None or not handle.accepted:
            with self.lock:
                self.arm1_scan_requested = False
            return
        self._publish_status('ARM1_SHIP_MARKER_SCAN')
        handle.get_result_async().add_done_callback(self._on_arm1_scan_result)

    def _on_arm1_scan_result(self, future):
        try:
            result = future.result().result
            success = bool(result.success)
            error = '' if success else str(result.message)
        except Exception as exc:
            success = False
            error = str(exc)
        with self.lock:
            self.arm1_scan_requested = False
            self.arm1_cache_ready = success
        self._publish_status(
            'ARM1_SHIP_CACHE_READY' if success else 'ARM1_SHIP_SCAN_RETRY',
            error=error,
        )

    def _on_port_event(self, message):
        if message.event_type == PortEvent.VESSEL_DEPARTED:
            with self.lock:
                mission_id = self.active_mission_id
                self.active_mission_id = ''
                self.arrival_event_id = ''
                self.scan_requested = False
                self.scan_retry_pending = False
                self.inbound_scan_requested = False
                self.inbound_scan_pending = False
            self._publish_status('VESSEL_DEPARTED', mission_id=mission_id)
            return
        if message.event_type != PortEvent.VESSEL_ARRIVED:
            return
        with self.lock:
            if self.arrival_event_id == message.event_id:
                return
            self.arrival_event_id = message.event_id
            self.active_mission_id = f'port-{uuid.uuid4().hex[:10]}'
            mission_id = self.active_mission_id
            self.inbound_scan_pending = True
        self._publish_status('VESSEL_ARRIVED', mission_id=mission_id)
        if bool(self.get_parameter('auto_scan_arm2_on_arrival').value):
            self._request_arm2_scan(mission_id)
        self._request_arm1_inbound_scan(mission_id)

    def _request_arm1_inbound_scan(self, mission_id):
        with self.lock:
            if (
                not self.arm1_cache_ready
                or self.inbound_scan_requested
                or not self.inbound_scan_pending
            ):
                return
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            return
        goal = DispatchArmCommand.Goal()
        goal.command_id = f'{mission_id}-inbound-{uuid.uuid4().hex[:6]}'
        goal.mission_id = goal.command_id
        goal.arm_id = 'arm1'
        goal.operation = 'scan_inbound'
        with self.lock:
            self.inbound_scan_requested = True
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, mid=mission_id: self._on_inbound_scan_accepted(
                mid, done
            )
        )

    def _on_inbound_scan_accepted(self, mission_id, future):
        try:
            handle = future.result()
        except Exception as exc:
            handle = None
            self._publish_status('INBOUND_SCAN_FAILED', mission_id, error=str(exc))
        if handle is None or not handle.accepted:
            with self.lock:
                self.inbound_scan_requested = False
            return
        self._publish_status('SCANNING_INBOUND', mission_id)
        handle.get_result_async().add_done_callback(
            lambda done, mid=mission_id: self._on_inbound_scan_result(mid, done)
        )

    def _on_inbound_scan_result(self, mission_id, future):
        try:
            result = future.result().result
            success = bool(result.success)
            error = '' if success else str(result.message)
        except Exception as exc:
            success = False
            error = str(exc)
        with self.lock:
            self.inbound_scan_requested = False
            self.inbound_scan_pending = not success
            if not success:
                # A restarted ARM1 loses its in-memory ship cache. Rebuild it
                # before retrying the inbound scan; an already complete cache
                # makes the Trigger idempotent.
                self.arm1_cache_ready = False
        self._publish_status(
            'INBOUND_SCAN_COMPLETE' if success else 'INBOUND_SCAN_RETRY',
            mission_id, error=error,
        )

    def _request_arm2_scan(self, mission_id):
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            with self.lock:
                self.scan_retry_pending = True
            self._publish_status(
                'WAITING_FOR_ARM2_DISPATCHER', mission_id=mission_id
            )
            return
        goal = DispatchArmCommand.Goal()
        goal.command_id = f'{mission_id}-destination-scan'
        goal.mission_id = mission_id
        goal.arm_id = 'arm2'
        goal.operation = 'scan_destinations'
        with self.lock:
            self.scan_requested = True
            self.scan_retry_pending = False
        future = self.arm_client.send_goal_async(goal)
        future.add_done_callback(
            lambda done, mid=mission_id: self._on_scan_accepted(mid, done)
        )

    def _on_scan_accepted(self, mission_id, future):
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._publish_status(
                'ARM2_SCAN_REQUEST_FAILED', mission_id, error=str(exc)
            )
            with self.lock:
                self.scan_requested = False
                self.scan_retry_pending = True
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status('ARM2_SCAN_REJECTED', mission_id)
            with self.lock:
                self.scan_requested = False
                self.scan_retry_pending = True
            return
        self._publish_status('ARM2_DESTINATION_SCAN', mission_id)
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda done, mid=mission_id: self._on_scan_result(mid, done)
        )

    def _on_scan_result(self, mission_id, future):
        try:
            result = future.result().result
        except Exception as exc:
            self._publish_status(
                'ARM2_SCAN_FAILED', mission_id, error=str(exc)
            )
            return
        if result.success:
            self._publish_status(
                'WAITING_FOR_CARGO_POLICY',
                mission_id,
                note='ARM2 scan complete; ARM1 remains unconfigured',
            )
        else:
            self._publish_status(
                'WAITING_OPERATOR', mission_id, error=result.message
            )

    def _retry_scan_if_needed(self):
        with self.lock:
            mission_id = self.active_mission_id
            should_retry = (
                bool(mission_id)
                and self.scan_retry_pending
                and not self.scan_requested
            )
        if should_retry:
            self._request_arm2_scan(mission_id)
        with self.lock:
            should_scan_inbound = bool(
                mission_id and self.inbound_scan_pending
                and not self.inbound_scan_requested
            )
        if should_scan_inbound:
            self._request_arm1_inbound_scan(mission_id)

    def _on_arm_result(self, message):
        try:
            result = json.loads(message.data)
        except json.JSONDecodeError:
            return
        if not result.get('vehicle_release_allowed'):
            return
        if not result.get('success') or not result.get('vehicle_id'):
            return
        release = String()
        release.data = json.dumps({
            'vehicle_id': result['vehicle_id'],
            'mission_id': result.get('mission_id', ''),
            'command_id': result.get('command_id', ''),
            'operation_id': result.get('operation_id', ''),
            'state': 'RELEASE_ALLOWED',
            'reason': 'all declared cargo operations completed',
        }, ensure_ascii=False)
        self.release_publisher.publish(release)

    def _publish_status(self, state, mission_id='', **extra):
        message = String()
        payload = {
            'state': state,
            'mission_id': mission_id,
            'arm1_ship_cache_ready': bool(self.arm1_cache_ready),
            'inbound_scan_pending': bool(self.inbound_scan_pending),
        }
        payload.update(extra)
        message.data = json.dumps(payload, ensure_ascii=False)
        self.status_publisher.publish(message)
        self.get_logger().info(message.data)


def main(args=None):
    rclpy.init(args=args)
    node = AutonomyOrchestrator()
    executor = MultiThreadedExecutor(num_threads=4)
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
