#!/usr/bin/env python3

"""Start a port mission on vessel arrival and expose vehicle release events."""

from __future__ import annotations

import json
import os
from pathlib import Path
import threading
import time
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
from std_srvs.srv import Trigger


class AutonomyOrchestrator(Node):
    def __init__(self):
        super().__init__('autonomy_orchestrator')
        self.declare_parameter('auto_scan_arm2_on_start', True)
        self.declare_parameter('auto_scan_arm2_on_arrival', True)
        self.declare_parameter('arm2_scan_retry_sec', 10.0)
        self.declare_parameter('auto_scan_arm1_ship_on_start', True)
        self.declare_parameter(
            'arm1_ship_cache_state_path',
            '~/.local/state/port_control/arm1_ship_cache.json',
        )
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
        self.arm1_cache_status_client = self.create_client(
            Trigger,
            '/arm/pick_place/ship_cache_status',
            callback_group=self.callback_group,
        )
        self.arm2_cache_status_client = self.create_client(
            Trigger,
            '/arm2/destination_cache_status',
            callback_group=self.callback_group,
        )
        self.lock = threading.Lock()
        self.active_mission_id = ''
        self.arrival_event_id = ''
        self.scan_requested = False
        self.arm2_cache_ready = False
        self.arm2_cache_probe_pending = False
        self.arm2_cache_probe_complete = False
        self.arm2_scan_retry_sec = max(
            1.0, float(self.get_parameter('arm2_scan_retry_sec').value)
        )
        self.arm2_scan_retry_not_before = 0.0
        # Never issue a startup scan until the live ARM process has answered
        # whether its in-memory cache already exists.
        self.scan_retry_pending = False
        self.arm1_scan_requested = False
        self.arm1_cache_state_path = os.path.abspath(os.path.expanduser(str(
            self.get_parameter('arm1_ship_cache_state_path').value
        )))
        # The state file is diagnostic only. The live ARM process is the
        # authority because its cache intentionally lasts exactly as long as
        # that process does.
        self.arm1_cache_ready = False
        self.arm1_cache_probe_pending = False
        self.arm1_cache_probe_complete = False
        self.arm1_startup_scan_done = False
        self.inbound_scan_requested = False
        self.inbound_scan_pending = False
        self.create_timer(1.0, self._probe_arm_caches)
        self.create_timer(1.0, self._retry_scan_if_needed)
        self.create_timer(10.0, self._retry_arm1_ship_scan)
        self._publish_status('WAITING_FOR_VESSEL')

    def _retry_arm1_ship_scan(self):
        if not bool(self.get_parameter('auto_scan_arm1_ship_on_start').value):
            return
        with self.lock:
            if not self.arm1_cache_probe_complete:
                return
            if self.arm1_scan_requested:
                return
            if self.arm1_cache_ready:
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
            # A failed refresh does not erase ARM1's last known-good in-memory
            # cache: the coordinator replaces saved_marker_poses only after a
            # complete 19..23 scan.  Keep readiness unless ARM1 explicitly
            # reports that its cache is incomplete during an inbound request.
            if success:
                self.arm1_cache_ready = True
                self.arm1_cache_probe_complete = True
                self.arm1_startup_scan_done = True
                self._save_arm1_cache_state(True)
        self._publish_status(
            'ARM1_SHIP_CACHE_READY' if success else 'ARM1_SHIP_SCAN_RETRY',
            error=error,
        )

    def _probe_arm_caches(self):
        """Ask each live ARM process whether its fixed-marker cache exists."""
        with self.lock:
            probe_arm1 = not (
                self.arm1_cache_probe_complete
                or self.arm1_cache_probe_pending
            )
            probe_arm2 = not (
                self.arm2_cache_probe_complete
                or self.arm2_cache_probe_pending
            )
        if probe_arm1 and self.arm1_cache_status_client.wait_for_service(
            timeout_sec=0.0
        ):
            with self.lock:
                self.arm1_cache_probe_pending = True
            future = self.arm1_cache_status_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_arm1_cache_probe)
        if probe_arm2 and self.arm2_cache_status_client.wait_for_service(
            timeout_sec=0.0
        ):
            with self.lock:
                self.arm2_cache_probe_pending = True
            future = self.arm2_cache_status_client.call_async(Trigger.Request())
            future.add_done_callback(self._on_arm2_cache_probe)

    def _on_arm1_cache_probe(self, future):
        try:
            response = future.result()
        except Exception as exc:
            with self.lock:
                self.arm1_cache_probe_pending = False
            self._publish_status('WAITING_FOR_ARM1_CACHE_STATUS', error=str(exc))
            return
        ready = bool(response.success)
        with self.lock:
            self.arm1_cache_probe_pending = False
            self.arm1_cache_probe_complete = True
            self.arm1_cache_ready = ready
            self.arm1_startup_scan_done = ready
            self._save_arm1_cache_state(ready)
        self._publish_status(
            'ARM1_SHIP_CACHE_READY' if ready else 'ARM1_SHIP_SCAN_REQUIRED',
            note=str(response.message),
        )

    def _on_arm2_cache_probe(self, future):
        try:
            response = future.result()
        except Exception as exc:
            with self.lock:
                self.arm2_cache_probe_pending = False
            self._publish_status('WAITING_FOR_ARM2_CACHE_STATUS', error=str(exc))
            return
        ready = bool(response.success)
        with self.lock:
            self.arm2_cache_probe_pending = False
            self.arm2_cache_probe_complete = True
            self.arm2_cache_ready = ready
            self.scan_retry_pending = bool(
                not ready
                and self.get_parameter('auto_scan_arm2_on_start').value
            )
            self.arm2_scan_retry_not_before = 0.0
        self._publish_status(
            'ARM2_DESTINATION_CACHE_READY'
            if ready else 'ARM2_DESTINATION_SCAN_REQUIRED',
            note=str(response.message),
        )

    def _on_port_event(self, message):
        if message.event_type == PortEvent.VESSEL_DEPARTED:
            with self.lock:
                mission_id = self.active_mission_id
                self.active_mission_id = ''
                self.arrival_event_id = ''
                self.scan_retry_pending = bool(
                    not self.arm2_cache_ready
                    and not self.scan_requested
                    and self.get_parameter(
                        'auto_scan_arm2_on_start'
                    ).value
                )
                self.inbound_scan_requested = False
                self.inbound_scan_pending = False
            self._publish_status('VESSEL_DEPARTED', mission_id=mission_id)
            return
        if message.event_type != PortEvent.VESSEL_ARRIVED:
            return
        try:
            details = json.loads(message.details_json or '{}')
        except (TypeError, json.JSONDecodeError):
            details = {}
        cargo_added = details.get('change_type') == 'CARGO_ADDED'
        with self.lock:
            if self.arrival_event_id == message.event_id:
                return
            self.arrival_event_id = message.event_id
            if not self.active_mission_id:
                self.active_mission_id = f'port-{uuid.uuid4().hex[:10]}'
            mission_id = self.active_mission_id
            self.inbound_scan_pending = True
        self._publish_status(
            'CARGO_ADDED' if cargo_added else 'VESSEL_ARRIVED',
            mission_id=mission_id,
            detection_details=details,
        )
        if (
            bool(self.get_parameter('auto_scan_arm2_on_arrival').value)
            and not self.arm2_cache_ready
        ):
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
            arrival_event_id = self.arrival_event_id
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
            lambda done, mid=mission_id, eid=arrival_event_id:
            self._on_inbound_scan_accepted(
                mid, eid, done
            )
        )

    def _on_inbound_scan_accepted(self, mission_id, event_id, future):
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
            lambda done, mid=mission_id, eid=event_id:
            self._on_inbound_scan_result(mid, eid, done)
        )

    def _on_inbound_scan_result(self, mission_id, event_id, future):
        try:
            result = future.result().result
            success = bool(result.success)
            error = '' if success else str(result.message)
        except Exception as exc:
            success = False
            error = str(exc)
        with self.lock:
            self.inbound_scan_requested = False
            newer_arrival_waiting = self.arrival_event_id != event_id
            self.inbound_scan_pending = bool(
                not success or newer_arrival_waiting
            )
            if not success:
                # Missing container IDs are an ordinary retriable observation
                # failure and must not destroy the valid 19..23 slot cache.
                # Only ARM1's explicit cache-incomplete response proves that
                # its process restarted and the slot scan really is required.
                if self._failure_means_arm1_cache_missing(error):
                    self.arm1_cache_ready = False
                    self.arm1_cache_probe_complete = True
                    self.arm1_startup_scan_done = False
                    self._save_arm1_cache_state(False)
        self._publish_status(
            (
                'INBOUND_RESCAN_PENDING'
                if success and newer_arrival_waiting
                else 'INBOUND_SCAN_COMPLETE'
                if success else 'INBOUND_SCAN_RETRY'
            ),
            mission_id, error=error,
        )

    @staticmethod
    def _failure_means_arm1_cache_missing(error):
        text = str(error or '').lower()
        return bool(
            'ship marker cache' in text
            and ('incomplete' in text or 'missing' in text or 'lost' in text)
        )

    def _load_arm1_cache_state(self):
        path = getattr(self, 'arm1_cache_state_path', '')
        if not path:
            return False
        try:
            payload = json.loads(Path(path).read_text(encoding='utf-8'))
            return bool(payload.get('ready'))
        except (OSError, ValueError, TypeError, AttributeError):
            return False

    def _save_arm1_cache_state(self, ready):
        path = getattr(self, 'arm1_cache_state_path', '')
        if not path:
            return
        target = Path(path)
        try:
            target.parent.mkdir(parents=True, exist_ok=True)
            temporary = target.with_suffix(target.suffix + '.tmp')
            temporary.write_text(
                json.dumps({'ready': bool(ready)}, ensure_ascii=False),
                encoding='utf-8',
            )
            os.replace(temporary, target)
        except OSError as exc:
            self.get_logger().warning(
                f'ARM1 ship cache state persistence failed: {exc}'
            )

    def _request_arm2_scan(self, mission_id):
        with self.lock:
            if self.scan_requested:
                return
        if not self.arm_client.wait_for_server(timeout_sec=1.0):
            with self.lock:
                self.scan_retry_pending = True
                self.arm2_scan_retry_not_before = (
                    time.monotonic() + self.arm2_scan_retry_sec
                )
            self._publish_status(
                'WAITING_FOR_ARM2_DISPATCHER', mission_id=mission_id
            )
            return
        goal = DispatchArmCommand.Goal()
        # A failed dispatcher mission is intentionally blocked from issuing
        # later commands. Give every scan retry its own mission so a transient
        # marker failure can actually be retried.
        scan_attempt_id = (
            f'{mission_id}-destination-scan-{uuid.uuid4().hex[:6]}'
        )
        goal.command_id = scan_attempt_id
        goal.mission_id = scan_attempt_id
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
                self.arm2_scan_retry_not_before = (
                    time.monotonic() + self.arm2_scan_retry_sec
                )
            return
        if goal_handle is None or not goal_handle.accepted:
            self._publish_status('ARM2_SCAN_REJECTED', mission_id)
            with self.lock:
                self.scan_requested = False
                self.scan_retry_pending = True
                self.arm2_scan_retry_not_before = (
                    time.monotonic() + self.arm2_scan_retry_sec
                )
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
            with self.lock:
                self.scan_requested = False
                self.scan_retry_pending = True
                self.arm2_scan_retry_not_before = (
                    time.monotonic() + self.arm2_scan_retry_sec
                )
            self._publish_status(
                'ARM2_SCAN_FAILED', mission_id, error=str(exc)
            )
            return
        success = bool(result.success)
        with self.lock:
            self.scan_requested = False
            self.arm2_cache_ready = success
            self.arm2_cache_probe_complete = True
            self.scan_retry_pending = not success
            self.arm2_scan_retry_not_before = (
                0.0 if success
                else time.monotonic() + self.arm2_scan_retry_sec
            )
        if success:
            self._publish_status(
                'WAITING_FOR_CARGO_POLICY',
                mission_id,
                note='ARM2 destination cache ready',
            )
        else:
            self._publish_status(
                'ARM2_SCAN_RETRY', mission_id, error=result.message
            )

    def _retry_scan_if_needed(self):
        with self.lock:
            mission_id = self.active_mission_id
            should_retry = (
                self.arm2_cache_probe_complete
                and self.scan_retry_pending
                and not self.scan_requested
                and time.monotonic() >= self.arm2_scan_retry_not_before
            )
        if should_retry:
            scan_context = mission_id or f'arm2-init-{uuid.uuid4().hex[:6]}'
            self._request_arm2_scan(scan_context)
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
        if (
            str(result.get('arm_id', '')).lower() == 'arm2'
            and not result.get('success')
            and 'scan_destinations first'
            in str(result.get('message', '')).lower()
        ):
            with self.lock:
                self.arm2_cache_ready = False
                self.arm2_cache_probe_complete = True
                self.scan_retry_pending = True
                self.arm2_scan_retry_not_before = 0.0
            self._publish_status(
                'ARM2_DESTINATION_SCAN_REQUIRED',
                str(result.get('mission_id', '')),
                error=str(result.get('message', '')),
            )
        if not result.get('vehicle_release_allowed'):
            return
        if not result.get('success') or not result.get('vehicle_id'):
            return
        mission_id = str(result.get('mission_id', ''))
        payload = {
            'vehicle_id': result['vehicle_id'],
            'mission_id': mission_id,
            'command_id': result.get('command_id', ''),
            'operation_id': result.get('operation_id', ''),
            'state': 'RELEASE_ALLOWED',
            'reason': 'all declared cargo operations completed',
        }
        self._publish_status(
            'RELEASE_ALLOWED',
            mission_id,
            vehicle_id=result['vehicle_id'],
            command_id=result.get('command_id', ''),
            operation_id=result.get('operation_id', ''),
            reason='all declared cargo operations completed',
        )
        release = String()
        release.data = json.dumps(payload, ensure_ascii=False)
        self.release_publisher.publish(release)

    def _publish_status(self, state, mission_id='', **extra):
        message = String()
        payload = {
            'state': state,
            'mission_id': mission_id,
            'arm1_ship_cache_ready': bool(self.arm1_cache_ready),
            'arm2_destination_cache_ready': bool(self.arm2_cache_ready),
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
