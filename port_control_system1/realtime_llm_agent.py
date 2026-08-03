"""Event-driven VLM supervisor for the desktop fleet dashboard."""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
from dataclasses import dataclass, replace
from typing import Dict, Optional

import cv2

from cargo_dispatch_tool import (
    load_cargo_details,
    load_cargo_registry,
    load_named_locations,
)
from cctv_monitor_view import CCTVMonitorView
from central_control_client import (
    CentralControlApiError,
    CentralControlClient,
)
from llm_command_parser import (
    LLMParseError,
    parse_command_with_llm,
    resolve_execution_mode,
)
from visual_navigation import (
    VisualNavigationError,
    compact_detections,
    resolve_detection_approach,
    select_nearest_visible_vehicle,
    validate_pixel_navigation,
    zone_mode_for_label,
)
from yolo_detection_client import (
    YoloDetectionClient,
    YoloDetectionError,
)


VEHICLE_IDS = ('agv1', 'agv2')
READY_STATE = 'READY'


@dataclass(frozen=True)
class RealtimeAgentSnapshot:
    """Thread-safe status exposed to the dashboard UI."""

    enabled: bool = True
    state: str = 'WAITING_FOR_OBJECTIVE'
    objective: str = ''
    last_decision: str = ''
    last_error: str = ''
    last_evaluation_at: float = 0.0
    dispatched_actions: int = 0


def _rounded(value, quantum):
    return round(float(value) / quantum) * quantum


def observation_signature(detection_summary, fleet_status):
    """Return a stable signature that changes on meaningful scene events."""
    detections = []
    for item in compact_detections(detection_summary):
        bbox = item.get('bbox_xyxy') or [0.0, 0.0, 0.0, 0.0]
        center_x = (float(bbox[0]) + float(bbox[2])) * 0.5
        center_y = (float(bbox[1]) + float(bbox[3])) * 0.5
        detections.append((
            str(item.get('label', '')),
            _rounded(center_x, 20.0),
            _rounded(center_y, 20.0),
        ))
    telemetry = (fleet_status or {}).get('telemetry', {})
    vehicles = telemetry.get('vehicles', {})
    vehicle_state = []
    for vehicle_id in VEHICLE_IDS:
        value = vehicles.get(vehicle_id) or {}
        pose = value.get('pose') or {}
        vehicle_state.append((
            vehicle_id,
            str(value.get('state', 'UNKNOWN')),
            str(value.get('current_command_id', '')),
            str(value.get('locked_zone', '')),
            bool(value.get('emergency_stopped', False)),
            _rounded(pose.get('x', 0.0), 0.10),
            _rounded(pose.get('y', 0.0), 0.10),
        ))
    payload = {
        'detections': sorted(detections),
        'vehicles': vehicle_state,
        'zones': str(telemetry.get('b1_zone', '')),
    }
    encoded = json.dumps(payload, sort_keys=True, separators=(',', ':'))
    return hashlib.sha256(encoded.encode('utf-8')).hexdigest()


def action_signature(action):
    """Identify one normalized action for objective-level deduplication."""
    if not isinstance(action, dict):
        return ''
    relevant = {
        key: action.get(key)
        for key in (
            'type',
            'vehicle_id',
            'detection_index',
            'source_detection_index',
            'destination_detection_index',
            'approach_side',
            'target',
            'heading',
        )
        if key in action
    }
    return json.dumps(relevant, sort_keys=True, separators=(',', ':'))


def supervision_command(objective, fleet_status, previous_decision):
    """Build the stateful instruction supplied to the existing VLM parser."""
    telemetry = (fleet_status or {}).get('telemetry', {})
    vehicles = telemetry.get('vehicles', {})
    context = {
        'vehicles': vehicles,
        'zones': telemetry.get('b1_zone', ''),
        'last_map_target': telemetry.get('last_map_target'),
    }
    return (
        '[지속 실시간 관제 모드]\n'
        f'사용자의 지속 목표: {objective}\n'
        f'현재 Fleet 상태 JSON: {json.dumps(context, ensure_ascii=False)}\n'
        f'직전 자동 판단: {previous_decision or "없음"}\n\n'
        '현재 영상과 YOLO JSON 및 Fleet 상태를 함께 판단하세요. 이미 같은 목표로 '
        '이동 중이거나 도착한 차량에는 명령을 반복하지 마세요. 유휴 차량에 지금 '
        '새로 필요한 이동만 action으로 반환하세요. 두 차량의 독립 작업은 parallel로 '
        '반환하세요. 구역 대기열과 점유 해제 후 진입은 fleet dispatcher가 자동으로 '
        '처리하므로 같은 명령을 재전송하지 마세요. 현재 추가 동작이 필요하지 않으면 '
        'unknown action 하나만 반환하세요.'
    )


class RealtimeLLMAgent:
    """Continuously reassess one operator objective against live fleet state."""

    _instance: Optional['RealtimeLLMAgent'] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls):
        """Return the process-wide supervisor."""
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self):
        self.interval_sec = max(
            0.5,
            float(os.environ.get('PORT_CONTROL_REALTIME_LLM_INTERVAL_SEC', '2.0')),
        )
        self.heartbeat_sec = max(
            self.interval_sec,
            float(os.environ.get('PORT_CONTROL_REALTIME_LLM_HEARTBEAT_SEC', '5.0')),
        )
        self.initial_delay_sec = max(
            0.0,
            float(os.environ.get('PORT_CONTROL_REALTIME_LLM_INITIAL_DELAY_SEC', '5.0')),
        )
        enabled = os.environ.get(
            'PORT_CONTROL_REALTIME_LLM_ENABLED', 'true'
        ).strip().lower() in {'1', 'true', 'yes', 'on'}
        self._lock = threading.RLock()
        self._snapshot = RealtimeAgentSnapshot(enabled=enabled)
        self._stop_event = threading.Event()
        self._wake_event = threading.Event()
        self._thread = None
        self._objective_revision = 0
        self._sent_actions = set()
        self._last_observation = ''
        self._last_evaluation_monotonic = 0.0
        self._not_before_monotonic = 0.0

    def start(self):
        """Start the background evaluation loop once."""
        if self._thread is not None and self._thread.is_alive():
            return
        CCTVMonitorView.ensure_capture_running()
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name='port-control-realtime-llm',
            daemon=True,
        )
        self._thread.start()

    def stop(self):
        """Stop the background loop."""
        self._stop_event.set()
        self._wake_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None

    def snapshot(self):
        """Return an immutable copy of current supervisor status."""
        with self._lock:
            return replace(self._snapshot)

    def set_enabled(self, enabled):
        """Enable or suspend autonomous VLM decisions."""
        with self._lock:
            state = (
                'WAITING_FOR_OBJECTIVE'
                if enabled and not self._snapshot.objective
                else 'MONITORING' if enabled else 'DISABLED'
            )
            self._snapshot = replace(
                self._snapshot,
                enabled=bool(enabled),
                state=state,
            )
        self._wake_event.set()

    def set_objective(self, objective, initial_actions=None):
        """Replace the persistent operator objective and reset deduplication."""
        objective = str(objective or '').strip()
        if not objective:
            return
        with self._lock:
            self._objective_revision += 1
            self._sent_actions = {
                action_signature(action)
                for action in (initial_actions or [])
                if action_signature(action)
            }
            self._last_observation = ''
            self._last_evaluation_monotonic = time.monotonic()
            self._not_before_monotonic = (
                self._last_evaluation_monotonic + self.initial_delay_sec
            )
            self._snapshot = replace(
                self._snapshot,
                objective=objective,
                state=(
                    'MONITORING'
                    if self._snapshot.enabled else 'DISABLED'
                ),
                last_decision='사용자 명령을 지속 목표로 등록',
                last_error='',
            )
        print(f'[실시간 LLM 관제] 지속 목표 등록: {objective}')
        self._wake_event.set()

    def _update_snapshot(self, **changes):
        with self._lock:
            self._snapshot = replace(self._snapshot, **changes)

    def _run(self):
        while not self._stop_event.is_set():
            self._wake_event.wait(timeout=self.interval_sec)
            self._wake_event.clear()
            snapshot = self.snapshot()
            if not snapshot.enabled or not snapshot.objective:
                continue
            with self._lock:
                if time.monotonic() < self._not_before_monotonic:
                    continue
            try:
                self._evaluate(snapshot.objective)
            except Exception as exc:
                message = f'unexpected supervisor failure: {exc}'
                self._update_snapshot(state='ERROR', last_error=message)
                print(f'[실시간 LLM 관제 오류] {message}')

    def _evaluate(self, objective):
        shared_frame = CCTVMonitorView.SHARED_FRAME
        if shared_frame is None:
            self._update_snapshot(
                state='WAITING_FOR_IMAGE',
                last_error='최신 CCTV 프레임 없음',
            )
            return
        # Capture runs on another thread. Work from one immutable snapshot so
        # JPEG encoding and the matching YOLO result see a coherent frame.
        frame = shared_frame.copy()
        try:
            detection_summary = YoloDetectionClient().get_latest()
            fleet_status = CentralControlClient(timeout_sec=3.0).status()
        except Exception as exc:
            self._update_snapshot(state='WAITING_FOR_DATA', last_error=str(exc))
            return

        signature = observation_signature(detection_summary, fleet_status)
        now = time.monotonic()
        with self._lock:
            changed = signature != self._last_observation
            heartbeat_due = (
                now - self._last_evaluation_monotonic >= self.heartbeat_sec
            )
            if not changed and not heartbeat_due:
                return
            self._last_observation = signature
            self._last_evaluation_monotonic = now
            previous_decision = self._snapshot.last_decision
            revision = self._objective_revision

        height, width = frame.shape[:2]
        success, encoded = cv2.imencode(
            '.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 75]
        )
        if not success:
            self._update_snapshot(state='ERROR', last_error='JPEG encode failed')
            return
        compact = compact_detections(detection_summary)
        cargo_registry = load_cargo_registry()
        cargo_details = load_cargo_details()
        known_types = sorted({
            detail.get('화물종류', '')
            for detail in cargo_details.values()
            if detail.get('화물종류')
        })
        prompt = supervision_command(
            objective, fleet_status, previous_decision
        )
        self._update_snapshot(state='EVALUATING', last_error='')
        try:
            result = parse_command_with_llm(
                prompt,
                list(cargo_registry.keys()),
                list(load_named_locations().keys()),
                known_types,
                image_jpeg=encoded.tobytes(),
                image_width=width,
                image_height=height,
                yolo_detections=compact,
                normalization_command=objective,
            )
        except LLMParseError as exc:
            self._update_snapshot(state='ERROR', last_error=str(exc))
            print(f'[실시간 LLM 관제 오류] {exc}')
            return

        with self._lock:
            if revision != self._objective_revision:
                return
        actions = result.get('actions') or []
        actionable = [
            action for action in actions
            if isinstance(action, dict) and action.get('type') != 'unknown'
        ]
        if not actionable:
            self._update_snapshot(
                state='MONITORING',
                last_decision='추가 동작 없음',
                last_evaluation_at=time.time(),
            )
            return

        execution_mode = resolve_execution_mode(objective, result)
        predecessor = ''
        dispatched = 0
        decisions = []
        reserved_vehicles = set()
        for action in actionable:
            fingerprint = action_signature(action)
            with self._lock:
                if fingerprint in self._sent_actions:
                    decisions.append(f'중복 생략:{action.get("type")}')
                    continue
            command_id = self._dispatch_action(
                action,
                detection_summary,
                fleet_status,
                width,
                height,
                '' if execution_mode == 'parallel' else predecessor,
                (
                    reserved_vehicles
                    if execution_mode == 'parallel' else set()
                ),
            )
            if command_id:
                predecessor = command_id
                dispatched += 1
                with self._lock:
                    self._sent_actions.add(fingerprint)
                decisions.append(
                    f'{action.get("type")}:{action.get("vehicle_id") or "AUTO"}'
                )

        decision_text = ', '.join(decisions) or '실행 가능한 새 동작 없음'
        with self._lock:
            total = self._snapshot.dispatched_actions + dispatched
        self._update_snapshot(
            state='MONITORING',
            last_decision=decision_text,
            last_evaluation_at=time.time(),
            dispatched_actions=total,
            last_error='',
        )
        print(
            f'[실시간 LLM 관제] mode={execution_mode}, '
            f'dispatched={dispatched}, decision={decision_text}'
        )

    @staticmethod
    def _vehicle_states(fleet_status):
        telemetry = (fleet_status or {}).get('telemetry', {})
        return telemetry.get('vehicles', {})

    def _resolve_vehicle(
        self,
        requested,
        selected_detection,
        summary,
        status,
        reserved_vehicles,
    ):
        states = self._vehicle_states(status)
        requested = str(requested or '').strip().lower()
        if requested in VEHICLE_IDS:
            value = states.get(requested) or {}
            if (
                value.get('state') == READY_STATE
                and requested not in reserved_vehicles
            ):
                reserved_vehicles.add(requested)
                return requested
            return ''
        ready = {
            vehicle_id for vehicle_id in VEHICLE_IDS
            if (states.get(vehicle_id) or {}).get('state') == READY_STATE
            and vehicle_id not in reserved_vehicles
        }
        if not ready:
            return ''
        selected = ''
        if selected_detection is not None:
            selected = select_nearest_visible_vehicle(
                selected_detection, summary, ready
            )
        selected = selected or sorted(ready)[0]
        reserved_vehicles.add(selected)
        return selected

    def _dispatch_action(
        self,
        action,
        detection_summary,
        fleet_status,
        image_width,
        image_height,
        predecessor,
        reserved_vehicles,
    ):
        action_type = action.get('type')
        if action_type == 'visual_navigation':
            try:
                target, heading, selected = resolve_detection_approach(
                    action,
                    detection_summary,
                    image_width,
                    image_height,
                )
                vehicle_id = self._resolve_vehicle(
                    action.get('vehicle_id'),
                    selected,
                    detection_summary,
                    fleet_status,
                    reserved_vehicles,
                )
                if not vehicle_id:
                    return ''
                mode = zone_mode_for_label(selected['label'])
                response = CentralControlClient().send_pixel_goal(
                    target,
                    heading,
                    predecessor_command_id=predecessor,
                    mode=mode,
                    vehicle_id=vehicle_id,
                    zone_visually_empty=(
                        mode in {'parking_a', 'parking_b1'}
                    ),
                    queue_if_busy=bool(predecessor),
                )
                return response.get('command_id', '')
            except (VisualNavigationError, CentralControlApiError):
                return ''

        if action_type == 'pixel_navigation':
            target = action.get('target')
            heading = action.get('heading')
            try:
                validate_pixel_navigation(
                    target,
                    heading,
                    image_width,
                    image_height,
                    detection_summary,
                )
                vehicle_id = self._resolve_vehicle(
                    action.get('vehicle_id'),
                    None,
                    detection_summary,
                    fleet_status,
                    reserved_vehicles,
                )
                if not vehicle_id:
                    return ''
                response = CentralControlClient().send_pixel_goal(
                    target,
                    heading,
                    predecessor_command_id=predecessor,
                    vehicle_id=vehicle_id,
                    queue_if_busy=bool(predecessor),
                )
                return response.get('command_id', '')
            except (VisualNavigationError, CentralControlApiError):
                return ''

        if action_type == 'visual_transfer':
            source_action = {
                'detection_index': action.get('source_detection_index'),
                'approach_side': 'bottom',
            }
            destination_action = {
                'detection_index': action.get('destination_detection_index'),
                'approach_side': 'bottom',
            }
            try:
                source_target, source_heading, source = (
                    resolve_detection_approach(
                        source_action,
                        detection_summary,
                        image_width,
                        image_height,
                    )
                )
                destination_target, destination_heading, destination = (
                    resolve_detection_approach(
                        destination_action,
                        detection_summary,
                        image_width,
                        image_height,
                    )
                )
                vehicle_id = self._resolve_vehicle(
                    action.get('vehicle_id'),
                    source,
                    detection_summary,
                    fleet_status,
                    reserved_vehicles,
                )
                if not vehicle_id:
                    return ''
                client = CentralControlClient()
                source_mode = zone_mode_for_label(source['label'])
                source_response = client.send_pixel_goal(
                    source_target,
                    source_heading,
                    predecessor_command_id=predecessor,
                    mode=source_mode,
                    vehicle_id=vehicle_id,
                    zone_visually_empty=True,
                    queue_if_busy=bool(predecessor),
                )
                source_command = source_response.get('command_id', '')
                if not source_command:
                    return ''
                destination_mode = zone_mode_for_label(destination['label'])
                destination_response = client.send_pixel_goal(
                    destination_target,
                    destination_heading,
                    predecessor_command_id=source_command,
                    mode=destination_mode,
                    vehicle_id=vehicle_id,
                    zone_visually_empty=True,
                    queue_if_busy=True,
                )
                return destination_response.get('command_id', '')
            except (VisualNavigationError, CentralControlApiError):
                return ''
        return ''
