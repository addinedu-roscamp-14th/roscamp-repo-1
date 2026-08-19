"""Event-driven VLM supervisor for the desktop fleet dashboard."""

from __future__ import annotations

import hashlib
import json
import os
import threading
import time
import uuid
from dataclasses import dataclass, replace
from typing import Dict, Optional

import cv2

from central_control_client import (
    CentralControlApiError,
    CentralControlClient,
)
from llm_command_parser import (
    LLMParseError,
    parse_command_with_llm,
    resolve_execution_mode,
)
from inventory_client import InventoryClientError
from inventory_decision_planner import InventoryDecisionPlanner
from autonomous_inventory import (
    AutonomousCycle,
    AutonomousPolicyError,
    CANONICAL_LOCATIONS,
    CycleStore,
    SHIP_LOCATIONS,
    choose_policy,
    compile_move,
    validate_first_move,
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


def _cctv_monitor_view():
    """Import GUI-backed capture only when visual supervision needs it."""
    from cctv_monitor_view import CCTVMonitorView
    return CCTVMonitorView


def _load_cargo_context():
    """Load legacy local cargo context for visual control mode."""
    from cargo_dispatch_tool import load_cargo_details, load_cargo_registry
    return load_cargo_registry(), load_cargo_details()


def _load_registered_locations():
    """Load destination names without coupling module import to the GUI."""
    from cargo_dispatch_tool import load_named_locations
    return load_named_locations()


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
    mode: str = 'control'
    cycle_id: str = ''
    phase: str = ''
    db_snapshot_id: str = ''
    next_move: str = ''
    active_command: str = ''
    llm_plan_json: str = ''
    execution_steps_json: str = ''
    current_step_json: str = ''
    command_payload_json: str = ''
    db_sync_state: str = 'UNKNOWN'
    db_pending_count: int = 0
    replan_count: int = 0


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

    def __init__(
        self, inventory_planner=None, location_loader=None, cycle_store=None
    ):
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
        self.arm_command_orphan_timeout_sec = max(
            10.0,
            float(os.environ.get(
                'PORT_AUTONOMY_ARM_COMMAND_ORPHAN_TIMEOUT_SEC', '120.0'
            )),
        )
        self.nav_command_retry_grace_sec = max(
            1.0,
            float(os.environ.get(
                'PORT_AUTONOMY_NAV_RETRY_GRACE_SEC', '5.0'
            )),
        )
        self.nav_command_max_resends = max(
            0,
            int(os.environ.get(
                'PORT_AUTONOMY_NAV_MAX_RESENDS', '0'
            )),
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
        self._last_inventory_snapshot_id = ''
        self.inventory_planner = (
            inventory_planner or InventoryDecisionPlanner()
        )
        self.location_loader = location_loader or _load_registered_locations
        self.cycle_store = cycle_store or CycleStore()
        # A dashboard process restart is an explicit new operating session.
        # Do not restore inbound/outbound IDs, an in-flight move, or failure
        # counters from the previous process. The PostgreSQL inventory remains
        # the source of truth and will be reassessed when autonomy starts.
        self._cycle = AutonomousCycle()
        # A parked vehicle normally reports READY with no zone lock, which is
        # indistinguishable from an idle unparked vehicle in fleet telemetry.
        # Remember successful/requested parking locally so the waiting loop
        # cannot enqueue a new park command on every heartbeat.
        self._autonomy_park_requests = set()
        self._diagnostic_path = os.path.abspath(os.path.expanduser(
            os.environ.get(
                'PORT_AUTONOMY_DIAGNOSTIC_PATH',
                '~/.local/state/port_control/autonomy_status.json',
            )
        ))

    def start(self):
        """Start the background evaluation loop once."""
        if self._thread is not None and self._thread.is_alive():
            return
        _cctv_monitor_view().ensure_capture_running()
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
            self._last_inventory_snapshot_id = ''
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
                mode='control',
            )
        print(f'[실시간 LLM 관제] 지속 목표 등록: {objective}')
        self._wake_event.set()

    def set_inventory_objective(self, objective, initial_plan=None):
        """Continuously reassess a read-only inventory objective."""
        objective = str(objective or '').strip()
        if not objective:
            return
        initial_plan = initial_plan if isinstance(initial_plan, dict) else None
        with self._lock:
            self._objective_revision += 1
            self._sent_actions = set()
            self._last_observation = ''
            self._last_inventory_snapshot_id = str(
                (initial_plan or {}).get('snapshot_id', '')
            )
            self._last_evaluation_monotonic = time.monotonic()
            self._not_before_monotonic = self._last_evaluation_monotonic
            decision = (
                json.dumps(initial_plan, ensure_ascii=False, indent=2)
                if initial_plan else '원격 DB 기반 지속 판단 목표 등록'
            )
            self._snapshot = replace(
                self._snapshot,
                objective=objective,
                state='MONITORING' if self._snapshot.enabled else 'DISABLED',
                last_decision=decision,
                last_error=str((initial_plan or {}).get('error', '')),
                mode='inventory',
            )
        print(f'[실시간 LLM 재고 판단] 지속 목표 등록: {objective}')
        self._wake_event.set()

    def start_autonomous_policy(self):
        """Resume autonomy by reassessing current persisted and live state."""
        # Starting again is an explicit operator retry.  Keep the cycle's
        # inbound/outbound identity and any in-flight physical move, but never
        # force the phase back to WAITING_FOR_INBOUND: that discarded a vessel,
        # cargo, or vehicle already present when the button was pressed.
        self._cycle.identical_failures = 0
        self._cycle.failure_key = ''
        self._cycle.last_error = ''
        self._cycle.phase = (
            'EXECUTING_MOVE'
            if self._cycle.active_move
            else 'REASSESSING_CURRENT_STATE'
        )
        self._save_cycle()
        with self._lock:
            self._objective_revision += 1
            self._last_observation = ''
            self._last_inventory_snapshot_id = ''
            self._last_evaluation_monotonic = 0.0
            self._not_before_monotonic = 0.0
            self._snapshot = replace(
                self._snapshot,
                enabled=True,
                objective='DB 기반 완전 자율 항만 관제',
                state='MONITORING',
                mode='autonomous',
                cycle_id=self._cycle.cycle_id,
                phase=self._cycle.phase,
                last_decision=(
                    '자율 관제 재시작: 최신 DB·ROI·Fleet·ARM 상태를 '
                    '다시 확인합니다.'
                ),
                last_error='',
                llm_plan_json='',
                execution_steps_json='',
                current_step_json='',
                command_payload_json='',
            )
        self._wake_event.set()

    def stop_autonomous_policy(self):
        """Stop creating new plans; an already running ARM call may finish."""
        with self._lock:
            self._objective_revision += 1
            self._snapshot = replace(
                self._snapshot,
                enabled=False,
                state='DISABLED',
                objective='',
                last_decision='운영자가 자율 관제 신규 계획을 중지함',
            )
        self._wake_event.set()

    def _update_snapshot(self, **changes):
        with self._lock:
            self._snapshot = replace(self._snapshot, **changes)
            snapshot = self._snapshot
        self._write_diagnostic_snapshot(snapshot)

    def _write_diagnostic_snapshot(self, snapshot):
        """Persist transient planner errors that otherwise disappear in UI."""
        payload = dict(snapshot.__dict__)
        payload['updated_at'] = time.time()
        try:
            parent = os.path.dirname(self._diagnostic_path)
            os.makedirs(parent, exist_ok=True)
            temporary = f'{self._diagnostic_path}.tmp'
            with open(temporary, 'w', encoding='utf-8') as stream:
                json.dump(payload, stream, ensure_ascii=False, indent=2)
            os.replace(temporary, self._diagnostic_path)
        except OSError as exc:
            print(f'[자율 관제 진단 기록 실패] {exc}', flush=True)
        if changes_error := str(payload.get('last_error') or ''):
            print(
                '[DB 기반 자율 관제 오류] '
                f'state={payload.get("state")}, '
                f'phase={payload.get("phase")}, error={changes_error}',
                flush=True,
            )

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
                if snapshot.mode == 'autonomous':
                    self._evaluate_autonomous()
                elif snapshot.mode == 'inventory':
                    self._evaluate_inventory(snapshot.objective)
                else:
                    self._evaluate(snapshot.objective)
            except Exception as exc:
                message = f'unexpected supervisor failure: {exc}'
                self._update_snapshot(state='ERROR', last_error=message)
                print(f'[실시간 LLM 관제 오류] {message}')

    def _evaluate_inventory(self, objective):
        """Generate a plan without reading ROS state or dispatching commands."""
        try:
            inventory = self.inventory_planner.inventory_client.fetch_snapshot()
        except InventoryClientError as exc:
            with self._lock:
                # Replan immediately after recovery, even if the remote side
                # returns the same snapshot ID it used before the outage.
                self._last_inventory_snapshot_id = ''
            result = self.inventory_planner.error_result(objective, '', exc)
            self._update_snapshot(
                state='ERROR',
                last_decision=json.dumps(result, ensure_ascii=False, indent=2),
                last_error=str(exc),
                last_evaluation_at=time.time(),
            )
            return result

        now = time.monotonic()
        with self._lock:
            changed = inventory.snapshot_id != self._last_inventory_snapshot_id
            heartbeat_due = (
                now - self._last_evaluation_monotonic >= self.heartbeat_sec
            )
            if not changed and not heartbeat_due:
                return None
            self._last_inventory_snapshot_id = inventory.snapshot_id
            self._last_evaluation_monotonic = now
            revision = self._objective_revision

        self._update_snapshot(state='EVALUATING', last_error='')
        result = self.inventory_planner.plan_snapshot(
            objective,
            inventory,
            list(self.location_loader().keys()),
        )
        with self._lock:
            if revision != self._objective_revision:
                return None
        error = str(result.get('error', ''))
        self._update_snapshot(
            state='ERROR' if result.get('status') == 'error' else 'MONITORING',
            last_decision=json.dumps(result, ensure_ascii=False, indent=2),
            last_error=error,
            last_evaluation_at=time.time(),
        )
        print(
            '[실시간 LLM 재고 판단] '
            f'status={result.get("status")}, moves={len(result.get("moves", []))}'
        )
        return result

    @staticmethod
    def _sync_status(fleet_status):
        telemetry = (fleet_status or {}).get('telemetry') or {}
        value = telemetry.get('inventory_sync') or {}
        return {
            'state': str(value.get('state') or 'OFFLINE').upper(),
            'pending_count': int(value.get('pending_count') or 0),
            'last_error': str(value.get('last_error') or ''),
        }

    @staticmethod
    def _fleet_emergency(fleet_status):
        vehicles = ((fleet_status or {}).get('telemetry') or {}).get(
            'vehicles', {}
        )
        return any(
            bool((vehicle or {}).get('emergency_stopped'))
            for vehicle in vehicles.values()
        )

    @staticmethod
    def _autonomy_scan_blocker(phase, fleet_status):
        """Return why an ARM cache/scan prevents the next cargo move."""
        telemetry = (fleet_status or {}).get('telemetry') or {}
        autonomy = telemetry.get('autonomy') or {}
        arms = telemetry.get('arms') or {}
        arm1 = arms.get('arm1') or {}
        arm1_operation = str(arm1.get('current_operation') or '')
        if phase == 'UNLOADING_INBOUND':
            if (
                autonomy.get('inbound_scan_pending')
                or arm1_operation in {
                    'scan_inbound', 'scan_ship_destinations'
                }
            ):
                return 'ARM1 입항 컨테이너 스캔 완료 대기 중'
            if not autonomy.get('arm1_ship_cache_ready'):
                return 'ARM1 선박 마커 18~23 캐시 완료 대기 중'
            if not autonomy.get('arm2_destination_cache_ready'):
                return 'ARM2 창고 목적지 마커 캐시 완료 대기 중'
        if (
            phase == 'LOADING_OUTBOUND'
            and not autonomy.get('arm1_ship_cache_ready')
        ):
            return 'ARM1 선박 마커 18~23 캐시 완료 대기 중'
        return ''

    def _save_cycle(self):
        self.cycle_store.save(self._cycle)

    def _active_move_entries(self):
        """Return vehicle-scoped executions, migrating the legacy single move."""
        active = getattr(self._cycle, 'active_moves', None)
        if not isinstance(active, dict):
            active = {}
            self._cycle.active_moves = active
        legacy = getattr(self._cycle, 'active_move', None)
        if legacy and legacy not in active.values():
            vehicle_id = str(legacy.get('_vehicle_id') or 'legacy')
            legacy.setdefault(
                '_mission_id',
                str(getattr(self._cycle, 'active_mission_id', '') or ''),
            )
            active[vehicle_id] = legacy
        return list(active.items())

    def _sync_legacy_active_move(self):
        """Keep old diagnostics and persisted state compatible with one move."""
        entries = self._active_move_entries()
        if entries:
            _vehicle_id, move = entries[0]
            self._cycle.active_move = move
            self._cycle.active_mission_id = str(
                move.get('_mission_id') or ''
            )
        else:
            self._cycle.active_move = None
            self._cycle.active_mission_id = ''

    def _remove_active_move(self, vehicle_id):
        active = getattr(self._cycle, 'active_moves', None)
        removed = None
        if isinstance(active, dict):
            removed = active.pop(str(vehicle_id), None)
        if getattr(self._cycle, 'active_move', None) is removed:
            self._cycle.active_move = None
            self._cycle.active_mission_id = ''
        self._sync_legacy_active_move()

    @staticmethod
    def _active_step_resources(move):
        """Map a physical step to the shared ARM/station resources it owns."""
        steps = (move or {}).get('_steps') or []
        index = int((move or {}).get('_step_index') or 0)
        if index >= len(steps):
            return set()
        step = steps[index]
        action_type = str(step.get('type') or '')
        if action_type == 'zone_navigation':
            zone = str(step.get('zone') or '').upper()
            return {'station:B-1'} if zone == 'B-1' else {'station:A'}
        if action_type.startswith('arm1'):
            return {'arm:arm1', 'station:B-1'}
        if action_type.startswith('arm_'):
            return {'arm:arm2', 'station:A'}
        return set()

    def _active_step_resources_available(self, move, fleet_status):
        wanted = self._active_step_resources(move)
        if not wanted:
            return True
        active = getattr(self._cycle, 'active_moves', None)
        active_items = active.items() if isinstance(active, dict) else ()
        for _vehicle_id, other in active_items:
            if other is move or not str(other.get('_current_command_id') or ''):
                continue
            if wanted & self._active_step_resources(other):
                return False
        arms = ((fleet_status.get('telemetry') or {}).get('arms') or {})
        for resource in wanted:
            if not resource.startswith('arm:'):
                continue
            arm = arms.get(resource.split(':', 1)[1]) or {}
            if (
                str(arm.get('current_command_id') or '')
                or str(arm.get('current_operation') or '')
            ):
                return False
        return True

    def _dispatch_step_for_move(self, client, move, mission_id):
        """Dispatch with backward compatibility for legacy single-move tests."""
        if not getattr(self._cycle, 'active_moves', None):
            return self._dispatch_active_step(client)
        return self._dispatch_active_step(
            client, move=move, mission_id=mission_id
        )

    def _evaluate_autonomous(self):
        """Run one observe-plan-first-move cycle and wait for DB truth."""
        client = CentralControlClient(timeout_sec=3.0)
        try:
            fleet_status = client.status()
        except Exception as exc:
            self._update_snapshot(state='WAITING_FOR_DATA', last_error=str(exc))
            return None
        sync = self._sync_status(fleet_status)
        self._update_snapshot(
            db_sync_state=sync['state'],
            db_pending_count=sync['pending_count'],
        )
        if self._fleet_emergency(fleet_status):
            self._update_snapshot(
                state='EMERGENCY_STOPPED', phase='EMERGENCY_STOPPED',
                last_error='차량 비상정지가 활성화되어 신규 계획을 차단함',
            )
            return None
        if sync['state'] != 'READY' or sync['pending_count']:
            self._update_snapshot(
                state='WAITING_FOR_DB_SYNC', phase='WAITING_FOR_DB_SYNC',
                last_error=sync['last_error'] or 'DB 미동기화 이벤트가 존재함',
            )
            return None
        try:
            inventory = self.inventory_planner.inventory_client.fetch_snapshot()
        except InventoryClientError as exc:
            self._update_snapshot(state='ERROR', last_error=str(exc))
            return None
        snapshot = inventory.to_dict()
        self._update_snapshot(db_snapshot_id=inventory.snapshot_id)

        if (
            self._cycle.phase == 'WAITING_OPERATOR'
            and self._cycle.identical_failures >= 3
        ):
            self._update_snapshot(
                state='WAITING_OPERATOR', phase='WAITING_OPERATOR',
                last_error=self._cycle.last_error,
                replan_count=self._cycle.replan_count,
            )
            return None

        for vehicle_id, active_move in list(self._active_move_entries()):
            outcome, detail = self._advance_active_move(
                client,
                fleet_status,
                snapshot,
                move=active_move,
                mission_id=str(active_move.get('_mission_id') or ''),
            )
            if outcome == 'completed':
                self._remove_active_move(vehicle_id)
                self._cycle.identical_failures = 0
                self._cycle.failure_key = ''
                self._cycle.replan_count += 1
                self._save_cycle()
            elif outcome == 'failed':
                self._record_autonomous_failure(
                    detail, move=active_move, vehicle_id=vehicle_id
                )
                if self._cycle.phase == 'WAITING_OPERATOR':
                    return None

        active_entries = self._active_move_entries()
        if active_entries:
            self._sync_legacy_active_move()
            active_payload = [
                {
                    'vehicle_id': vehicle_id,
                    **{
                        key: value for key, value in move.items()
                        if not str(key).startswith('_')
                    },
                    'current_command_id': str(
                        move.get('_current_command_id') or ''
                    ),
                    'current_step': int(move.get('_step_index') or 0) + 1,
                    'total_steps': len(move.get('_steps') or []),
                }
                for vehicle_id, move in active_entries
            ]
            self._update_snapshot(
                state='EXECUTING', phase='EXECUTING_MOVE',
                active_command=', '.join(filter(None, (
                    str(move.get('_current_command_id') or '')
                    for _vehicle_id, move in active_entries
                ))),
                next_move=json.dumps(
                    active_payload, ensure_ascii=False, indent=2
                ),
                execution_steps_json=json.dumps({
                    vehicle_id: json.loads(self._execution_steps_json(move))
                    for vehicle_id, move in active_entries
                }, ensure_ascii=False, indent=2),
                current_step_json=json.dumps({
                    vehicle_id: json.loads(
                        self._current_step_json(move) or '{}'
                    )
                    for vehicle_id, move in active_entries
                }, ensure_ascii=False, indent=2),
            )
            if len(active_entries) >= len(VEHICLE_IDS):
                return None

        port_status = ((fleet_status.get('telemetry') or {}).get(
            'port_status'
        ) or {})
        port_present = bool(port_status.get('vessel_present'))
        reserved_ids = {
            str(move.get('container_id') or '')
            for _vehicle_id, move in active_entries
        }
        reserved_destinations = {
            str(move.get('destination_location') or '')
            for _vehicle_id, move in active_entries
        }
        phase, objective = choose_policy(
            self._cycle,
            snapshot,
            port_present,
            reserved_container_ids=reserved_ids,
            reserved_destinations=reserved_destinations,
        )
        previous_phase = self.snapshot().phase
        self._save_cycle()
        self._update_snapshot(
            state='MONITORING', phase=phase,
            cycle_id=self._cycle.cycle_id,
            replan_count=self._cycle.replan_count,
            active_command='', next_move='', last_error='',
        )
        if phase != previous_phase:
            print(
                f'[자율 항만 관제] cycle={self._cycle.cycle_id}, '
                f'phase={phase}, snapshot={inventory.snapshot_id}'
            )

        if phase == 'WAITING_FOR_CLEAR' and not port_present:
            return self._complete_outbound(client, snapshot)
        if phase in {'WAITING_FOR_INBOUND', 'SCANNING_INBOUND',
                     'WAITING_FOR_CLEAR'}:
            self._park_idle_vehicles(client, fleet_status)
            return None
        if not objective:
            return None
        scan_blocker = self._autonomy_scan_blocker(phase, fleet_status)
        if scan_blocker:
            self._update_snapshot(
                state='WAITING_FOR_ARM_SCAN',
                last_error=scan_blocker,
            )
            return None

        now = time.monotonic()
        planning_signature = json.dumps({
            'snapshot_id': inventory.snapshot_id,
            'phase': phase,
            'vehicles': {
                key: {
                    'state': value.get('state'),
                    'command': value.get('current_command_id'),
                    'zone': value.get('locked_zone'),
                }
                for key, value in (
                    (fleet_status.get('telemetry') or {}).get(
                        'vehicles', {}
                    )
                ).items()
            },
            'arms': {
                key: {
                    'state': value.get('state_text'),
                    'operation': value.get('current_operation'),
                }
                for key, value in (
                    (fleet_status.get('telemetry') or {}).get('arms', {})
                ).items()
            },
        }, ensure_ascii=False, sort_keys=True)
        with self._lock:
            heartbeat_due = (
                now - self._last_evaluation_monotonic >= self.heartbeat_sec
            )
            if (
                planning_signature == self._last_observation
                and not heartbeat_due
            ):
                return None
            self._last_observation = planning_signature
            self._last_evaluation_monotonic = now

        try:
            planning_detections = YoloDetectionClient().get_latest()
        except YoloDetectionError as exc:
            self._update_snapshot(
                state='WAITING_FOR_DATA', last_error=str(exc)
            )
            return None
        compact_planning_detections = compact_detections(
            planning_detections
        )
        if phase == 'UNLOADING_INBOUND':
            # A uses one calibrated fixed Fleet stop and ARM2's cached marker
            # positions for the individual slots.  A zone labels can be
            # occluded even when PostgreSQL proves that a destination slot is
            # available, so visibility must not remove valid DB candidates.
            phase, objective = choose_policy(
                self._cycle,
                snapshot,
                port_present,
                reserved_container_ids=reserved_ids,
                reserved_destinations=reserved_destinations,
            )
        telemetry = fleet_status.get('telemetry') or {}
        operating_context = {
            'cycle_id': self._cycle.cycle_id,
            'phase': phase,
            'port_roi': telemetry.get('port_status'),
            'yolo': compact_planning_detections,
            'vehicles': telemetry.get('vehicles'),
            'arms': telemetry.get('arms'),
            'arm_scan': telemetry.get('autonomy'),
            'inventory_sync': sync,
        }
        llm_objective = (
            f'{objective}\n현재 운영 상태 JSON: '
            f'{json.dumps(operating_context, ensure_ascii=False)}'
        )
        self._update_snapshot(state='EVALUATING', last_decision=llm_objective)
        plan = self.inventory_planner.plan_single_move_snapshot(
            llm_objective, inventory, list(CANONICAL_LOCATIONS)
        )
        if plan.get('status') != 'ready' or not plan.get('moves'):
            error = str(plan.get('error') or plan.get('summary') or '')
            self._update_snapshot(
                state='WAITING_OPERATOR' if plan.get('status') == 'error'
                else 'MONITORING',
                last_decision=json.dumps(plan, ensure_ascii=False, indent=2),
                llm_plan_json=json.dumps(
                    plan, ensure_ascii=False, indent=2
                ),
                last_error=error if plan.get('status') == 'error' else '',
            )
            return plan
        execution = None
        vehicle_id = ''
        try:
            move = validate_first_move(plan['moves'][0], snapshot)
            if str(move.get('container_id') or '') in reserved_ids:
                raise AutonomousPolicyError(
                    f'컨테이너 {move.get("container_id")}는 이미 다른 차량이 작업 중'
                )
            if str(move.get('destination_location') or '') in (
                reserved_destinations
            ):
                raise AutonomousPolicyError(
                    f'목적지 {move.get("destination_location")}는 다른 작업이 예약 중'
                )
            self._validate_policy_move(phase, move, snapshot)
            source_location = str(move.get('source_location') or '')
            preferred_zone = (
                'B-1' if source_location.startswith('선박-')
                else 'A' if source_location.startswith('A-')
                else ''
            )
            vehicle_id = self._ready_vehicle(
                fleet_status,
                preferred_zone=preferred_zone,
                inventory_snapshot=snapshot,
                require_empty_trailer=True,
                excluded_vehicle_ids={
                    key for key, _value in active_entries
                },
            )
            if not vehicle_id:
                self._update_snapshot(
                    state='WAITING_FOR_VEHICLE',
                    last_error='사용 가능한 READY 차량이 없음',
                )
                return plan
            self._autonomy_park_requests.discard(vehicle_id)
            mission_id = f'auto-{uuid.uuid4().hex[:12]}'
            execution = dict(move)
            execution['_steps'] = compile_move(move, vehicle_id)
            execution['_step_index'] = 0
            execution['_vehicle_id'] = vehicle_id
            execution['_current_command_id'] = ''
            execution['_dispatched_at'] = 0.0
            execution['_nav_missing_since'] = 0.0
            execution['_step_retry_counts'] = {}
            execution['_mission_id'] = mission_id
            self._cycle.last_vehicle_id = vehicle_id
            self._cycle.active_moves[vehicle_id] = execution
            self._sync_legacy_active_move()
            self._save_cycle()
            outcome, detail = self._advance_active_move(
                client,
                fleet_status,
                snapshot,
                move=execution,
                mission_id=mission_id,
            )
            if outcome == 'failed':
                raise AutonomousPolicyError(detail)
        except (AutonomousPolicyError, CentralControlApiError,
                VisualNavigationError, YoloDetectionError) as exc:
            if execution is None and active_entries:
                # A rejected second plan must never cancel the independent
                # move that is already running on the other vehicle.
                self._update_snapshot(
                    state='EXECUTING',
                    phase='EXECUTING_MOVE',
                    last_error=str(exc),
                )
            else:
                self._record_autonomous_failure(
                    str(exc),
                    move=execution,
                    vehicle_id=vehicle_id,
                )
            return plan

        if phase == 'LOADING_OUTBOUND':
            cargo_id = str(move['container_id'])
            if cargo_id not in self._cycle.outbound_ids:
                self._cycle.outbound_ids.append(cargo_id)
        self._save_cycle()
        self._update_snapshot(
            state='EXECUTING', phase='EXECUTING_MOVE',
            next_move=json.dumps(move, ensure_ascii=False),
            active_command=str(
                execution.get('_current_command_id', '')
            ),
            last_decision=json.dumps(plan, ensure_ascii=False, indent=2),
            llm_plan_json=json.dumps(plan, ensure_ascii=False, indent=2),
            execution_steps_json=self._execution_steps_json(
                execution
            ),
            current_step_json=self._current_step_json(
                execution
            ),
            last_error='',
        )
        return plan

    @staticmethod
    def _execution_steps_json(move):
        """Render the deterministic physical sequence sent after LLM choice."""
        move = move or {}
        return json.dumps([
            {'sequence': index, **dict(step)}
            for index, step in enumerate(move.get('_steps') or [], 1)
        ], ensure_ascii=False, indent=2)

    @staticmethod
    def _current_step_json(move):
        """Render the current deterministic execution step."""
        move = move or {}
        steps = move.get('_steps') or []
        index = int(move.get('_step_index') or 0)
        if index >= len(steps):
            return ''
        return json.dumps({
            'sequence': index + 1,
            'total': len(steps),
            **dict(steps[index]),
        }, ensure_ascii=False, indent=2)

    def _advance_active_move(
        self, client, fleet_status, snapshot, move=None, mission_id=''
    ):
        move = move if move is not None else (self._cycle.active_move or {})
        mission_id = str(
            mission_id
            or move.get('_mission_id')
            or getattr(self._cycle, 'active_mission_id', '')
            or ''
        )
        cargo = next((
            item for item in snapshot.get('cargos', [])
            if str(item.get('container_id'))
            == str(move.get('container_id'))
        ), None)
        if cargo is None:
            return (
                'failed',
                f'진행 중 컨테이너 {move.get("container_id")}가 '
                '최신 DB에 없어 과거 작업을 폐기함',
            )
        steps = move.get('_steps') or []
        index = int(move.get('_step_index') or 0)
        if index >= len(steps):
            if (
                cargo and str(cargo.get('location'))
                == str(move.get('destination_location'))
            ):
                return 'completed', ''
            return 'failed', '물리 시퀀스 완료 후 DB 최종 위치가 일치하지 않음'
        step = steps[index]
        expected_location = self._arm_step_destination(step)
        if (
            str(step.get('type') or '').startswith('arm')
            and expected_location
            and str(cargo.get('location') or '') == expected_location
        ):
            # A successful movement may reach PostgreSQL before a transient
            # ARM result or process state is observed. Never send the same
            # physical pick/place again when DB already proves this step.
            move['_step_index'] = index + 1
            move['_current_command_id'] = ''
            move['_dispatched_at'] = 0.0
            move['_nav_missing_since'] = 0.0
            self._save_cycle()
            return self._advance_active_move(
                client, fleet_status, snapshot, move, mission_id
            )
        command_id = str(move.get('_current_command_id') or '')
        if not command_id:
            if self._navigation_step_already_reached(
                steps[index], move, fleet_status
            ):
                move['_step_index'] = index + 1
                move['_nav_missing_since'] = 0.0
                self._save_cycle()
                return self._advance_active_move(
                    client, fleet_status, snapshot, move, mission_id
                )
            try:
                self._validate_active_step(
                    steps[index], move, snapshot, fleet_status
                )
                if not self._active_step_resources_available(
                    move, fleet_status
                ):
                    return 'waiting', ''
                self._dispatch_step_for_move(client, move, mission_id)
            except Exception as exc:
                return 'failed', str(exc)
            return 'waiting', ''
        telemetry = fleet_status.get('telemetry') or {}
        recent_arm_results = telemetry.get('arm_results') or {}
        result = (
            recent_arm_results.get(command_id)
            or telemetry.get('last_arm_result')
            or {}
        )
        if step['type'].startswith('arm'):
            if str(result.get('command_id')) != command_id:
                arm = (((fleet_status.get('telemetry') or {}).get(
                    'arms', {}
                )).get(str(step.get('arm_id') or '')) or {})
                if str(arm.get('current_command_id') or '') == command_id:
                    return 'waiting', ''
                expected_location = self._arm_step_destination(step)
                if expected_location and str(cargo.get('location')) == (
                    expected_location
                ):
                    # The physical result reached PostgreSQL before a
                    # dashboard restart lost the transient central result.
                    pass
                else:
                    age = time.time() - float(
                        move.get('_dispatched_at') or 0.0
                    )
                    timeout = getattr(
                        self, 'arm_command_orphan_timeout_sec', 120.0
                    )
                    if age >= timeout:
                        return (
                            'failed',
                            f'중앙관제 큐와 결과에서 사라진 ARM 명령 '
                            f'{command_id}을 {age:.0f}초 대기하여 폐기함',
                        )
                    return 'waiting', ''
            else:
                if result.get('success') is False:
                    return 'failed', str(
                        result.get('message') or 'ARM operation failed'
                    )
                if result.get('success') is not True:
                    return 'waiting', ''
                expected_location = self._arm_step_destination(step)
                if expected_location and str(cargo.get('location')) != (
                    expected_location
                ):
                    return 'waiting', ''
        else:
            vehicle = (((fleet_status.get('telemetry') or {}).get(
                'vehicles', {}
            )).get(move.get('_vehicle_id')) or {})
            if str(vehicle.get('state')).upper() in {'ERROR', 'FAILED'}:
                return 'failed', f'{move.get("_vehicle_id")} 이동 실패'
            age = time.time() - float(move.get('_dispatched_at') or 0.0)
            if age < 0.5 or vehicle.get('state') != READY_STATE:
                move['_nav_missing_since'] = 0.0
                return 'waiting', ''
            locked = str(vehicle.get('locked_zone') or '').upper()
            if step['type'] == 'zone_navigation':
                expected = 'A' if step['zone'].startswith('A-') else step['zone']
                if locked != expected:
                    current_vehicle_command = str(
                        vehicle.get('current_command_id') or ''
                    )
                    if current_vehicle_command:
                        move['_nav_missing_since'] = 0.0
                        return 'waiting', ''
                    missing_since = float(
                        move.get('_nav_missing_since') or 0.0
                    )
                    now = time.time()
                    if not missing_since:
                        move['_nav_missing_since'] = now
                        self._save_cycle()
                        return 'waiting', ''
                    if now - missing_since < getattr(
                        self, 'nav_command_retry_grace_sec', 5.0
                    ):
                        return 'waiting', ''
                    retry_counts = move.setdefault(
                        '_step_retry_counts', {}
                    )
                    retry_key = str(index)
                    retries = int(retry_counts.get(retry_key) or 0)
                    maximum = int(getattr(
                        self, 'nav_command_max_resends', 2
                    ))
                    if retries >= maximum:
                        return (
                            'failed',
                            f'{move.get("_vehicle_id")}가 {step["zone"]}에 '
                            f'도착하지 못해 이동 명령 {maximum}회 재전송 후 중단',
                        )
                    retry_counts[retry_key] = retries + 1
                    move['_current_command_id'] = ''
                    move['_dispatched_at'] = 0.0
                    move['_nav_missing_since'] = 0.0
                    self._save_cycle()
                    try:
                        if not self._active_step_resources_available(
                            move, fleet_status
                        ):
                            return 'waiting', ''
                        self._dispatch_step_for_move(
                            client, move, mission_id
                        )
                    except Exception as exc:
                        return 'failed', str(exc)
                    return 'waiting', ''
            elif step['type'] == 'park_command' and not (
                locked.startswith('PARK') or not locked
            ):
                return 'waiting', ''
            elif step['type'] == 'park_command':
                self._autonomy_park_requests.add(
                    str(move.get('_vehicle_id') or '')
                )
        move['_step_index'] = index + 1
        move['_current_command_id'] = ''
        move['_dispatched_at'] = 0.0
        move['_nav_missing_since'] = 0.0
        self._save_cycle()
        if move['_step_index'] >= len(steps):
            return self._advance_active_move(
                client, fleet_status, snapshot, move, mission_id
            )
        return self._advance_active_move(
            client, fleet_status, snapshot, move, mission_id
        )

    @staticmethod
    def _navigation_step_already_reached(step, move, fleet_status):
        """Accept fresh Fleet arrival state when the zone label is occluded.

        An AGV normally covers the overhead B-1/A label after arriving.  A
        restarted or replanned mission must therefore use the completed Fleet
        lock instead of demanding that YOLO see the covered label again.
        READY plus no active command distinguishes a completed arrival from a
        zone reservation that is still navigating.
        """
        if str(step.get('type') or '') != 'zone_navigation':
            return False
        vehicle_id = str(
            step.get('vehicle_id') or move.get('_vehicle_id') or ''
        )
        vehicle = (((fleet_status.get('telemetry') or {}).get(
            'vehicles', {}
        )).get(vehicle_id) or {})
        if str(vehicle.get('state') or '').upper() != READY_STATE:
            return False
        if str(vehicle.get('current_command_id') or ''):
            return False
        zone = str(step.get('zone') or '').upper()
        expected = 'A' if zone.startswith('A-') else zone
        return str(vehicle.get('locked_zone') or '').upper() == expected

    @staticmethod
    def _validate_active_step(step, move, snapshot, fleet_status):
        """Recheck cargo, stack, cache and emergency immediately per step."""
        action_type = str(step.get('type', ''))
        move.pop('_db_a_navigation_verified', None)
        if action_type == 'zone_navigation':
            zone = str(step.get('zone') or '').upper()
            if zone == 'A' or zone.startswith('A-'):
                destination = str(move.get('destination_location') or '')
                source = str(move.get('source_location') or '')
                location = destination if destination.startswith('A-') else source
                if not location.startswith(zone):
                    raise AutonomousPolicyError(
                        f'A구역 이동 대상 불일치: zone={zone}, location={location}'
                    )
                if location == destination:
                    stack = [
                        item for item in snapshot.get('cargos', [])
                        if str(item.get('location') or '') == destination
                        and str(item.get('container_id') or '')
                        != str(move.get('container_id') or '')
                    ]
                    expected_floor = len(stack) + 1
                    requested_floor = int(
                        move.get('destination_floor') or expected_floor
                    )
                    if len(stack) >= 3 or requested_floor != expected_floor:
                        raise AutonomousPolicyError(
                            f'DB 목적지 {destination}가 현재 적재 가능한 상태가 아님'
                        )
                move['_db_a_navigation_verified'] = True
            return
        if not action_type.startswith('arm'):
            return
        if RealtimeLLMAgent._fleet_emergency(fleet_status):
            raise AutonomousPolicyError('비상정지 중 ARM 단계를 시작할 수 없음')
        container_id = str(move.get('container_id') or '')
        cargo = next((
            item for item in snapshot.get('cargos', [])
            if str(item.get('container_id')) == container_id
        ), None)
        if cargo is None:
            raise AutonomousPolicyError(f'컨테이너 {container_id} DB 상태 없음')
        location = str(cargo.get('location') or '')
        expected_source = RealtimeLLMAgent._arm_step_source(step, move)
        if expected_source and location != expected_source:
            raise AutonomousPolicyError(
                f'ARM source mismatch for container {container_id}: '
                f'expected {expected_source}, DB={location}'
            )
        stack = [
            item for item in snapshot.get('cargos', [])
            if str(item.get('location')) == location
        ]
        if location.startswith(('A-', '선박-')) and int(
            cargo.get('floor') or 1
        ) != max(int(item.get('floor') or 1) for item in stack):
            raise AutonomousPolicyError(
                f'컨테이너 {container_id} 위에 방해 컨테이너가 생김'
            )
        expected_destination = RealtimeLLMAgent._arm_step_destination(step)
        if expected_destination in {'AMR1', 'AMR2'}:
            occupants = [
                str(item.get('container_id') or '')
                for item in snapshot.get('cargos', [])
                if str(item.get('location') or '') == expected_destination
                and str(item.get('container_id') or '') != container_id
            ]
            if occupants:
                raise AutonomousPolicyError(
                    f'{expected_destination} already carries DB cargo: '
                    f'{", ".join(occupants)}'
                )
        source_id = step.get('source_id', -1)
        try:
            source_id = int(source_id)
        except (TypeError, ValueError):
            source_id = -1
        if 0 <= source_id <= 8 and str(source_id) != container_id:
            raise AutonomousPolicyError(
                f'ARM source_id={source_id} does not match DB container '
                f'{container_id}'
            )
        if expected_destination.startswith(('A-', '선박-')):
            destination_stack = [
                item for item in snapshot.get('cargos', [])
                if str(item.get('location')) == expected_destination
                and str(item.get('container_id')) != container_id
            ]
            if len(destination_stack) >= 3:
                raise AutonomousPolicyError(
                    f'목적지 {expected_destination} 적재 용량 초과'
                )
        if (
            step.get('type') == 'arm1_pick_place'
            and 18 <= int(step.get('destination_id', -1)) <= 23
        ):
            status = ((fleet_status.get('telemetry') or {}).get(
                'autonomy'
            ) or {})
            if not status.get('arm1_ship_cache_ready'):
                raise AutonomousPolicyError('ARM1 선박 마커 캐시가 준비되지 않음')

    @staticmethod
    def _arm_step_destination(step):
        action_type = step.get('type')
        if action_type == 'arm_transfer_to_slot':
            return str(step.get('destination_slot') or '')
        if action_type == 'arm_load_to_trailer':
            return {'agv1': 'AMR1', 'agv2': 'AMR2'}.get(
                str(step.get('vehicle_id') or ''), ''
            )
        if action_type == 'arm1_pick_place':
            destination_id = int(step.get('destination_id', -1))
            if destination_id == 10:
                return 'AMR1'
            if destination_id == 9:
                return 'AMR2'
            if 18 <= destination_id <= 23:
                return f'선박-{destination_id - 17}'
        if action_type == 'arm_transfer_by_id':
            locations = {
                11: 'A-1-1', 12: 'A-1-2', 13: 'A-2-1',
                14: 'A-2-2', 15: 'A-3-1', 16: 'A-3-2',
            }
            return locations.get(int(step.get('destination_id', -1)), '')
        return ''

    @staticmethod
    def _arm_step_source(step, move):
        """Return the DB location physically reachable by this ARM step."""
        action_type = str(step.get('type') or '')
        vehicle_id = str(step.get('vehicle_id') or move.get('_vehicle_id') or '')
        trailer_location = {
            'agv1': 'AMR1',
            'agv2': 'AMR2',
        }.get(vehicle_id, '')
        if action_type == 'arm_transfer_to_slot':
            return trailer_location
        if action_type in {'arm_load_to_trailer', 'arm_transfer_by_id'}:
            return str(move.get('source_location') or '')
        if action_type == 'arm1_pick_place':
            try:
                source_id = int(step.get('source_id', -1))
            except (TypeError, ValueError):
                source_id = -1
            if source_id == 10:
                return 'AMR1'
            if source_id == 9:
                return 'AMR2'
            return str(move.get('source_location') or '')
        return ''

    def _record_autonomous_failure(self, reason, move=None, vehicle_id=''):
        move = move if move is not None else (self._cycle.active_move or {})
        key = json.dumps({
            'container_id': move.get('container_id'),
            'source': move.get('source_location'),
            'destination': move.get('destination_location'),
            'reason': str(reason),
        }, ensure_ascii=False, sort_keys=True)
        terminal_failure = (
            self._terminal_navigation_failure(reason)
            or self._terminal_arm_failure(reason, move)
        )
        if terminal_failure:
            self._cycle.failure_key = key
            self._cycle.identical_failures = 3
        elif key == self._cycle.failure_key:
            self._cycle.identical_failures += 1
        else:
            self._cycle.failure_key = key
            self._cycle.identical_failures = 1
        active = getattr(self._cycle, 'active_moves', None)
        if isinstance(active, dict) and active:
            resolved_vehicle = str(
                vehicle_id or move.get('_vehicle_id') or ''
            )
            if resolved_vehicle:
                removed = active.pop(resolved_vehicle, None)
            else:
                removed = None
                for key, value in list(active.items()):
                    if value is move:
                        removed = active.pop(key, None)
                        break
            if getattr(self._cycle, 'active_move', None) is removed:
                self._cycle.active_move = None
                self._cycle.active_mission_id = ''
            self._sync_legacy_active_move()
        else:
            self._cycle.active_move = None
            self._cycle.active_mission_id = ''
        self._cycle.replan_count += 1
        self._cycle.last_error = str(reason)
        waiting = self._cycle.identical_failures >= 3
        if waiting:
            self._cycle.phase = 'WAITING_OPERATOR'
        self._save_cycle()
        self._update_snapshot(
            state='WAITING_OPERATOR' if waiting else 'MONITORING',
            phase=self._cycle.phase,
            replan_count=self._cycle.replan_count,
            last_error=str(reason),
        )

    @staticmethod
    def _terminal_navigation_failure(reason):
        """Stop replanning after one complete two-sided physical recovery."""
        text = str(reason or '').lower()
        return any(marker in text for marker in (
            'adaptive recovery failed',
            'adaptive lidar recovery',
            '이동 명령 0회 재전송 후 중단',
            'vehicle did not move after 0 re-send',
        ))

    @staticmethod
    def _terminal_arm_failure(reason, move=None):
        """Latch autonomous dispatch after a physical ARM step fails.

        Replanning an ARM pick/place failure as a fresh move can select a
        different container and vehicle.  That is unsafe: the failed vehicle
        may still occupy the shared A/B-1 station and the physical cargo state
        is no longer proven.  Require an operator reassessment/restart before
        admitting another move.  Data/cache readiness waits are handled before
        dispatch and do not reach this failure path.
        """
        move = move or {}
        steps = move.get('_steps') or []
        try:
            index = int(move.get('_step_index') or 0)
        except (TypeError, ValueError):
            index = 0
        current_type = ''
        if 0 <= index < len(steps):
            current_type = str((steps[index] or {}).get('type') or '')
        if current_type.startswith('arm'):
            return True
        text = str(reason or '').lower()
        return any(marker in text for marker in (
            'missing aruco',
            'all station poses exhausted',
            'arm operation failed',
            'arm 작업 실패',
            'pick failed',
            'place failed',
        ))

    @staticmethod
    def _validate_policy_move(phase, move, snapshot):
        source = str(move['source_location'])
        destination = str(move['destination_location'])
        if phase == 'UNLOADING_INBOUND' and not (
            source in SHIP_LOCATIONS and destination.startswith('A-')
        ):
            raise AutonomousPolicyError('입항 단계는 선박→창고 이동만 허용됨')
        if phase == 'LOADING_OUTBOUND':
            cargo = next(
                item for item in snapshot['cargos']
                if str(item.get('container_id')) == str(move['container_id'])
            )
            if not (
                source.startswith('A-') and destination in SHIP_LOCATIONS
                and int(cargo.get('floor', 1)) >= 3
            ):
                raise AutonomousPolicyError(
                    '출항 단계는 창고 3층 최상단→선박 이동만 허용됨'
                )

    def _ready_vehicle(
        self,
        fleet_status,
        preferred_zone='',
        inventory_snapshot=None,
        require_empty_trailer=False,
        excluded_vehicle_ids=None,
    ):
        """Choose a READY vehicle without replacing a station's owner.

        After an ARM failure the vehicle and trailer can remain physically at
        B-1 or A.  Round-robin selection must not send the other vehicle into
        that occupied work station.  Reuse its current owner when READY, or
        wait for that owner instead of assigning a replacement vehicle.
        """
        vehicles = ((fleet_status.get('telemetry') or {}).get(
            'vehicles', {}
        ))
        occupied_trailers = set()
        if require_empty_trailer and isinstance(inventory_snapshot, dict):
            occupied_trailers = {
                str(item.get('location') or '')
                for item in inventory_snapshot.get('cargos', [])
                if str(item.get('location') or '') in {'AMR1', 'AMR2'}
            }
        excluded = {
            str(value) for value in (excluded_vehicle_ids or [])
        }
        ready = []
        for vehicle_id in VEHICLE_IDS:
            vehicle = vehicles.get(vehicle_id) or {}
            trailer_location = (
                'AMR1' if vehicle_id == 'agv1' else 'AMR2'
            )
            if (
                vehicle.get('state') == READY_STATE
                and not vehicle.get('emergency_stopped')
                and not vehicle.get('current_command_id')
                and trailer_location not in occupied_trailers
                and vehicle_id not in excluded
            ):
                ready.append(vehicle_id)
        if not ready:
            return ''
        normalized_preferred = str(preferred_zone or '').upper()
        if normalized_preferred:
            owners = []
            for vehicle_id in VEHICLE_IDS:
                locked_zone = str(
                    (vehicles.get(vehicle_id) or {}).get('locked_zone') or ''
                ).upper()
                normalized_locked = (
                    'A' if locked_zone.startswith('A') else locked_zone
                )
                if normalized_locked == normalized_preferred:
                    owners.append(vehicle_id)
            if owners:
                owner = owners[0]
                return owner if owner in ready else ''
        last_vehicle = str(
            getattr(self._cycle, 'last_vehicle_id', '') or ''
        )
        if last_vehicle in VEHICLE_IDS:
            start = (VEHICLE_IDS.index(last_vehicle) + 1) % len(VEHICLE_IDS)
            preference = VEHICLE_IDS[start:] + VEHICLE_IDS[:start]
        else:
            preference = VEHICLE_IDS
        return next(
            vehicle_id for vehicle_id in preference if vehicle_id in ready
        )

    def _park_idle_vehicles(self, client, fleet_status):
        vehicles = ((fleet_status.get('telemetry') or {}).get(
            'vehicles', {}
        ))
        for vehicle_id in VEHICLE_IDS:
            vehicle = vehicles.get(vehicle_id) or {}
            locked_zone = str(vehicle.get('locked_zone') or '').upper()
            if locked_zone.startswith('PARK'):
                self._autonomy_park_requests.add(vehicle_id)
                continue
            if vehicle_id in self._autonomy_park_requests:
                continue
            if (
                vehicle.get('state') == READY_STATE
                and not vehicle.get('current_command_id')
                and not vehicle.get('locked_zone')
            ):
                try:
                    client.send_park(vehicle_id=vehicle_id)
                except CentralControlApiError:
                    pass
                else:
                    self._autonomy_park_requests.add(vehicle_id)

    def _zone_goal(
        self, zone, summary, width, height, allow_registered_a=False
    ):
        compact = compact_detections(summary)
        normalized = str(zone).upper()
        selected = next((
            item for item in compact
            if (
                str(item.get('label', '')).upper() == normalized
                or (
                    normalized == 'A'
                    and str(item.get('label', '')).upper() in {
                        'A-1', 'A-2', 'A-3'
                    }
                )
            )
        ), None)
        if selected is None:
            if (
                (normalized == 'A' or normalized.startswith('A-'))
                and allow_registered_a
            ):
                loader = getattr(
                    self, 'location_loader', _load_registered_locations
                )
                locations = loader() or {}
                registered = (
                    locations.get('창고 A')
                    or locations.get(normalized)
                    or locations.get('A')
                    or {}
                )
                pixel = registered.get('cctv_pixel') or []
                if len(pixel) == 2:
                    x = min(max(float(pixel[0]), 0.0), float(width - 1))
                    y = min(max(float(pixel[1]), 50.0), float(height - 1))
                    return (
                        {'x': x, 'y': y},
                        {'x': x, 'y': y - 50.0},
                        'parking_a',
                    )
            raise AutonomousPolicyError(f'YOLO에서 작업 구역 {zone}를 찾지 못함')
        action = {
            'detection_index': selected['detection_index'],
            'approach_side': 'bottom',
        }
        target, heading, detected = resolve_detection_approach(
            action, summary, width, height
        )
        return target, heading, zone_mode_for_label(detected['label'])

    def _dispatch_active_step(self, client, move=None, mission_id=''):
        move = move if move is not None else self._cycle.active_move
        mission_id = str(
            mission_id
            or move.get('_mission_id')
            or getattr(self._cycle, 'active_mission_id', '')
            or ''
        )
        steps = move.get('_steps') or []
        index = int(move.get('_step_index') or 0)
        if index >= len(steps):
            return ''
        action = steps[index]
        action_type = action['type']
        vehicle_id = str(move.get('_vehicle_id') or '')
        command_payload = {
            'mission_id': mission_id,
            'sequence': index + 1,
            'total_steps': len(steps),
            'action': dict(action),
            'retry_attempt': int((
                move.get('_step_retry_counts') or {}
            ).get(str(index), 0)),
        }
        if action_type == 'zone_navigation':
            summary = YoloDetectionClient().get_latest()
            width = int(summary.get('image_width') or 640)
            height = int(summary.get('image_height') or 480)
            target, heading, mode = self._zone_goal(
                action['zone'], summary, width, height,
                allow_registered_a=bool(
                    move.get('_db_a_navigation_verified')
                ),
            )
            response = client.send_pixel_goal(
                target, heading, vehicle_id=vehicle_id, mode=mode,
                zone_visually_empty=True,
            )
            command_payload['request'] = {
                'vehicle_id': vehicle_id,
                'zone': action['zone'],
                'mode': mode,
                'target': target,
                'heading': heading,
                'zone_visually_empty': True,
            }
        elif action_type == 'park_command':
            response = client.send_park(vehicle_id=vehicle_id)
            command_payload['request'] = {'vehicle_id': vehicle_id}
        else:
            operation = {
                'arm1_pick_place': 'pick_place',
                'arm_transfer_to_slot': 'transfer_to_slot',
                'arm_load_to_trailer': 'load_to_trailer',
                'arm_transfer_by_id': 'transfer_by_id',
            }[action_type]
            response = client.send_arm_command(
                operation=operation,
                arm_id=action['arm_id'],
                mission_id=mission_id,
                destination_slot=action.get('destination_slot', ''),
                source_id=action.get('source_id', -1),
                destination_id=action.get('destination_id', -1),
                vehicle_id=action.get('vehicle_id', ''),
                container_id=str(move.get('container_id') or ''),
                final_for_vehicle=action.get('final_for_vehicle', False),
            )
            command_payload['request'] = {
                'operation': operation,
                'arm_id': action['arm_id'],
                'mission_id': mission_id,
                'destination_slot': action.get('destination_slot', ''),
                'source_id': action.get('source_id', -1),
                'destination_id': action.get('destination_id', -1),
                'vehicle_id': action.get('vehicle_id', ''),
                'container_id': str(move.get('container_id') or ''),
                'final_for_vehicle': action.get(
                    'final_for_vehicle', False
                ),
            }
        move['_current_command_id'] = str(response.get('command_id') or '')
        if not move['_current_command_id']:
            raise AutonomousPolicyError('중앙관제가 command_id를 반환하지 않음')
        move['_dispatched_at'] = time.time()
        command_payload['command_id'] = move['_current_command_id']
        command_payload['response'] = response
        self._save_cycle()
        self._update_snapshot(
            active_command=move['_current_command_id'],
            execution_steps_json=self._execution_steps_json(move),
            current_step_json=self._current_step_json(move),
            command_payload_json=json.dumps(
                command_payload, ensure_ascii=False, indent=2
            ),
        )
        print(
            f'[자율 항만 실행] mission={self._cycle.active_mission_id}, '
            f'step={index + 1}/{len(steps)}, type={action_type}, '
            f'command={move["_current_command_id"]}'
        )
        return move['_current_command_id']

    def _complete_outbound(self, client, snapshot):
        by_id = {
            str(item.get('container_id')): item
            for item in snapshot.get('cargos', [])
        }
        submitted = 0
        for container_id in list(self._cycle.outbound_ids):
            cargo = by_id.get(container_id)
            if not cargo or cargo.get('location') == '출항완료':
                continue
            client.send_inventory_movement({
                'schema_version': '1.0',
                'operation_id': (
                    f'{self._cycle.cycle_id}-clear-{container_id}'
                ),
                'command_id': '', 'mission_id': self._cycle.cycle_id,
                'arm_id': 'operator', 'container_id': container_id,
                'source_location': str(cargo.get('location') or ''),
                'source_floor': int(cargo.get('floor') or 1),
                'source_base_aruco_id': str(cargo.get('base_aruco_id') or ''),
                'destination_location': '출항완료',
                'destination_floor': 1,
                'destination_base_aruco_id': '',
                'success': True,
                'error': '',
            })
            submitted += 1
        if not submitted:
            self._cycle = type(self._cycle)()
            self._save_cycle()
        return submitted

    def _evaluate(self, objective):
        shared_frame = _cctv_monitor_view().SHARED_FRAME
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
        cargo_registry, cargo_details = _load_cargo_context()
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
                list(self.location_loader().keys()),
                known_types,
                image_jpeg=encoded.tobytes(),
                image_width=width,
                image_height=height,
                yolo_detections=compact,
                normalization_command=objective,
                zone_status=str(
                    (fleet_status.get('telemetry') or {}).get(
                        'b1_zone', ''
                    )
                ),
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
