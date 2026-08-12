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
    CANONICAL_LOCATIONS,
    CycleStore,
    SHIP_LOCATIONS,
    AutonomousPolicyError,
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
        self._cycle = self.cycle_store.load()
        # A parked vehicle normally reports READY with no zone lock, which is
        # indistinguishable from an idle unparked vehicle in fleet telemetry.
        # Remember successful/requested parking locally so the waiting loop
        # cannot enqueue a new park command on every heartbeat.
        self._autonomy_park_requests = set()

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
        """Start the fixed autonomous port policy after operator approval."""
        if self._cycle.phase == 'WAITING_OPERATOR':
            self._cycle.identical_failures = 0
            self._cycle.failure_key = ''
            self._cycle.last_error = ''
            self._cycle.phase = 'WAITING_FOR_INBOUND'
            self._save_cycle()
        with self._lock:
            self._objective_revision += 1
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
                last_error='',
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

        if self._cycle.active_move:
            outcome, detail = self._advance_active_move(
                client, fleet_status, snapshot
            )
            if outcome == 'completed':
                self._cycle.active_move = None
                self._cycle.active_mission_id = ''
                self._cycle.identical_failures = 0
                self._cycle.failure_key = ''
                self._cycle.replan_count += 1
                self._save_cycle()
            elif outcome == 'failed':
                self._record_autonomous_failure(detail)
                return None
            else:
                self._update_snapshot(
                    state='EXECUTING', phase='EXECUTING_MOVE',
                    active_command=str(
                        self._cycle.active_move.get(
                            '_current_command_id', ''
                        )
                    ),
                    next_move=json.dumps(
                        {
                            key: value
                            for key, value in self._cycle.active_move.items()
                            if not str(key).startswith('_')
                        },
                        ensure_ascii=False,
                    ),
                )
                return None

        port_status = ((fleet_status.get('telemetry') or {}).get(
            'port_status'
        ) or {})
        port_present = bool(port_status.get('vessel_present'))
        phase, objective = choose_policy(
            self._cycle, snapshot, port_present
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
            visible_warehouse_zones = {
                str(item.get('label') or '').upper()
                for item in compact_planning_detections
                if str(item.get('label') or '').upper() in {
                    'A-1', 'A-2', 'A-3'
                }
            }
            if not visible_warehouse_zones:
                self._update_snapshot(
                    state='WAITING_FOR_DATA',
                    last_error='YOLO에서 실행 가능한 창고 구역을 찾지 못함',
                )
                return None
            phase, objective = choose_policy(
                self._cycle,
                snapshot,
                port_present,
                visible_warehouse_zones=visible_warehouse_zones,
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
                last_error=error if plan.get('status') == 'error' else '',
            )
            return plan
        try:
            move = validate_first_move(plan['moves'][0], snapshot)
            self._validate_policy_move(phase, move, snapshot)
            vehicle_id = self._ready_vehicle(fleet_status)
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
            self._cycle.active_move = execution
            self._cycle.active_mission_id = mission_id
            self._save_cycle()
            self._dispatch_active_step(client)
        except (AutonomousPolicyError, CentralControlApiError,
                VisualNavigationError, YoloDetectionError) as exc:
            self._record_autonomous_failure(str(exc))
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
                self._cycle.active_move.get('_current_command_id', '')
            ),
            last_decision=json.dumps(plan, ensure_ascii=False, indent=2),
            last_error='',
        )
        return plan

    def _advance_active_move(self, client, fleet_status, snapshot):
        move = self._cycle.active_move or {}
        steps = move.get('_steps') or []
        index = int(move.get('_step_index') or 0)
        if index >= len(steps):
            cargo = next((
                item for item in snapshot.get('cargos', [])
                if str(item.get('container_id'))
                == str(move.get('container_id'))
            ), None)
            if (
                cargo and str(cargo.get('location'))
                == str(move.get('destination_location'))
            ):
                return 'completed', ''
            return 'failed', '물리 시퀀스 완료 후 DB 최종 위치가 일치하지 않음'
        command_id = str(move.get('_current_command_id') or '')
        if not command_id:
            try:
                self._validate_active_step(
                    steps[index], move, snapshot, fleet_status
                )
                self._dispatch_active_step(client)
            except Exception as exc:
                return 'failed', str(exc)
            return 'waiting', ''
        step = steps[index]
        result = ((fleet_status.get('telemetry') or {}).get(
            'last_arm_result'
        ) or {})
        if step['type'].startswith('arm'):
            if str(result.get('command_id')) != command_id:
                return 'waiting', ''
            if result.get('success') is False:
                return 'failed', str(
                    result.get('message') or 'ARM operation failed'
                )
            if result.get('success') is not True:
                return 'waiting', ''
            expected_location = self._arm_step_destination(step)
            if expected_location:
                cargo = next((
                    item for item in snapshot.get('cargos', [])
                    if str(item.get('container_id'))
                    == str(move.get('container_id'))
                ), None)
                if (
                    cargo is None
                    or str(cargo.get('location')) != expected_location
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
                return 'waiting', ''
            locked = str(vehicle.get('locked_zone') or '').upper()
            if step['type'] == 'zone_navigation':
                expected = 'A' if step['zone'].startswith('A-') else step['zone']
                if locked != expected:
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
        self._save_cycle()
        if move['_step_index'] >= len(steps):
            return self._advance_active_move(client, fleet_status, snapshot)
        try:
            self._validate_active_step(
                steps[move['_step_index']], move, snapshot, fleet_status
            )
            self._dispatch_active_step(client)
        except Exception as exc:
            return 'failed', str(exc)
        return 'waiting', ''

    @staticmethod
    def _validate_active_step(step, move, snapshot, fleet_status):
        """Recheck cargo, stack, cache and emergency immediately per step."""
        if not str(step.get('type', '')).startswith('arm'):
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

    def _record_autonomous_failure(self, reason):
        move = self._cycle.active_move or {}
        key = json.dumps({
            'container_id': move.get('container_id'),
            'source': move.get('source_location'),
            'destination': move.get('destination_location'),
            'reason': str(reason),
        }, ensure_ascii=False, sort_keys=True)
        if key == self._cycle.failure_key:
            self._cycle.identical_failures += 1
        else:
            self._cycle.failure_key = key
            self._cycle.identical_failures = 1
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

    @staticmethod
    def _ready_vehicle(fleet_status):
        vehicles = ((fleet_status.get('telemetry') or {}).get(
            'vehicles', {}
        ))
        for vehicle_id in VEHICLE_IDS:
            vehicle = vehicles.get(vehicle_id) or {}
            if (
                vehicle.get('state') == READY_STATE
                and not vehicle.get('emergency_stopped')
                and not vehicle.get('current_command_id')
            ):
                return vehicle_id
        return ''

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

    def _zone_goal(self, zone, summary, width, height):
        compact = compact_detections(summary)
        normalized = str(zone).upper()
        selected = next((
            item for item in compact
            if str(item.get('label', '')).upper() == normalized
        ), None)
        if selected is None:
            raise AutonomousPolicyError(f'YOLO에서 작업 구역 {zone}를 찾지 못함')
        action = {
            'detection_index': selected['detection_index'],
            'approach_side': 'bottom',
        }
        target, heading, detected = resolve_detection_approach(
            action, summary, width, height
        )
        return target, heading, zone_mode_for_label(detected['label'])

    def _dispatch_active_step(self, client):
        move = self._cycle.active_move
        steps = move.get('_steps') or []
        index = int(move.get('_step_index') or 0)
        if index >= len(steps):
            return ''
        action = steps[index]
        action_type = action['type']
        vehicle_id = str(move.get('_vehicle_id') or '')
        if action_type == 'zone_navigation':
            summary = YoloDetectionClient().get_latest()
            width = int(summary.get('image_width') or 640)
            height = int(summary.get('image_height') or 480)
            target, heading, mode = self._zone_goal(
                action['zone'], summary, width, height
            )
            response = client.send_pixel_goal(
                target, heading, vehicle_id=vehicle_id, mode=mode,
                zone_visually_empty=True,
            )
        elif action_type == 'park_command':
            response = client.send_park(vehicle_id=vehicle_id)
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
                mission_id=self._cycle.active_mission_id,
                destination_slot=action.get('destination_slot', ''),
                source_id=action.get('source_id', -1),
                destination_id=action.get('destination_id', -1),
                vehicle_id=action.get('vehicle_id', ''),
                container_id=str(move.get('container_id') or ''),
                final_for_vehicle=action.get('final_for_vehicle', False),
            )
        move['_current_command_id'] = str(response.get('command_id') or '')
        if not move['_current_command_id']:
            raise AutonomousPolicyError('중앙관제가 command_id를 반환하지 않음')
        move['_dispatched_at'] = time.time()
        self._save_cycle()
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
