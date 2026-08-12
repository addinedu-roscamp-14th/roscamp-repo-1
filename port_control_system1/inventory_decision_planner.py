"""Generate read-only container relocation plans from remote inventory state."""

from __future__ import annotations

import json
import os
import uuid
from dataclasses import replace

from inventory_client import (
    InventoryClient,
    InventoryClientError,
    InventorySnapshot,
)


PLAN_SCHEMA_VERSION = '1.0'
PLAN_STATUSES = {'ready', 'no_action', 'error'}
MOVE_FIELDS = {
    'sequence',
    'container_id',
    'container_name',
    'source_location',
    'source_floor',
    'destination_location',
    'destination_floor',
    'destination_base_aruco_id',
    'reason',
}


_PLANNER_SYSTEM_PROMPT = """당신은 항만 컨테이너 재배치 계획기입니다.
주어진 운영 목표와 인벤토리 스냅샷만 근거로 목표 완료까지 필요한 이동 전체를
순서대로 작성하세요. DB에 없는 컨테이너나 위치를 만들지 마세요. 아래층 컨테이너를
옮겨야 하면 그 컨테이너를 기반으로 적재된 방해 컨테이너를 먼저 임시 위치로
이동하세요. 입력에 없는 중량, 용량, 위험물 제약은 추측하지 마세요.

반드시 JSON 객체만 반환하고 다음 형식을 지키세요.
{
  "status": "ready|no_action",
  "moves": [{
    "sequence": 1,
    "container_id": "원격 DB의 container_id",
    "container_name": "원격 DB의 name",
    "source_location": "이 단계 직전 위치",
    "source_floor": 1,
    "destination_location": "등록된 목적지",
    "destination_floor": 1,
    "destination_base_aruco_id": "바닥이면 빈 문자열 또는 고정 마커 ID",
    "reason": "이 이동이 필요한 이유"
  }],
  "summary": "전체 계획 요약"
}
이동이 필요 없으면 status=no_action, moves=[]로 반환하세요. sequence는 1부터 끊김 없이
증가해야 합니다. 같은 컨테이너을 임시로 치운 뒤 복원하는 것은 허용하지만 동일한
이동을 반복해서는 안 됩니다. 실제 로봇 명령, ROS 서비스명, DB 갱신 명령은 만들지
마세요."""


class InventoryPlanValidationError(ValueError):
    """Raised when an LLM plan is inconsistent with its source snapshot."""


def _error_result(objective, snapshot_id, message):
    return {
        'schema_version': PLAN_SCHEMA_VERSION,
        'plan_id': f'plan-{uuid.uuid4()}',
        'snapshot_id': str(snapshot_id or ''),
        'objective': str(objective or '').strip(),
        'status': 'error',
        'moves': [],
        'summary': '',
        'error': str(message),
    }


class InventoryDecisionPlanner:
    """Fetch inventory, ask an LLM for a plan, then validate it locally."""

    def __init__(
        self,
        inventory_client=None,
        model=None,
        host=None,
        timeout_sec=90.0,
        llm=None,
    ):
        self.inventory_client = inventory_client or InventoryClient()
        self.model = model or os.environ.get('LOCAL_LLM_MODEL', 'gemma4:31b')
        self.host = host or os.environ.get('OLLAMA_HOST', 'http://agent.sds.codes')
        self.timeout_sec = float(timeout_sec)
        self._llm = llm

    @staticmethod
    def error_result(objective, snapshot_id, message):
        """Build the public fail-closed result shape."""
        return _error_result(objective, snapshot_id, message)

    def plan(self, objective, known_locations):
        """Fetch a fresh snapshot and return a validated plan or error JSON."""
        try:
            snapshot = self.inventory_client.fetch_snapshot()
        except InventoryClientError as exc:
            return _error_result(objective, '', exc)
        return self.plan_snapshot(objective, snapshot, known_locations)

    def plan_snapshot(self, objective, snapshot, known_locations):
        """Plan against an already fetched immutable snapshot."""
        objective = str(objective or '').strip()
        if not objective:
            return _error_result(objective, snapshot.snapshot_id, 'objective is empty')
        locations = sorted({
            str(location).strip()
            for location in (known_locations or [])
            if str(location).strip()
        })
        if not locations:
            return _error_result(
                objective,
                snapshot.snapshot_id,
                'no registered destination locations are available',
            )
        prompt = self._build_prompt(objective, snapshot, locations)
        try:
            raw_plan = self._generate(prompt)
            return self.validate_plan(
                raw_plan,
                objective=objective,
                snapshot=snapshot,
                known_locations=locations,
            )
        except Exception as exc:
            return _error_result(objective, snapshot.snapshot_id, exc)

    def plan_single_move_snapshot(
        self, objective, snapshot, known_locations, attempts=3
    ):
        """Generate exactly one move, repairing one invalid LLM response."""
        objective = str(objective or '').strip()
        locations = sorted({
            str(location).strip()
            for location in (known_locations or [])
            if str(location).strip()
        })
        if not objective or not locations:
            return self.plan_snapshot(objective, snapshot, locations)
        prompt = self._build_prompt(objective, snapshot, locations)
        last_error = None
        invalid_output = None
        for _attempt in range(max(1, int(attempts))):
            repair_context = {
                'planning_mode': 'single_move',
                'hard_constraint': (
                    'status=ready이면 moves 배열에 정확히 1건만 반환한다. '
                    '목적지 floor와 base_aruco_id는 목표에 제공된 후보 값을 '
                    '그대로 사용한다.'
                ),
                'request': json.loads(prompt),
            }
            if last_error is not None:
                repair_context['previous_validation_error'] = str(last_error)
                repair_context['previous_invalid_output'] = invalid_output
            try:
                raw_plan = self._generate(json.dumps(
                    repair_context, ensure_ascii=False, separators=(',', ':')
                ))
                invalid_output = raw_plan
                status = str(raw_plan.get('status', '')).strip()
                if status != 'ready':
                    raise InventoryPlanValidationError(
                        'autonomous single-move planning requires status=ready'
                    )
                if len(raw_plan.get('moves') or []) != 1:
                    raise InventoryPlanValidationError(
                        'single-move planning requires exactly one move'
                    )
                return self.validate_plan(
                    raw_plan,
                    objective=objective,
                    snapshot=snapshot,
                    known_locations=locations,
                )
            except Exception as exc:
                last_error = exc
        return _error_result(objective, snapshot.snapshot_id, last_error)

    @staticmethod
    def _build_prompt(objective, snapshot, known_locations):
        context = {
            'objective': objective,
            'registered_locations': known_locations,
            'inventory_snapshot': snapshot.to_dict(),
        }
        return json.dumps(context, ensure_ascii=False, separators=(',', ':'))

    def _generate(self, prompt):
        if self._llm is not None:
            result = self._llm(prompt)
        else:
            try:
                import ollama
            except ImportError as exc:
                raise RuntimeError(f'ollama package is unavailable: {exc}') from exc
            client = ollama.Client(host=self.host, timeout=self.timeout_sec)
            kwargs = {
                'model': self.model,
                'messages': [
                    {'role': 'system', 'content': _PLANNER_SYSTEM_PROMPT},
                    {'role': 'user', 'content': prompt},
                ],
                'format': 'json',
                'options': {'temperature': 0},
            }
            try:
                result = client.chat(think=False, **kwargs)
            except TypeError:
                result = client.chat(**kwargs)
            except Exception as exc:
                raise RuntimeError(f'inventory planning LLM failed: {exc}') from exc
            try:
                result = result['message']['content']
            except (KeyError, TypeError) as exc:
                raise RuntimeError('inventory planning LLM returned no content') from exc
        if isinstance(result, dict):
            return result
        if not isinstance(result, str):
            raise InventoryPlanValidationError('LLM plan must be a JSON object')
        text = result.strip()
        if text.startswith('```'):
            text = text.strip('`').strip()
            if text.startswith('json'):
                text = text[4:].strip()
        try:
            decoded = json.loads(text)
        except json.JSONDecodeError as exc:
            raise InventoryPlanValidationError('LLM plan is not valid JSON') from exc
        return decoded

    @staticmethod
    def validate_plan(raw_plan, objective, snapshot, known_locations):
        """Validate and simulate every move without causing side effects."""
        if not isinstance(snapshot, InventorySnapshot):
            raise InventoryPlanValidationError('snapshot has an invalid type')
        if not isinstance(raw_plan, dict):
            raise InventoryPlanValidationError('LLM plan must be an object')
        status = str(raw_plan.get('status', '')).strip()
        if status not in PLAN_STATUSES - {'error'}:
            raise InventoryPlanValidationError('LLM status must be ready or no_action')
        moves = raw_plan.get('moves')
        if not isinstance(moves, list):
            raise InventoryPlanValidationError('LLM moves must be an array')
        if status == 'no_action' and moves:
            raise InventoryPlanValidationError('no_action plan must have no moves')
        if status == 'ready' and not moves:
            raise InventoryPlanValidationError('ready plan must contain moves')

        locations = set(known_locations)
        by_name = {cargo.name: cargo for cargo in snapshot.cargos}
        by_id = {
            cargo.container_id: cargo
            for cargo in snapshot.cargos
            if cargo.container_id
        }
        seen_moves = set()
        normalized_moves = []

        for expected_sequence, raw_move in enumerate(moves, start=1):
            if not isinstance(raw_move, dict):
                raise InventoryPlanValidationError(
                    f'move {expected_sequence} must be an object'
                )
            missing = sorted(MOVE_FIELDS - set(raw_move))
            if missing:
                raise InventoryPlanValidationError(
                    f'move {expected_sequence} is missing: {", ".join(missing)}'
                )
            sequence = raw_move['sequence']
            if (
                isinstance(sequence, bool)
                or not isinstance(sequence, int)
                or sequence != expected_sequence
            ):
                raise InventoryPlanValidationError(
                    f'move sequence must be contiguous at {expected_sequence}'
                )
            container_id = str(raw_move['container_id']).strip()
            container_name = str(raw_move['container_name']).strip()
            cargo_by_id = by_id.get(container_id)
            cargo_by_name = by_name.get(container_name)
            if cargo_by_id is None or cargo_by_name is None or cargo_by_id != cargo_by_name:
                raise InventoryPlanValidationError(
                    f'move {sequence} references an unknown or mismatched container'
                )
            cargo = cargo_by_id
            source_location = str(raw_move['source_location']).strip()
            source_floor = raw_move['source_floor']
            if (
                isinstance(source_floor, bool)
                or not isinstance(source_floor, int)
                or source_location != cargo.location
                or source_floor != cargo.floor
            ):
                raise InventoryPlanValidationError(
                    f'move {sequence} source does not match the simulated inventory'
                )
            blockers = [
                value.container_id or value.name
                for value in by_name.values()
                if value.base_aruco_id == cargo.container_id
            ]
            if blockers:
                raise InventoryPlanValidationError(
                    f'move {sequence} container is blocked by: {", ".join(blockers)}'
                )

            destination_location = str(raw_move['destination_location']).strip()
            destination_floor = raw_move['destination_floor']
            destination_base = str(
                raw_move['destination_base_aruco_id'] or ''
            ).strip()
            reason = str(raw_move['reason']).strip()
            if destination_location not in locations:
                raise InventoryPlanValidationError(
                    f'move {sequence} destination is not registered: {destination_location}'
                )
            if destination_location == source_location:
                raise InventoryPlanValidationError(
                    f'move {sequence} source and destination are identical'
                )
            if (
                isinstance(destination_floor, bool)
                or not isinstance(destination_floor, int)
                or destination_floor < 1
            ):
                raise InventoryPlanValidationError(
                    f'move {sequence} destination_floor must be positive'
                )
            if not reason:
                raise InventoryPlanValidationError(
                    f'move {sequence} reason must not be empty'
                )
            if destination_floor > 1:
                base_cargo = by_id.get(destination_base)
                if (
                    base_cargo is None
                    or base_cargo.location != destination_location
                    or base_cargo.floor != destination_floor - 1
                ):
                    raise InventoryPlanValidationError(
                        f'move {sequence} destination base is inconsistent'
                    )
            if destination_base and any(
                value.name != cargo.name
                and value.location == destination_location
                and value.base_aruco_id == destination_base
                for value in by_name.values()
            ):
                raise InventoryPlanValidationError(
                    f'move {sequence} destination base is already occupied'
                )

            fingerprint = (
                container_id,
                source_location,
                source_floor,
                destination_location,
                destination_floor,
                destination_base,
            )
            if fingerprint in seen_moves:
                raise InventoryPlanValidationError(
                    f'move {sequence} duplicates an earlier move'
                )
            seen_moves.add(fingerprint)
            updated = replace(
                cargo,
                location=destination_location,
                floor=destination_floor,
                base_aruco_id=destination_base,
            )
            by_name[container_name] = updated
            by_id[container_id] = updated
            normalized_moves.append({
                'sequence': sequence,
                'container_id': container_id,
                'container_name': container_name,
                'source_location': source_location,
                'source_floor': source_floor,
                'destination_location': destination_location,
                'destination_floor': destination_floor,
                'destination_base_aruco_id': destination_base,
                'reason': reason,
            })

        return {
            'schema_version': PLAN_SCHEMA_VERSION,
            'plan_id': f'plan-{uuid.uuid4()}',
            'snapshot_id': snapshot.snapshot_id,
            'objective': str(objective).strip(),
            'status': status,
            'moves': normalized_moves,
            'summary': str(raw_plan.get('summary', '')).strip(),
            'error': '',
        }
