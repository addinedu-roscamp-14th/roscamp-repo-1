"""Deterministic policy and execution compilation for autonomous port cycles."""

from __future__ import annotations

from dataclasses import dataclass, field
import json
import os
import tempfile
import uuid


WAREHOUSE_LOCATIONS = tuple(
    f'A-{bay}-{slot}' for bay in range(1, 4) for slot in range(1, 3)
)
SHIP_LOCATIONS = tuple(f'선박-{slot}' for slot in range(1, 7))
CANONICAL_LOCATIONS = (
    *WAREHOUSE_LOCATIONS, 'AMR1', 'AMR2', *SHIP_LOCATIONS, '출항완료'
)
WAREHOUSE_MARKERS = {
    location: str(11 + index)
    for index, location in enumerate(WAREHOUSE_LOCATIONS)
}
SHIP_MARKERS = {
    location: str(18 + index) for index, location in enumerate(SHIP_LOCATIONS)
}
TRAILER_MARKERS = {'agv1': 10, 'agv2': 9}


class AutonomousPolicyError(RuntimeError):
    """Raised when current state cannot be executed safely."""


@dataclass
class AutonomousCycle:
    cycle_id: str = field(default_factory=lambda: f'cycle-{uuid.uuid4().hex[:12]}')
    phase: str = 'WAITING_FOR_INBOUND'
    inbound_ids: list[str] = field(default_factory=list)
    outbound_ids: list[str] = field(default_factory=list)
    active_move: dict | None = None
    active_mission_id: str = ''
    replan_count: int = 0
    failure_key: str = ''
    identical_failures: int = 0
    last_error: str = ''

    def to_dict(self):
        return {
            'cycle_id': self.cycle_id,
            'phase': self.phase,
            'inbound_ids': list(self.inbound_ids),
            'outbound_ids': list(self.outbound_ids),
            'active_move': self.active_move,
            'active_mission_id': self.active_mission_id,
            'replan_count': self.replan_count,
            'failure_key': self.failure_key,
            'identical_failures': self.identical_failures,
            'last_error': self.last_error,
        }

    @classmethod
    def from_dict(cls, value):
        value = value if isinstance(value, dict) else {}
        cycle = cls()
        for key in cycle.to_dict():
            if key in value:
                setattr(cycle, key, value[key])
        return cycle


class CycleStore:
    """Persist cycle identity so outbound cargo is not reclassified on restart."""

    def __init__(self, path=None):
        self.path = os.path.abspath(os.path.expanduser(path or os.environ.get(
            'PORT_AUTONOMY_STATE_PATH',
            '~/.local/state/port_control/autonomy_cycle.json',
        )))

    def load(self):
        try:
            with open(self.path, 'r', encoding='utf-8') as stream:
                return AutonomousCycle.from_dict(json.load(stream))
        except (OSError, ValueError, TypeError):
            return AutonomousCycle()

    def save(self, cycle):
        parent = os.path.dirname(self.path)
        os.makedirs(parent, exist_ok=True)
        descriptor, temporary = tempfile.mkstemp(
            prefix='.autonomy-', suffix='.json', dir=parent
        )
        try:
            with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
                json.dump(cycle.to_dict(), stream, ensure_ascii=False, indent=2)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


def cargo_rows(snapshot):
    if hasattr(snapshot, 'to_dict'):
        snapshot = snapshot.to_dict()
    return list((snapshot or {}).get('cargos') or [])


def cargo_by_id(snapshot):
    return {
        str(cargo.get('container_id')): cargo
        for cargo in cargo_rows(snapshot)
        if str(cargo.get('container_id') or '')
    }


def top_cargos(snapshot, location):
    values = [
        cargo for cargo in cargo_rows(snapshot)
        if str(cargo.get('location')) == location
    ]
    return sorted(values, key=lambda item: int(item.get('floor', 1)))


def destination_candidates(snapshot, locations, capacity=3):
    """Describe exact safe next-floor metadata for candidate destinations."""
    candidates = []
    for location in locations:
        stack = top_cargos(snapshot, location)
        if len(stack) >= capacity:
            continue
        candidates.append({
            'location': location,
            'destination_floor': len(stack) + 1,
            'destination_base_aruco_id': (
                '' if not stack else str(stack[-1].get('container_id') or '')
            ),
        })
    return candidates


def validate_first_move(move, snapshot):
    """Revalidate the first LLM move against the newest snapshot."""
    cargos = cargo_by_id(snapshot)
    container_id = str(move.get('container_id') or '')
    cargo = cargos.get(container_id)
    if cargo is None:
        raise AutonomousPolicyError(f'unknown container_id: {container_id}')
    source = str(move.get('source_location') or '')
    destination = str(move.get('destination_location') or '')
    if source != str(cargo.get('location')):
        raise AutonomousPolicyError(
            f'container {container_id} source changed: '
            f'{source} -> {cargo.get("location")}'
        )
    if source == destination:
        raise AutonomousPolicyError('source and destination must differ')
    if source not in CANONICAL_LOCATIONS or destination not in CANONICAL_LOCATIONS:
        raise AutonomousPolicyError('move uses a non-canonical location')
    source_stack = top_cargos(snapshot, source)
    if source_stack and source_stack[-1].get('container_id') != container_id:
        raise AutonomousPolicyError(
            f'container {container_id} is blocked by a higher floor'
        )
    destination_stack = top_cargos(snapshot, destination)
    capacity = 1 if destination in {'AMR1', 'AMR2', '출항완료'} else 3
    if destination != '출항완료' and len(destination_stack) >= capacity:
        raise AutonomousPolicyError(f'destination is full: {destination}')
    expected_floor = 1 if destination == '출항완료' else len(destination_stack) + 1
    requested_floor = int(move.get('destination_floor') or expected_floor)
    if requested_floor != expected_floor:
        raise AutonomousPolicyError(
            f'destination floor must be {expected_floor}, got {requested_floor}'
        )
    return dict(move, destination_floor=expected_floor)


def compile_move(move, vehicle_id):
    """Compile one validated container move into physical high-level steps."""
    source = str(move['source_location'])
    destination = str(move['destination_location'])
    container_id = int(move['container_id'])
    if vehicle_id not in TRAILER_MARKERS:
        raise AutonomousPolicyError(f'unsupported vehicle: {vehicle_id}')
    trailer_id = TRAILER_MARKERS[vehicle_id]
    trailer_location = 'AMR1' if vehicle_id == 'agv1' else 'AMR2'
    if source.startswith('선박-') and destination.startswith('A-'):
        return [
            {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': vehicle_id},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                'source_id': container_id, 'destination_id': trailer_id,
                'vehicle_id': vehicle_id, 'final_for_vehicle': True,
            },
            {'type': 'zone_navigation', 'zone': destination[:3], 'vehicle_id': vehicle_id},
            {
                'type': 'arm_transfer_to_slot', 'arm_id': 'arm2',
                'destination_slot': destination, 'vehicle_id': vehicle_id,
                'final_for_vehicle': True,
            },
            {'type': 'park_command', 'vehicle_id': vehicle_id},
        ]
    if source.startswith('A-') and destination.startswith('선박-'):
        return [
            {'type': 'zone_navigation', 'zone': source[:3], 'vehicle_id': vehicle_id},
            {
                'type': 'arm_load_to_trailer', 'arm_id': 'arm2',
                'source_id': container_id, 'vehicle_id': vehicle_id,
                'final_for_vehicle': True,
            },
            {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': vehicle_id},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                'source_id': trailer_id,
                'destination_id': int(SHIP_MARKERS[destination]),
                'vehicle_id': vehicle_id, 'final_for_vehicle': True,
            },
            {'type': 'park_command', 'vehicle_id': vehicle_id},
        ]
    if source.startswith('A-') and destination.startswith('A-'):
        return [{
            'type': 'arm_transfer_by_id', 'arm_id': 'arm2',
            'source_id': container_id,
            'destination_id': int(WAREHOUSE_MARKERS[destination]),
            'vehicle_id': '', 'final_for_vehicle': False,
        }]
    if source.startswith('선박-') and destination.startswith('선박-'):
        return [{
            'type': 'arm1_pick_place', 'arm_id': 'arm1',
            'source_id': container_id,
            'destination_id': int(SHIP_MARKERS[destination]),
            'vehicle_id': '', 'final_for_vehicle': False,
        }]
    raise AutonomousPolicyError(
        f'unsupported movement combination: {source} -> {destination}'
    )


def choose_policy(
    cycle, snapshot, port_present, visible_warehouse_zones=None
):
    """Return the fixed phase and LLM objective for the next policy action."""
    cargos = cargo_rows(snapshot)
    ship = [cargo for cargo in cargos if cargo.get('location') in SHIP_LOCATIONS]
    ship_ids = {str(cargo.get('container_id')) for cargo in ship}
    outbound = set(str(value) for value in cycle.outbound_ids)
    inbound = [cargo for cargo in ship if str(cargo.get('container_id')) not in outbound]

    if cycle.active_move:
        cycle.phase = 'EXECUTING_MOVE'
        return cycle.phase, ''
    if not port_present and outbound:
        cycle.phase = 'WAITING_FOR_CLEAR'
        return cycle.phase, ''
    if not port_present and not cycle.inbound_ids:
        cycle.phase = 'WAITING_FOR_INBOUND'
        return cycle.phase, ''
    if not ship and not cycle.inbound_ids and not outbound:
        cycle.phase = 'SCANNING_INBOUND'
        return cycle.phase, ''
    if inbound:
        for cargo in inbound:
            value = str(cargo.get('container_id'))
            if value not in cycle.inbound_ids:
                cycle.inbound_ids.append(value)
        cycle.phase = 'UNLOADING_INBOUND'
        ids = ', '.join(cycle.inbound_ids)
        warehouse_locations = WAREHOUSE_LOCATIONS
        if visible_warehouse_zones is not None:
            visible = {
                str(zone).upper() for zone in visible_warehouse_zones
            }
            warehouse_locations = tuple(
                location for location in warehouse_locations
                if location[:3].upper() in visible
            )
        destinations = destination_candidates(snapshot, warehouse_locations)
        return cycle.phase, (
            f'이번 회차 입항 컨테이너 [{ids}] 중 선박에 남은 화물을 '
            '최상단에서 하나만 선택하여 아래 목적지 후보 중 하나로 이동하라. '
            'moves 배열에는 이동 1건만 반환하고 선박 외 컨테이너는 이동하지 '
            '마라. 선택한 후보의 destination_floor와 '
            'destination_base_aruco_id를 그대로 사용하라. 목적지 후보 JSON: '
            f'{json.dumps(destinations, ensure_ascii=False)}'
        )
    third_floor = [
        cargo for cargo in cargos
        if str(cargo.get('location', '')).startswith('A-')
        and int(cargo.get('floor', 1)) >= 3
    ]
    if third_floor:
        cycle.phase = 'LOADING_OUTBOUND'
        ids = ', '.join(str(cargo.get('container_id')) for cargo in third_floor)
        destinations = destination_candidates(snapshot, SHIP_LOCATIONS)
        return cycle.phase, (
            f'창고 3층 최상단 컨테이너 [{ids}] 중 하나만 아래 선박 목적지 '
            '후보 중 하나로 이동하라. moves 배열에는 이동 1건만 반환하고 '
            '선택한 후보의 destination_floor와 destination_base_aruco_id를 '
            '그대로 사용하라. 목적지 후보 JSON: '
            f'{json.dumps(destinations, ensure_ascii=False)}'
        )
    if outbound and ship_ids & outbound:
        cycle.phase = 'WAITING_FOR_CLEAR'
        return cycle.phase, ''
    cycle.phase = 'WAITING_FOR_CLEAR'
    return cycle.phase, ''
