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
    outbound_seen_in_roi: bool = False
    active_move: dict | None = None
    active_mission_id: str = ''
    active_moves: dict[str, dict] = field(default_factory=dict)
    last_vehicle_id: str = ''
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
            'outbound_seen_in_roi': self.outbound_seen_in_roi,
            'active_move': self.active_move,
            'active_mission_id': self.active_mission_id,
            'active_moves': dict(self.active_moves),
            'last_vehicle_id': self.last_vehicle_id,
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
    """Persist current-cycle diagnostics while the dashboard is running."""

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


def destination_candidates(
    snapshot, locations, capacity=3, excluded_locations=None
):
    """Describe exact safe next-floor metadata for candidate destinations."""
    excluded = {str(value) for value in (excluded_locations or [])}
    candidates = []
    for location in locations:
        if location in excluded:
            continue
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
    # Moves performed entirely by one stationary arm do not consume an AMR.
    if source.startswith('A-') and destination.startswith('A-'):
        destination_floor = int(move.get('destination_floor') or 1)
        if destination_floor > 1:
            try:
                destination_id = int(
                    str(move.get('destination_base_aruco_id') or '')
                )
            except (TypeError, ValueError) as exc:
                raise AutonomousPolicyError(
                    'stacked warehouse move requires the supporting '
                    'container ID'
                ) from exc
            if destination_id not in range(9):
                raise AutonomousPolicyError(
                    'warehouse stack support must be container ID 0..8'
                )
        else:
            destination_id = int(WAREHOUSE_MARKERS[destination])
        return [{
            'type': 'arm_transfer_by_id', 'arm_id': 'arm2',
            'source_id': container_id,
            'destination_id': destination_id,
            'destination_slot': destination,
            'destination_floor': destination_floor,
            'vehicle_id': '', 'final_for_vehicle': False,
        }]
    if source.startswith('선박-') and destination.startswith('선박-'):
        return [{
            'type': 'arm1_pick_place', 'arm_id': 'arm1',
            'source_id': container_id,
            'destination_id': int(SHIP_MARKERS[destination]),
            'vehicle_id': '', 'final_for_vehicle': False,
        }]
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
            # Every warehouse slot is served from the shared ARM2 stop A.
            # The exact A-x-y slot belongs only to the following ARM command.
            {'type': 'zone_navigation', 'zone': 'A', 'vehicle_id': vehicle_id},
            {
                'type': 'arm_transfer_to_slot', 'arm_id': 'arm2',
                'destination_slot': destination, 'vehicle_id': vehicle_id,
                'destination_floor': int(move['destination_floor']),
                'final_for_vehicle': True,
            },
            # ARM2 success only releases the arm-side safety gate.  The
            # vehicle still owns exclusive zone A until it physically leaves,
            # so finish the move by driving it out to its assigned park.
            {'type': 'park_command', 'vehicle_id': vehicle_id},
        ]
    if source.startswith('A-') and destination.startswith('선박-'):
        return [
            {'type': 'zone_navigation', 'zone': 'A', 'vehicle_id': vehicle_id},
            {
                'type': 'arm_load_to_trailer', 'arm_id': 'arm2',
                'source_id': container_id, 'vehicle_id': vehicle_id,
                'final_for_vehicle': True,
            },
            {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': vehicle_id},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                # The container hides or partially occludes the trailer
                # marker once ARM2 has loaded it.  ARM1 must therefore pick
                # the exposed cargo marker, while vehicle_id still identifies
                # which trailer is positioned at B-1.
                'source_id': container_id,
                'destination_id': int(SHIP_MARKERS[destination]),
                'vehicle_id': vehicle_id, 'final_for_vehicle': True,
            },
            # Likewise, do not leave an empty vehicle holding B-1 after the
            # ship placement has completed.
            {'type': 'park_command', 'vehicle_id': vehicle_id},
        ]
    if source in {'AMR1', 'AMR2'} and destination.startswith('선박-'):
        expected_vehicle = 'agv1' if source == 'AMR1' else 'agv2'
        if vehicle_id != expected_vehicle:
            raise AutonomousPolicyError(
                f'{source} cargo must use {expected_vehicle}, got {vehicle_id}'
            )
        return [
            {'type': 'zone_navigation', 'zone': 'B-1', 'vehicle_id': vehicle_id},
            {
                'type': 'arm1_pick_place', 'arm_id': 'arm1',
                'source_id': container_id,
                'destination_id': int(SHIP_MARKERS[destination]),
                'vehicle_id': vehicle_id, 'final_for_vehicle': True,
            },
            {'type': 'park_command', 'vehicle_id': vehicle_id},
        ]
    raise AutonomousPolicyError(
        f'unsupported movement combination: {source} -> {destination}'
    )


def choose_policy(
    cycle, snapshot, port_present, visible_warehouse_zones=None,
    reserved_container_ids=None, reserved_destinations=None,
):
    """Return the fixed phase and LLM objective for the next policy action."""
    cargos = cargo_rows(snapshot)
    current_by_id = {
        str(cargo.get('container_id')): cargo for cargo in cargos
    }
    cycle.outbound_ids = [
        str(container_id) for container_id in cycle.outbound_ids
        if str(container_id) in current_by_id
        and str(current_by_id[str(container_id)].get('location') or '')
        != '출항완료'
    ]
    ship = [cargo for cargo in cargos if cargo.get('location') in SHIP_LOCATIONS]
    ship_ids = {str(cargo.get('container_id')) for cargo in ship}
    outbound = set(str(value) for value in cycle.outbound_ids)
    reserved_ids = {str(value) for value in (reserved_container_ids or [])}
    reserved_destinations = {
        str(value) for value in (reserved_destinations or [])
    }
    all_inbound = [
        cargo for cargo in ship
        if str(cargo.get('container_id')) not in outbound
    ]
    inbound = [
        cargo for cargo in all_inbound
        if str(cargo.get('container_id')) not in reserved_ids
    ]
    third_floor = [
        cargo for cargo in cargos
        if str(cargo.get('location', '')).startswith('A-')
        and int(cargo.get('floor', 1)) >= 3
        and str(cargo.get('container_id')) not in reserved_ids
    ]
    outbound_in_transit = [
        cargo for cargo in cargos
        if str(cargo.get('container_id')) in outbound
        and str(cargo.get('location') or '') in {'AMR1', 'AMR2'}
        and str(cargo.get('container_id')) not in reserved_ids
    ]

    active_values = list((cycle.active_moves or {}).values())
    if cycle.active_move and cycle.active_move not in active_values:
        active_values.append(cycle.active_move)
    active_outbound = any(
        str(value.get('container_id') or '') in outbound
        and str(value.get('destination_location') or '') in SHIP_LOCATIONS
        for value in active_values
    )
    if active_values and not inbound and any(
        str(value.get('source_location') or '').startswith('선박-')
        for value in active_values
    ):
        cycle.phase = 'EXECUTING_MOVE'
        return cycle.phase, ''
    if active_outbound and not inbound and not third_floor:
        cycle.phase = 'EXECUTING_MOVE'
        return cycle.phase, ''
    if outbound_in_transit:
        cycle.phase = 'LOADING_OUTBOUND'
        cargo = outbound_in_transit[0]
        destinations = destination_candidates(
            snapshot,
            SHIP_LOCATIONS,
            excluded_locations=reserved_destinations,
        )
        return cycle.phase, (
            f'출항 작업 중 {cargo.get("location")} 트레일러에 남은 컨테이너 '
            f'[{cargo.get("container_id")}]를 아래 선박 목적지 후보 중 하나로 '
            '이동하라. moves 배열에는 이 컨테이너 이동 1건만 반환하고 '
            'source_location을 현재 AMR 위치 그대로 사용하라. 목적지 후보 JSON: '
            f'{json.dumps(destinations, ensure_ascii=False)}'
        )
    if not port_present and outbound and not third_floor:
        cycle.phase = 'WAITING_FOR_CLEAR'
        return cycle.phase, ''
    # An empty vessel ROI normally means that we wait for inbound cargo, but
    # warehouse third-floor cargo is an independent outbound trigger.  Check
    # it below before falling back to WAITING_FOR_INBOUND.
    if not port_present and not cycle.inbound_ids and not third_floor:
        cycle.phase = 'WAITING_FOR_INBOUND'
        return cycle.phase, ''
    if not ship and not cycle.inbound_ids and not outbound and not third_floor:
        cycle.phase = 'SCANNING_INBOUND'
        return cycle.phase, ''
    if inbound:
        for cargo in inbound:
            value = str(cargo.get('container_id'))
            if value not in cycle.inbound_ids:
                cycle.inbound_ids.append(value)
        cycle.phase = 'UNLOADING_INBOUND'
        ids = ', '.join(
            str(cargo.get('container_id')) for cargo in inbound
        )
        warehouse_locations = WAREHOUSE_LOCATIONS
        if visible_warehouse_zones is not None:
            visible = {
                str(zone).upper() for zone in visible_warehouse_zones
            }
            warehouse_locations = tuple(
                location for location in warehouse_locations
                if location[:3].upper() in visible
            )
        destinations = destination_candidates(
            snapshot,
            warehouse_locations,
            excluded_locations=reserved_destinations,
        )
        return cycle.phase, (
            f'이번 회차 입항 컨테이너 [{ids}] 중 선박에 남은 화물을 '
            '최상단에서 하나만 선택하여 아래 목적지 후보 중 하나로 이동하라. '
            'moves 배열에는 이동 1건만 반환하고 선박 외 컨테이너는 이동하지 '
            '마라. 선택한 후보의 destination_floor와 '
            'destination_base_aruco_id를 그대로 사용하라. 목적지 후보 JSON: '
            f'{json.dumps(destinations, ensure_ascii=False)}'
        )
    if third_floor:
        cycle.phase = 'LOADING_OUTBOUND'
        ids = ', '.join(str(cargo.get('container_id')) for cargo in third_floor)
        destinations = destination_candidates(
            snapshot,
            SHIP_LOCATIONS,
            excluded_locations=reserved_destinations,
        )
        ship_marker_mapping = {
            location: int(marker_id)
            for location, marker_id in SHIP_MARKERS.items()
        }
        return cycle.phase, (
            f'창고 3층 최상단 컨테이너 [{ids}] 중 하나만 아래 선박 목적지 '
            '후보 중 하나로 이동하라. moves 배열에는 이동 1건만 반환하고 '
            '선택한 후보의 destination_floor와 destination_base_aruco_id를 '
            '그대로 사용하라. destination_location에는 반드시 선박-1부터 '
            '선박-6 중 하나를 정확히 넣어라. 목적지 후보 JSON: '
            f'{json.dumps(destinations, ensure_ascii=False)}. '
            'ARM1 선박 목적지 ArUco 매핑 JSON: '
            f'{json.dumps(ship_marker_mapping, ensure_ascii=False)}'
        )
    if active_values:
        cycle.phase = 'EXECUTING_MOVE'
        return cycle.phase, ''
    if outbound and ship_ids & outbound:
        cycle.phase = 'WAITING_FOR_CLEAR'
        return cycle.phase, ''
    cycle.phase = 'WAITING_FOR_CLEAR'
    return cycle.phase, ''
