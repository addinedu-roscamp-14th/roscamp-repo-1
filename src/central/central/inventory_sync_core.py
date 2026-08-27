"""Durable, idempotent PostgreSQL synchronization for cargo movements."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import json
import os
import sqlite3
import threading


WAREHOUSE_MARKERS = {
    'A-1-1': '11', 'A-1-2': '12',
    'A-2-1': '13', 'A-2-2': '14',
    'A-3-1': '15', 'A-3-2': '16',
}
SHIP_MARKERS = {f'선박-{index}': str(17 + index) for index in range(2, 7)}
LOCATION_MARKERS = {
    **WAREHOUSE_MARKERS,
    **SHIP_MARKERS,
    'AMR1': '10',
    'AMR2': '9',
    '출항완료': '',
}
VALID_LOCATIONS = frozenset(LOCATION_MARKERS)


class InventorySyncError(RuntimeError):
    """Raised when a movement cannot safely be persisted."""


def utc_now_iso():
    return datetime.now(timezone.utc).isoformat()


def normalize_movement(payload):
    """Validate and normalize one ARM movement event."""
    if not isinstance(payload, dict):
        raise InventorySyncError('movement event must be a JSON object')
    operation_id = str(payload.get('operation_id') or '').strip()
    container_id = str(payload.get('container_id') or '').strip()
    if not operation_id:
        raise InventorySyncError('operation_id is required')
    if not container_id:
        raise InventorySyncError('container_id is required')
    result = dict(payload)
    result['operation_id'] = operation_id
    result['container_id'] = container_id
    result['success'] = bool(payload.get('success'))
    result['source_location'] = str(
        payload.get('source_location') or ''
    ).strip()
    result['destination_location'] = str(
        payload.get('destination_location') or ''
    ).strip()
    for key in ('source_location', 'destination_location'):
        value = result[key]
        if value and value not in VALID_LOCATIONS:
            raise InventorySyncError(f'unsupported {key}: {value}')
    if result['success'] and not result['destination_location']:
        raise InventorySyncError(
            'successful movement requires destination_location'
        )
    for key in ('source_floor', 'destination_floor'):
        value = payload.get(key)
        if value in (None, ''):
            result[key] = None
            continue
        try:
            value = int(value)
        except (TypeError, ValueError) as exc:
            raise InventorySyncError(f'{key} must be an integer') from exc
        if value < 1:
            raise InventorySyncError(f'{key} must be positive')
        result[key] = value
    result['source_base_aruco_id'] = str(
        payload.get('source_base_aruco_id') or ''
    ).strip()
    result['destination_base_aruco_id'] = str(
        payload.get('destination_base_aruco_id') or ''
    ).strip()
    result['completed_at'] = str(
        payload.get('completed_at') or utc_now_iso()
    )
    result['error'] = str(payload.get('error') or '')
    return result


@dataclass(frozen=True)
class SyncStatus:
    state: str
    pending_count: int
    last_error: str = ''
    last_operation_id: str = ''

    def to_dict(self):
        return {
            'state': self.state,
            'pending_count': self.pending_count,
            'last_error': self.last_error,
            'last_operation_id': self.last_operation_id,
        }


class SQLiteMovementOutbox:
    """Store every event locally before attempting remote DB writes."""

    def __init__(self, path):
        self.path = os.path.abspath(os.path.expanduser(str(path)))
        parent = os.path.dirname(self.path)
        if parent:
            os.makedirs(parent, exist_ok=True)
        self._lock = threading.RLock()
        self._connection = sqlite3.connect(
            self.path, check_same_thread=False
        )
        self._connection.execute('PRAGMA journal_mode=WAL')
        self._connection.execute(
            'CREATE TABLE IF NOT EXISTS movement_outbox ('
            'operation_id TEXT PRIMARY KEY, payload TEXT NOT NULL, '
            'created_at TEXT NOT NULL, attempts INTEGER NOT NULL DEFAULT 0, '
            'last_error TEXT NOT NULL DEFAULT "")'
        )
        self._connection.commit()

    def enqueue(self, payload):
        event = normalize_movement(payload)
        with self._lock:
            self._connection.execute(
                'INSERT OR IGNORE INTO movement_outbox '
                '(operation_id, payload, created_at) VALUES (?, ?, ?)',
                (
                    event['operation_id'],
                    json.dumps(event, ensure_ascii=False, sort_keys=True),
                    utc_now_iso(),
                ),
            )
            self._connection.commit()
        return event

    def pending(self, limit=100):
        with self._lock:
            rows = self._connection.execute(
                'SELECT operation_id, payload FROM movement_outbox '
                'ORDER BY created_at, operation_id LIMIT ?',
                (int(limit),),
            ).fetchall()
        return [(row[0], json.loads(row[1])) for row in rows]

    def count(self):
        with self._lock:
            return int(self._connection.execute(
                'SELECT COUNT(*) FROM movement_outbox'
            ).fetchone()[0])

    def delivered(self, operation_id):
        with self._lock:
            self._connection.execute(
                'DELETE FROM movement_outbox WHERE operation_id = ?',
                (str(operation_id),),
            )
            self._connection.commit()

    def failed(self, operation_id, error):
        with self._lock:
            self._connection.execute(
                'UPDATE movement_outbox SET attempts = attempts + 1, '
                'last_error = ? WHERE operation_id = ?',
                (str(error), str(operation_id)),
            )
            self._connection.commit()

    def close(self):
        with self._lock:
            self._connection.close()


class PostgresInventoryWriter:
    """Apply events transactionally to audit history and current state."""

    SCHEMA_SQL = (
        'CREATE TABLE IF NOT EXISTS cargo_movements ('
        "operation_id TEXT PRIMARY KEY, command_id TEXT NOT NULL DEFAULT '', "
        "mission_id TEXT NOT NULL DEFAULT '', arm_id TEXT NOT NULL, "
        "container_id TEXT NOT NULL, source_location TEXT NOT NULL DEFAULT '', "
        "source_floor INTEGER, source_base_aruco_id TEXT NOT NULL DEFAULT '', "
        "destination_location TEXT NOT NULL DEFAULT '', "
        'destination_floor INTEGER, '
        "destination_base_aruco_id TEXT NOT NULL DEFAULT '', "
        'success BOOLEAN NOT NULL, completed_at TIMESTAMPTZ NOT NULL, '
        "error TEXT NOT NULL DEFAULT '', "
        "observation_mismatch TEXT NOT NULL DEFAULT '', "
        'event_json JSONB NOT NULL)'
    )

    def __init__(self, connect, connection_kwargs=None):
        self._connect = connect
        self.connection_kwargs = dict(connection_kwargs or {})

    @classmethod
    def from_environment(cls, connect):
        return cls(connect, {
            'host': os.environ.get('PORT_INVENTORY_DB_HOST', '192.168.5.5'),
            'port': int(os.environ.get('PORT_INVENTORY_DB_PORT', '5432')),
            'dbname': os.environ.get('PORT_INVENTORY_DB_NAME', 'port_db'),
            'user': os.environ.get('PORT_INVENTORY_DB_USER', 'postgres'),
            'password': os.environ.get('PORT_INVENTORY_DB_PASSWORD', '1234'),
            'connect_timeout': max(1, int(float(os.environ.get(
                'PORT_INVENTORY_DB_TIMEOUT_SEC', '3'
            )))),
        })

    def apply(self, payload):
        event = normalize_movement(payload)
        connection = self._connect(**self.connection_kwargs)
        cursor = connection.cursor()
        try:
            cursor.execute(self.SCHEMA_SQL)
            cursor.execute(
                'SELECT 1 FROM cargo_movements WHERE operation_id = %s',
                (event['operation_id'],),
            )
            if cursor.fetchone():
                connection.commit()
                return {'duplicate': True, 'mismatch': ''}

            current = self._cargo_for_update(cursor, event['container_id'])
            mismatch = ''
            if event['success']:
                if current is None:
                    self._register_unknown(cursor, event['container_id'])
                    current = self._cargo_for_update(
                        cursor, event['container_id']
                    )
                mismatch = self._mismatch_message(current, event)
                displaced = self._reconcile_inbound_observation(cursor, event)
                if displaced:
                    detail = (
                        f'robot observed {event["container_id"]} at '
                        f'{event["destination_location"]}; displaced DB cargo='
                        f'{",".join(displaced)}'
                    )
                    mismatch = '; '.join(filter(None, (mismatch, detail)))
                self._apply_success(cursor, current, event, mismatch)
            self._insert_audit(cursor, event, mismatch)
            connection.commit()
            return {'duplicate': False, 'mismatch': mismatch}
        except Exception:
            connection.rollback()
            raise
        finally:
            cursor.close()
            connection.close()

    @staticmethod
    def _cargo_for_update(cursor, container_id):
        cursor.execute(
            'SELECT name, location, container_id, cargo_type, note, '
            'base_aruco_id, floor FROM cargos '
            'WHERE container_id = %s FOR UPDATE',
            (container_id,),
        )
        rows = cursor.fetchall()
        if len(rows) > 1:
            raise InventorySyncError(
                f'duplicate cargos.container_id: {container_id}'
            )
        return rows[0] if rows else None

    @staticmethod
    def _register_unknown(cursor, container_id):
        cursor.execute(
            'INSERT INTO cargos '
            '(name, location, container_id, cargo_type, note, '
            'base_aruco_id, floor) VALUES (%s, %s, %s, %s, %s, %s, %s) '
            'ON CONFLICT (name) DO NOTHING',
            (
                f'컨테이너_C{container_id}', '출항완료', container_id,
                '미분류', 'ARM 관측으로 자동 등록', '', 1,
            ),
        )

    @staticmethod
    def _mismatch_message(current, event):
        if current is None or not event['source_location']:
            return ''
        _, location, _, _, _, base, floor = current
        differences = []
        if str(location) != event['source_location']:
            differences.append(
                f'location DB={location}, robot={event["source_location"]}'
            )
        if event['source_floor'] and int(floor) != event['source_floor']:
            differences.append(
                f'floor DB={floor}, robot={event["source_floor"]}'
            )
        if (
            event['source_base_aruco_id']
            and str(base or '') != event['source_base_aruco_id']
        ):
            differences.append(
                'base DB='
                f'{base}, robot={event["source_base_aruco_id"]}'
            )
        return '; '.join(differences)

    @staticmethod
    def _is_inbound_observation(event):
        """Return whether an event is an authoritative ARM1 ship scan.

        An inbound scan describes physical state; it is not a stacking move.
        Ship slots accept one inbound cargo, so stale DB occupants must not
        make a floor-1 observation fail with "destination floor must be 2".
        """
        operation_id = str(event.get('operation_id') or '')
        command_id = str(event.get('command_id') or '')
        return bool(
            event.get('success')
            and str(event.get('arm_id') or '').lower() == 'arm1'
            and not str(event.get('source_location') or '')
            and event.get('destination_location') in SHIP_MARKERS
            and (
                operation_id.startswith('inbound-')
                or '-inbound-' in command_id
                or command_id.startswith('inbound-')
            )
        )

    def _reconcile_inbound_observation(self, cursor, event):
        """Make one observed ship slot authoritative and return displaced IDs."""
        if not self._is_inbound_observation(event):
            return []
        destination = event['destination_location']
        cursor.execute(
            'SELECT name, container_id, note FROM cargos '
            'WHERE location = %s AND container_id <> %s FOR UPDATE',
            (destination, event['container_id']),
        )
        rows = cursor.fetchall()
        for name, container_id, note in rows:
            suffix = (
                f'[관측우선 {event["operation_id"]}] ARM1이 {destination}에서 '
                f'{event["container_id"]}을 관측하여 기존 위치를 보정'
            )
            updated_note = f'{str(note or "")} | {suffix}'.strip(' |')
            cursor.execute(
                "UPDATE cargos SET location = '출항완료', floor = 1, "
                "base_aruco_id = '', note = %s WHERE name = %s",
                (updated_note, name),
            )
        return [str(row[1]) for row in rows]

    def _apply_success(self, cursor, current, event, mismatch):
        name, db_location, _, _, note, _, db_floor = current
        source_location = event['source_location'] or str(db_location)
        source_floor = event['source_floor'] or int(db_floor)
        destination = event['destination_location']

        if source_location == destination:
            if event['source_location']:
                raise InventorySyncError('source and destination must differ')
            # A repeated scanner observation confirms the same slot. It is
            # still audited, but it must not reshuffle or duplicate cargo.
            return
        if str(db_location) != source_location:
            self._compact_stack(
                cursor, str(db_location), int(db_floor), event['container_id']
            )
        self._compact_stack(
            cursor, source_location, source_floor, event['container_id']
        )
        self._compact_stack(
            cursor, destination, 0, event['container_id']
        )
        destination_floor, destination_base = self._destination_support(
            cursor, destination, event
        )
        updated_note = str(note or '')
        if mismatch:
            suffix = f'[관측우선 {event["operation_id"]}] {mismatch}'
            updated_note = f'{updated_note} | {suffix}'.strip(' |')
        cursor.execute(
            'UPDATE cargos SET location = %s, floor = %s, '
            'base_aruco_id = %s, note = %s WHERE name = %s',
            (
                destination, destination_floor, destination_base,
                updated_note, name,
            ),
        )

    @staticmethod
    def _stack_rows(cursor, location, excluded_id=''):
        cursor.execute(
            'SELECT name, container_id, floor FROM cargos '
            'WHERE location = %s AND container_id <> %s '
            'ORDER BY floor, name FOR UPDATE',
            (location, excluded_id),
        )
        return cursor.fetchall()

    def _compact_stack(self, cursor, location, removed_floor, excluded_id):
        if location not in WAREHOUSE_MARKERS and location not in SHIP_MARKERS:
            return
        rows = self._stack_rows(cursor, location, excluded_id)
        for floor, row in enumerate(rows, 1):
            name, container_id, _old_floor = row
            base = (
                LOCATION_MARKERS[location]
                if floor == 1 else str(rows[floor - 2][1])
            )
            cursor.execute(
                'UPDATE cargos SET floor = %s, base_aruco_id = %s '
                'WHERE name = %s',
                (floor, base, name),
            )

    def _destination_support(self, cursor, location, event):
        if location == '출항완료':
            return 1, ''
        if location in {'AMR1', 'AMR2'}:
            rows = self._stack_rows(cursor, location, event['container_id'])
            if rows:
                raise InventorySyncError(f'{location} already carries cargo')
            return 1, LOCATION_MARKERS[location]
        rows = self._stack_rows(cursor, location, event['container_id'])
        expected_floor = len(rows) + 1
        requested_floor = event['destination_floor']
        if requested_floor and requested_floor != expected_floor:
            raise InventorySyncError(
                f'destination floor must be {expected_floor}, '
                f'got {requested_floor}'
            )
        max_floor = 3
        if expected_floor > max_floor:
            raise InventorySyncError(f'{location} exceeds {max_floor} floors')
        base = (
            LOCATION_MARKERS[location]
            if expected_floor == 1 else str(rows[-1][1])
        )
        observed_base = event['destination_base_aruco_id']
        if observed_base and observed_base != base:
            raise InventorySyncError(
                f'destination support mismatch: expected {base}, '
                f'observed {observed_base}'
            )
        return expected_floor, base

    @staticmethod
    def _insert_audit(cursor, event, mismatch):
        cursor.execute(
            'INSERT INTO cargo_movements '
            '(operation_id, command_id, mission_id, arm_id, container_id, '
            'source_location, source_floor, source_base_aruco_id, '
            'destination_location, destination_floor, '
            'destination_base_aruco_id, success, completed_at, error, '
            'observation_mismatch, event_json) VALUES '
            '(%s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, %s, '
            '%s, %s, %s, %s::jsonb)',
            (
                event['operation_id'], str(event.get('command_id') or ''),
                str(event.get('mission_id') or ''),
                str(event.get('arm_id') or ''), event['container_id'],
                event['source_location'], event['source_floor'],
                event['source_base_aruco_id'],
                event['destination_location'], event['destination_floor'],
                event['destination_base_aruco_id'], event['success'],
                event['completed_at'], event['error'], mismatch,
                json.dumps(event, ensure_ascii=False, sort_keys=True),
            ),
        )


class InventorySynchronizer:
    """Coordinate the durable outbox and remote writer."""

    def __init__(self, outbox, writer):
        self.outbox = outbox
        self.writer = writer
        self.last_error = ''
        self.last_operation_id = ''

    def submit(self, payload):
        event = self.outbox.enqueue(payload)
        self.flush()
        return event

    def flush(self, limit=100):
        for operation_id, payload in self.outbox.pending(limit):
            try:
                self.writer.apply(payload)
            except Exception as exc:
                self.last_error = str(exc)
                self.outbox.failed(operation_id, exc)
                break
            self.outbox.delivered(operation_id)
            self.last_operation_id = operation_id
            self.last_error = ''
        return self.status()

    def status(self):
        pending = self.outbox.count()
        return SyncStatus(
            state='BLOCKED' if pending else 'READY',
            pending_count=pending,
            last_error=self.last_error,
            last_operation_id=self.last_operation_id,
        )
