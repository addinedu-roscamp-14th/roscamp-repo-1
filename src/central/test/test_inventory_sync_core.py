"""Tests for the durable inventory movement outbox."""

from central.inventory_sync_core import (
    InventorySynchronizer,
    PostgresInventoryWriter,
    normalize_movement,
    SQLiteMovementOutbox,
)


def movement(operation_id='op-1'):
    return {
        'operation_id': operation_id,
        'container_id': '6',
        'arm_id': 'arm1',
        'source_location': '선박-2',
        'destination_location': 'AMR1',
        'success': True,
    }


def test_outbox_deduplicates_operation_id(tmp_path):
    outbox = SQLiteMovementOutbox(tmp_path / 'outbox.sqlite3')
    outbox.enqueue(movement())
    outbox.enqueue(movement())
    assert outbox.count() == 1
    outbox.close()


def test_db_failure_keeps_event_and_recovery_delivers_once(tmp_path):
    class Writer:
        def __init__(self):
            self.fail = True
            self.applied = []

        def apply(self, payload):
            if self.fail:
                raise RuntimeError('db offline')
            self.applied.append(payload['operation_id'])

    outbox = SQLiteMovementOutbox(tmp_path / 'outbox.sqlite3')
    writer = Writer()
    sync = InventorySynchronizer(outbox, writer)
    sync.submit(movement())
    assert sync.status().state == 'BLOCKED'
    assert sync.status().pending_count == 1
    writer.fail = False
    sync.flush()
    sync.flush()
    assert writer.applied == ['op-1']
    assert sync.status().state == 'READY'
    outbox.close()


def test_failed_physical_event_is_still_valid_audit_input():
    value = movement()
    value.update(success=False, destination_location='', error='grip failed')
    normalized = normalize_movement(value)
    assert normalized['success'] is False
    assert normalized['error'] == 'grip failed'


def test_inbound_scan_is_authoritative_observation():
    event = {
        'operation_id': 'inbound-scan-4',
        'command_id': 'port-1-inbound-scan',
        'container_id': '4',
        'arm_id': 'arm1',
        'source_location': '',
        'destination_location': '선박-4',
        'destination_floor': 1,
        'success': True,
    }
    assert PostgresInventoryWriter._is_inbound_observation(event)
    event['source_location'] = 'AMR1'
    assert not PostgresInventoryWriter._is_inbound_observation(event)


def test_inbound_observation_displaces_stale_ship_slot_occupant():
    class Cursor:
        def __init__(self):
            self.rows = [('컨테이너_C6', '6', '기존 상태')]
            self.updated = []

        def execute(self, statement, parameters):
            if statement.startswith('UPDATE cargos SET location'):
                self.updated.append(parameters)

        def fetchall(self):
            return self.rows

    event = normalize_movement({
        'operation_id': 'inbound-scan-4',
        'command_id': 'port-1-inbound-scan',
        'container_id': '4',
        'arm_id': 'arm1',
        'source_location': '',
        'destination_location': '선박-4',
        'destination_floor': 1,
        'success': True,
    })
    cursor = Cursor()
    writer = object.__new__(PostgresInventoryWriter)

    displaced = writer._reconcile_inbound_observation(cursor, event)

    assert displaced == ['6']
    assert len(cursor.updated) == 1
    note, name = cursor.updated[0]
    assert name == '컨테이너_C6'
    assert '관측우선' in note
