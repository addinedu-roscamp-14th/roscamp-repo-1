"""Tests for the durable inventory movement outbox."""

from central.inventory_sync_core import (
    InventorySynchronizer,
    normalize_movement,
    SQLiteMovementOutbox,
)


def movement(operation_id='op-1'):
    return {
        'operation_id': operation_id,
        'container_id': '6',
        'arm_id': 'arm1',
        'source_location': '선박-1',
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
