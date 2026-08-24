"""Tests for the read-only PostgreSQL inventory client."""

from datetime import datetime, timezone

import pytest

from inventory_client import (
    InventoryAdminClient,
    InventoryClient,
    InventoryClientError,
)


NOW = datetime(2026, 8, 11, 3, 0, 0, tzinfo=timezone.utc)
ROWS = [
    ('컨테이너_C0', 'A-1-1', '0', '컨테이너', '', '11', 1),
]


class FakeCursor:
    def __init__(self, rows):
        self.rows = rows
        self.query = ''
        self.queries = []
        self.closed = False

    def execute(self, query, params=None):
        self.query = query
        self.queries.append(query)
        self.params = params

    def fetchall(self):
        return self.rows

    def close(self):
        self.closed = True


class FakeConnection:
    def __init__(self, rows):
        self.cursor_value = FakeCursor(rows)
        self.session = None
        self.rolled_back = False
        self.committed = False
        self.closed = False

    def set_session(self, **kwargs):
        self.session = kwargs

    def cursor(self):
        return self.cursor_value

    def rollback(self):
        self.rolled_back = True

    def commit(self):
        self.committed = True

    def close(self):
        self.closed = True


def test_fetches_cargos_using_read_only_postgresql_session():
    captured = {}
    connection = FakeConnection(ROWS)

    def connect(**kwargs):
        captured.update(kwargs)
        return connection

    client = InventoryClient(
        host='10.11.4.249',
        database='port_db',
        user='postgres',
        password='1234',
        timeout_sec=3,
        connect=connect,
        now=lambda: NOW,
    )

    snapshot = client.fetch_snapshot()

    assert captured == {
        'host': '10.11.4.249',
        'port': 5432,
        'dbname': 'port_db',
        'user': 'postgres',
        'password': '1234',
        'connect_timeout': 3,
    }
    assert connection.session == {'readonly': True, 'autocommit': False}
    assert connection.cursor_value.query == InventoryClient.SELECT_CARGOS
    assert snapshot.cargos[0].container_id == '0'
    assert snapshot.snapshot_id.startswith('sql-')
    assert connection.rolled_back and connection.closed


def test_connection_failure_fails_closed():
    def connect(**kwargs):
        raise OSError('database offline')

    client = InventoryClient(connect=connect)

    with pytest.raises(InventoryClientError, match='query failed'):
        client.fetch_snapshot()


def test_operator_inventory_reset_clears_history_and_current_state_atomically():
    connection = FakeConnection([])
    reader = InventoryClient(connect=lambda **kwargs: connection)
    admin = InventoryAdminClient(reader)

    admin.clear_inventory()

    assert connection.cursor_value.queries == [
        'TRUNCATE TABLE cargo_movements',
        'DELETE FROM cargos',
    ]
    assert connection.committed
    assert not connection.rolled_back
    assert connection.cursor_value.closed
    assert connection.closed


def test_operator_can_delete_one_inventory_row_without_clearing_history():
    connection = FakeConnection([])
    reader = InventoryClient(connect=lambda **kwargs: connection)
    admin = InventoryAdminClient(reader)

    admin.delete_cargo('컨테이너_C6')

    assert connection.cursor_value.queries == [
        'DELETE FROM cargos WHERE name = %s',
    ]
    assert connection.cursor_value.params == ('컨테이너_C6',)
    assert connection.committed
    assert not connection.rolled_back
    assert connection.cursor_value.closed
    assert connection.closed


def test_snapshot_id_changes_only_when_row_content_changes():
    client = InventoryClient(connect=lambda **kwargs: None, now=lambda: NOW)

    first = client.snapshot_from_rows(ROWS)
    same = client.snapshot_from_rows(list(ROWS))
    changed = client.snapshot_from_rows([
        ('컨테이너_C0', 'B-1', '0', '컨테이너', '', '11', 1),
    ])

    assert first.snapshot_id == same.snapshot_id
    assert first.snapshot_id != changed.snapshot_id


@pytest.mark.parametrize(
    'rows, message',
    [
        ([('C0', 'A-1-1')], 'exactly 7 columns'),
        ([('C0', '', '0', '', '', '', 1)], 'empty name or location'),
        ([('C0', 'A-1-1', '0', '', '', '', 0)], 'positive integer'),
        (
            [
                ('C0', 'A-1-1', '0', '', '', '', 1),
                ('C0', 'A-2-1', '1', '', '', '', 1),
            ],
            'duplicate cargo name',
        ),
        (
            [
                ('C0', 'A-1-1', '0', '', '', '', 1),
                ('C1', 'A-2-1', '0', '', '', '', 1),
            ],
            'duplicate container_id',
        ),
    ],
)
def test_sql_row_validation_rejects_bad_data(rows, message):
    client = InventoryClient(connect=lambda **kwargs: None, now=lambda: NOW)

    with pytest.raises(InventoryClientError, match=message):
        client.snapshot_from_rows(rows)
