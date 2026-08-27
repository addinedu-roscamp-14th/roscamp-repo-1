"""Read container inventory directly from the PostgreSQL ``port_db``."""

from __future__ import annotations

import hashlib
import json
import os
from dataclasses import asdict, dataclass
from datetime import datetime, timezone
from typing import Callable, Tuple


SUPPORTED_SCHEMA_VERSION = '1.0'
DEFAULT_DB_HOST = '192.168.5.5'
DEFAULT_DB_PORT = 5432
DEFAULT_DB_NAME = 'port_db'
DEFAULT_DB_USER = 'postgres'
DEFAULT_DB_PASSWORD = '1234'


class InventoryClientError(RuntimeError):
    """Raised when the PostgreSQL inventory cannot be read safely."""


@dataclass(frozen=True)
class CargoRecord:
    """One normalized row from the remote ``cargos`` table."""

    name: str
    location: str
    container_id: str
    cargo_type: str
    note: str
    base_aruco_id: str
    floor: int

    def to_dict(self):
        """Return a JSON-serializable representation."""
        return asdict(self)


@dataclass(frozen=True)
class InventorySnapshot:
    """Immutable inventory state supplied to the LLM planner."""

    schema_version: str
    snapshot_id: str
    generated_at: str
    cargos: Tuple[CargoRecord, ...]

    def to_dict(self):
        """Return the stable JSON context supplied to the planner."""
        return {
            'schema_version': self.schema_version,
            'snapshot_id': self.snapshot_id,
            'generated_at': self.generated_at,
            'cargos': [cargo.to_dict() for cargo in self.cargos],
        }


def _positive_float(value, default):
    try:
        parsed = float(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0.0 else default


def _positive_int(value, default):
    try:
        parsed = int(value)
    except (TypeError, ValueError):
        return default
    return parsed if parsed > 0 else default


def _psycopg_connect(**kwargs):
    """Import psycopg2 lazily so configuration errors remain understandable."""
    try:
        import psycopg2
    except ImportError as exc:
        raise InventoryClientError(
            'psycopg2 is not installed; run pip install psycopg2-binary'
        ) from exc
    return psycopg2.connect(**kwargs)


class InventoryClient:
    """Query ``cargos`` in a read-only transaction without caching or writes."""

    SELECT_CARGOS = (
        'SELECT name, location, container_id, cargo_type, note, '
        'base_aruco_id, floor FROM cargos ORDER BY name'
    )

    def __init__(
        self,
        host=None,
        port=None,
        database=None,
        user=None,
        password=None,
        timeout_sec=None,
        connect: Callable | None = None,
        now: Callable[[], datetime] | None = None,
    ):
        self.host = str(
            host
            if host is not None
            else os.environ.get('PORT_INVENTORY_DB_HOST', DEFAULT_DB_HOST)
        ).strip()
        self.port = _positive_int(
            port
            if port is not None
            else os.environ.get('PORT_INVENTORY_DB_PORT', DEFAULT_DB_PORT),
            DEFAULT_DB_PORT,
        )
        self.database = str(
            database
            if database is not None
            else os.environ.get('PORT_INVENTORY_DB_NAME', DEFAULT_DB_NAME)
        ).strip()
        self.user = str(
            user
            if user is not None
            else os.environ.get('PORT_INVENTORY_DB_USER', DEFAULT_DB_USER)
        ).strip()
        self.password = str(
            password
            if password is not None
            else os.environ.get(
                'PORT_INVENTORY_DB_PASSWORD', DEFAULT_DB_PASSWORD
            )
        )
        self.timeout_sec = _positive_float(
            timeout_sec
            if timeout_sec is not None
            else os.environ.get('PORT_INVENTORY_DB_TIMEOUT_SEC', '3'),
            3.0,
        )
        self._connect = connect or _psycopg_connect
        self._now = now or (lambda: datetime.now(timezone.utc))

    def fetch_snapshot(self):
        """Read one current DB snapshot and fail closed on connection errors."""
        connection = None
        cursor = None
        try:
            connection = self._connect(
                host=self.host,
                port=self.port,
                dbname=self.database,
                user=self.user,
                password=self.password,
                connect_timeout=max(1, int(self.timeout_sec)),
            )
            connection.set_session(readonly=True, autocommit=False)
            cursor = connection.cursor()
            cursor.execute(self.SELECT_CARGOS)
            rows = cursor.fetchall()
            return self.snapshot_from_rows(rows)
        except InventoryClientError:
            raise
        except Exception as exc:
            raise InventoryClientError(
                f'PostgreSQL inventory query failed: {exc}'
            ) from exc
        finally:
            if cursor is not None:
                try:
                    cursor.close()
                except Exception:
                    pass
            if connection is not None:
                try:
                    connection.rollback()
                except Exception:
                    pass
                try:
                    connection.close()
                except Exception:
                    pass

    def snapshot_from_rows(self, rows):
        """Validate SQL rows and derive a content-addressed snapshot ID."""
        if not isinstance(rows, (list, tuple)):
            raise InventoryClientError('cargos query result must be a row sequence')
        cargos = []
        names = set()
        container_ids = set()
        for index, row in enumerate(rows):
            if not isinstance(row, (list, tuple)) or len(row) != 7:
                raise InventoryClientError(
                    f'cargos row {index} must contain exactly 7 columns'
                )
            name, location, container_id, cargo_type, note, base_aruco_id, floor = row
            name = str(name or '').strip()
            location = str(location or '').strip()
            container_id = str(container_id or '').strip()
            cargo_type = str(cargo_type or '').strip()
            note = str(note or '').strip()
            base_aruco_id = str(base_aruco_id or '').strip()
            if not name or not location:
                raise InventoryClientError(
                    f'cargos row {index} has an empty name or location'
                )
            if isinstance(floor, bool) or not isinstance(floor, int) or floor < 1:
                raise InventoryClientError(
                    f'cargos row {index} floor must be a positive integer'
                )
            if name in names:
                raise InventoryClientError(f'duplicate cargo name: {name}')
            names.add(name)
            if container_id:
                if container_id in container_ids:
                    raise InventoryClientError(
                        f'duplicate container_id: {container_id}'
                    )
                container_ids.add(container_id)
            cargos.append(CargoRecord(
                name=name,
                location=location,
                container_id=container_id,
                cargo_type=cargo_type,
                note=note,
                base_aruco_id=base_aruco_id,
                floor=floor,
            ))

        serialized = json.dumps(
            [cargo.to_dict() for cargo in cargos],
            ensure_ascii=False,
            sort_keys=True,
            separators=(',', ':'),
        )
        digest = hashlib.sha256(serialized.encode('utf-8')).hexdigest()[:20]
        generated = self._now()
        if generated.tzinfo is None:
            generated = generated.replace(tzinfo=timezone.utc)
        generated_at = generated.astimezone(timezone.utc).isoformat().replace(
            '+00:00', 'Z'
        )
        return InventorySnapshot(
            schema_version=SUPPORTED_SCHEMA_VERSION,
            snapshot_id=f'sql-{digest}',
            generated_at=generated_at,
            cargos=tuple(cargos),
        )


class InventoryAdminClient:
    """Explicit write-capable client for operator-approved inventory resets."""

    def __init__(self, inventory_client=None, connect=None):
        config = inventory_client or InventoryClient(connect=connect)
        self.host = config.host
        self.port = config.port
        self.database = config.database
        self.user = config.user
        self.password = config.password
        self.timeout_sec = config.timeout_sec
        self._connect = connect or config._connect

    def clear_inventory(self):
        """Atomically remove movement history and current cargo state."""
        connection = None
        cursor = None
        try:
            connection = self._connect(
                host=self.host,
                port=self.port,
                dbname=self.database,
                user=self.user,
                password=self.password,
                connect_timeout=max(1, int(self.timeout_sec)),
            )
            cursor = connection.cursor()
            cursor.execute('TRUNCATE TABLE cargo_movements')
            cursor.execute('DELETE FROM cargos')
            connection.commit()
        except Exception as exc:
            if connection is not None:
                try:
                    connection.rollback()
                except Exception:
                    pass
            raise InventoryClientError(
                f'PostgreSQL inventory reset failed: {exc}'
            ) from exc
        finally:
            if cursor is not None:
                try:
                    cursor.close()
                except Exception:
                    pass
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass

    def upsert_cargo(
        self,
        name,
        location,
        container_id='',
        cargo_type='',
        note='',
        base_aruco_id='',
        floor=1,
    ):
        """Insert or update a single cargo row in PostgreSQL.

        Returns True on success, False on failure (with the error logged).
        """
        connection = None
        cursor = None
        try:
            connection = self._connect(
                host=self.host,
                port=self.port,
                dbname=self.database,
                user=self.user,
                password=self.password,
                connect_timeout=max(1, int(self.timeout_sec)),
            )
            cursor = connection.cursor()
            cursor.execute(
                """
                INSERT INTO cargos
                    (name, location, container_id, cargo_type, note,
                     base_aruco_id, floor)
                VALUES (%s, %s, %s, %s, %s, %s, %s)
                ON CONFLICT (name) DO UPDATE SET
                    location = EXCLUDED.location,
                    container_id = EXCLUDED.container_id,
                    cargo_type = EXCLUDED.cargo_type,
                    note = EXCLUDED.note,
                    base_aruco_id = EXCLUDED.base_aruco_id,
                    floor = EXCLUDED.floor
                """,
                (
                    str(name), str(location), str(container_id),
                    str(cargo_type), str(note), str(base_aruco_id),
                    int(floor) if floor else 1,
                ),
            )
            connection.commit()
            return True
        except Exception as exc:
            if connection is not None:
                try:
                    connection.rollback()
                except Exception:
                    pass
            raise InventoryClientError(
                f'PostgreSQL cargo upsert failed: {exc}'
            ) from exc
        finally:
            if cursor is not None:
                try:
                    cursor.close()
                except Exception:
                    pass
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass

    def delete_cargo(self, name):
        """Delete a single cargo row from PostgreSQL by name."""
        connection = None
        cursor = None
        try:
            connection = self._connect(
                host=self.host,
                port=self.port,
                dbname=self.database,
                user=self.user,
                password=self.password,
                connect_timeout=max(1, int(self.timeout_sec)),
            )
            cursor = connection.cursor()
            cursor.execute(
                'DELETE FROM cargos WHERE name = %s', (str(name),)
            )
            connection.commit()
            return True
        except Exception as exc:
            if connection is not None:
                try:
                    connection.rollback()
                except Exception:
                    pass
            raise InventoryClientError(
                f'PostgreSQL cargo delete failed: {exc}'
            ) from exc
        finally:
            if cursor is not None:
                try:
                    cursor.close()
                except Exception:
                    pass
            if connection is not None:
                try:
                    connection.close()
                except Exception:
                    pass
