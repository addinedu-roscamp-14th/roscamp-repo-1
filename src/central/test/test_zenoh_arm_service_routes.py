"""Keep central and remote ARM Zenoh service routes symmetrical."""

import json
from pathlib import Path


ROOT = Path(__file__).resolve().parents[3]


def _service_routes(filename, route_kind):
    config = json.loads(
        (ROOT / 'config' / 'network' / filename).read_text(encoding='utf-8')
    )
    return set(config['plugins']['ros2dds']['allow'][route_kind])


def test_arm1_service_servers_are_allowed_as_central_clients():
    central = _service_routes('zenoh_central.json5', 'service_clients')
    arm1 = _service_routes('zenoh_arm1.json5', 'service_servers')

    assert arm1 <= central


def test_arm2_service_servers_are_allowed_as_central_clients():
    central = _service_routes('zenoh_central.json5', 'service_clients')
    arm2 = _service_routes('zenoh_arm2.json5', 'service_servers')

    assert arm2 <= central
    assert '/arm2/transfer_to_slot' in arm2
