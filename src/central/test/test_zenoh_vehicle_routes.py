"""Keep central and remote vehicle Zenoh routes symmetrical."""

import json
from pathlib import Path
import re


ROOT = Path(__file__).resolve().parents[3]


def _routes(filename, route_kind):
    config = json.loads(
        (ROOT / 'config' / 'network' / filename).read_text(encoding='utf-8')
    )
    return tuple(config['plugins']['ros2dds']['allow'][route_kind])


def _is_allowed(route, patterns):
    return any(re.fullmatch(pattern, route) for pattern in patterns)


def test_vehicle_bridge_routes_match_central_bridge_routes():
    central_publishers = _routes('zenoh_central.json5', 'publishers')
    central_subscribers = _routes('zenoh_central.json5', 'subscribers')
    central_service_clients = _routes(
        'zenoh_central.json5', 'service_clients'
    )
    central_action_clients = _routes('zenoh_central.json5', 'action_clients')

    for vehicle_number in (1, 2):
        filename = f'zenoh_agv{vehicle_number}.json5'
        peer_pose = f'/agv{3 - vehicle_number}/shared_amcl_pose'
        own_pose = f'/agv{vehicle_number}/shared_amcl_pose'

        for route in _routes(filename, 'subscribers'):
            if route != peer_pose:
                assert _is_allowed(route, central_publishers), route
        for route in _routes(filename, 'publishers'):
            if route != own_pose:
                assert _is_allowed(route, central_subscribers), route
        for route in _routes(filename, 'service_servers'):
            assert _is_allowed(route, central_service_clients), route
        for route in _routes(filename, 'action_servers'):
            assert _is_allowed(route, central_action_clients), route


def test_direct_vehicle_motion_has_manual_and_priority_routes():
    central_publishers = _routes('zenoh_central.json5', 'publishers')

    for vehicle_number in (1, 2):
        subscribers = _routes(
            f'zenoh_agv{vehicle_number}.json5', 'subscribers'
        )
        for suffix in ('manual', 'parking'):
            route = f'/agv{vehicle_number}/cmd_vel_{suffix}'
            assert route in subscribers
            assert _is_allowed(route, central_publishers)
