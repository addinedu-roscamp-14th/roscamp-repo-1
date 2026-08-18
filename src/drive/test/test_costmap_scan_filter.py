import importlib.machinery
import importlib.util
import math
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'costmap_scan_filter'
loader = importlib.machinery.SourceFileLoader('costmap_scan_filter', str(SCRIPT))
spec = importlib.util.spec_from_loader(loader.name, loader)
scan_filter = importlib.util.module_from_spec(spec)
loader.exec_module(scan_filter)


def apply_filter(ranges, previous=None):
    return scan_filter.filter_isolated_ranges(
        ranges,
        previous,
        range_min=0.05,
        range_max=40.0,
        neighbor_window=2,
        neighbor_delta_m=0.08,
        temporal_window=1,
        temporal_delta_m=0.10,
    )


def test_single_frame_isolated_return_is_removed():
    filtered, removed = apply_filter([math.inf, math.inf, 0.7, math.inf])

    assert math.isinf(filtered[2])
    assert removed == 1


def test_neighboring_cluster_is_preserved():
    filtered, removed = apply_filter([math.inf, 0.70, 0.72, math.inf])

    assert filtered[1:3] == [0.70, 0.72]
    assert removed == 0


def test_temporally_repeated_thin_obstacle_is_preserved():
    current = [math.inf, math.inf, 0.72, math.inf]
    previous = [math.inf, math.inf, 0.70, math.inf]

    filtered, removed = apply_filter(current, previous)

    assert filtered[2] == 0.72
    assert removed == 0


def test_invalid_ranges_remain_available_for_raytrace_clearing():
    filtered, removed = apply_filter([math.inf, math.nan])

    assert math.isinf(filtered[0])
    assert math.isnan(filtered[1])
    assert removed == 0


def test_lidar_returns_on_peer_vehicle_are_removed():
    ranges = [math.inf, 1.0, math.inf]

    filtered, removed = scan_filter.filter_peer_ranges(
        ranges,
        angle_min=-0.1,
        angle_increment=0.1,
        peer_center=(1.0, 0.0),
        peer_radius_m=0.22,
    )

    assert math.isinf(filtered[1])
    assert removed == 1


def test_obstacle_away_from_peer_vehicle_is_preserved():
    ranges = [math.inf, 0.5, math.inf]

    filtered, removed = scan_filter.filter_peer_ranges(
        ranges,
        angle_min=-0.1,
        angle_increment=0.1,
        peer_center=(1.0, 0.0),
        peer_radius_m=0.22,
    )

    assert filtered[1] == 0.5
    assert removed == 0
