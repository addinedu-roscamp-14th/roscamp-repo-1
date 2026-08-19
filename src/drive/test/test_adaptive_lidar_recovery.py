import importlib.machinery
import importlib.util
import inspect
import math
from pathlib import Path
from types import SimpleNamespace
import xml.etree.ElementTree as ET

import pytest
import yaml


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'adaptive_lidar_recovery'
loader = importlib.machinery.SourceFileLoader('adaptive_recovery', str(SCRIPT))
spec = importlib.util.spec_from_loader(loader.name, loader)
recovery = importlib.util.module_from_spec(spec)
loader.exec_module(recovery)


def scan(default=0.10):
    return SimpleNamespace(
        ranges=[default] * 361,
        angle_min=-math.pi,
        angle_increment=math.radians(1.0),
        range_min=0.02,
        range_max=3.0,
    )


def costmap(width=10, height=10, resolution=0.1):
    origin = SimpleNamespace(
        position=SimpleNamespace(x=0.0, y=0.0),
        orientation=SimpleNamespace(z=0.0, w=1.0),
    )
    return SimpleNamespace(
        info=SimpleNamespace(
            width=width,
            height=height,
            resolution=resolution,
            origin=origin,
        ),
        data=[0] * (width * height),
    )


def test_footprint_collision_detects_lethal_costmap_cell():
    grid = costmap()
    grid.data[2 * grid.info.width + 2] = 100
    footprint = [(0.20, 0.20), (0.30, 0.20), (0.30, 0.30), (0.20, 0.30)]

    assert recovery.footprint_collision(grid, footprint)


def test_footprint_collision_allows_free_costmap_cells():
    grid = costmap()
    footprint = [(0.20, 0.20), (0.30, 0.20), (0.30, 0.30), (0.20, 0.30)]

    assert not recovery.footprint_collision(grid, footprint)


def test_footprint_outside_local_costmap_is_blocked():
    grid = costmap()
    footprint = [(-0.01, 0.20), (0.10, 0.20), (0.10, 0.30), (-0.01, 0.30)]

    assert recovery.footprint_collision(grid, footprint)


@pytest.mark.parametrize(
    ('rear_clearance', 'expected_distance'),
    [
        (0.01, 0.05),
        (0.03, 0.05),
        (0.05, 0.05),
        (0.099, 0.05),
        (0.10, 0.10),
        (0.30, 0.10),
    ],
)
def test_reverse_distance_uses_rear_lidar_clearance(
    rear_clearance, expected_distance,
):
    distance = recovery.select_reverse_distance(
        rear_clearance,
        short_distance_m=0.05,
        long_distance_m=0.10,
        long_clearance_threshold_m=0.10,
    )

    assert distance == expected_distance


@pytest.mark.parametrize(
    ('points', 'current_xy', 'expected_heading'),
    [
        ([(0.0, 0.0), (0.2, 0.0)], (0.01, 0.02), 0.0),
        ([(0.0, 0.0), (0.0, 0.2)], (0.01, 0.01), math.pi / 2.0),
        (
            [(0.0, 0.0), (0.1, 0.0), (0.2, 0.1)],
            (0.09, 0.0),
            math.pi / 4.0,
        ),
    ],
)
def test_path_rejoin_heading_uses_forward_path_tangent(
    points, current_xy, expected_heading,
):
    heading = recovery.path_rejoin_heading(
        points, current_xy, lookahead_m=0.10,
    )

    assert heading == pytest.approx(expected_heading)


@pytest.mark.parametrize(
    ('current', 'desired', 'remaining', 'expected'),
    [
        (0.0, 0.10, 1.0, 0.10),
        (0.0, -0.10, 1.0, -0.10),
        (0.0, 1.0, 1.0, 0.30),
        (0.0, math.radians(1.0), 1.0, 0.0),
    ],
)
def test_heading_alignment_rate_reduces_path_heading_error(
    current, desired, remaining, expected,
):
    rate = recovery.heading_alignment_rate(
        current,
        desired,
        remaining,
        maximum_rate=0.30,
        deadband_rad=math.radians(2.0),
    )

    assert rate == pytest.approx(expected)


@pytest.mark.parametrize('filename', [
    'navigate_to_pose_central_recovery.xml',
    'navigate_through_poses_central_recovery.xml',
])
def test_nav2_tree_repeats_recovery_until_success_or_cancel(filename):
    tree_path = Path(__file__).parents[1] / 'behavior_trees' / filename
    root = ET.parse(tree_path).getroot()
    retry = root.find('.//RetryUntilSuccessful')

    assert retry is not None
    assert retry.attrib['num_attempts'] == '-1'
    recovery_steps = root.findall('.//AdaptiveLidarRecovery')
    assert len(recovery_steps) == 1
    recover_sequence = root.find('.//Sequence[@name="RecoverThenRetry"]')
    assert recover_sequence is not None
    assert [child.tag for child in recover_sequence] == [
        'AdaptiveLidarRecovery', 'AlwaysFailure'
    ]
    assert not root.findall('.//RecoveryNode')
    assert not root.findall('.//Spin')
    assert not root.findall('.//BackUp')


def test_each_recovery_reverses_then_clears_for_replanning():
    source = inspect.getsource(recovery.AdaptiveLidarRecoveryNode._recover)

    reverse_index = source.index('self._reverse(deadline)')
    clear_index = source.index('self._clear_rebuild_costmap()')

    assert reverse_index < clear_index
    assert 'self._turn(' not in source
    assert 'self._select_target(' not in source


def test_controller_waits_longer_than_global_replanning_period():
    """A transient collision must not trigger another reverse before replan."""
    params_path = Path(__file__).parents[1] / 'params' / 'nav2_params.yaml'
    with params_path.open(encoding='utf-8') as stream:
        params = yaml.safe_load(stream)
    tolerance = params['controller_server']['ros__parameters'][
        'failure_tolerance'
    ]

    # The navigation BT replans at 1 Hz, so one complete replan interval plus
    # scheduling margin must fit inside the controller failure tolerance.
    assert tolerance >= 1.5


def test_reverse_is_not_skipped_by_stale_costmap():
    source = inspect.getsource(recovery.AdaptiveLidarRecoveryNode._reverse)

    assert 'self._cmd_pub.publish(command)' in source
    assert 'self._predicted_pose_is_safe' not in source
    assert 'self.reverse_stop_clearance' in source
    assert 'no fresh LiDAR scan for safe recovery reverse' in source
    assert 'command.angular.z = steering' in source
    assert 'heading_alignment_rate(' in source
    assert 'select_narrow_reverse_steering' not in source


def test_recovery_clears_both_costmaps():
    source = inspect.getsource(
        recovery.AdaptiveLidarRecoveryNode._clear_costmaps)

    assert 'self._clear_local_client' in source
    assert 'self._clear_global_client' in source


def test_recovery_waits_for_fresh_local_and_global_costmap_grids():
    source = inspect.getsource(
        recovery.AdaptiveLidarRecoveryNode._clear_rebuild_costmap)

    assert 'self._costmap_received_at > cleared_at' in source
    assert 'self._global_costmap_received_at > cleared_at' in source
    assert 'Costmap rebuild timed out' in source
