import importlib.machinery
import importlib.util
from pathlib import Path

import pytest


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'other_robot_obstacle'
loader = importlib.machinery.SourceFileLoader('other_robot_obstacle', str(SCRIPT))
spec = importlib.util.spec_from_loader(loader.name, loader)
obstacle = importlib.util.module_from_spec(spec)
loader.exec_module(obstacle)


def test_pose_distance_detects_peer_movement():
    assert obstacle.pose_distance((0.0, 0.0), (0.03, 0.04)) == pytest.approx(0.05)


def test_previous_disk_and_outer_ring_are_both_used_for_clearing():
    source = Path(SCRIPT).read_text(encoding='utf-8')

    assert 'list(self.obstacle_offsets)' in source
    assert 'self._make_ring_offsets(' in source
    assert 'self._clear_old_pose_via_services(' in source
