from pathlib import Path

import yaml


ROOT = Path(__file__).parents[1]


def test_only_amr1_parking_maneuver_bypasses_costmap():
    data = yaml.safe_load(
        (ROOT / 'params' / 'parking_spots.yaml').read_text(encoding='utf-8')
    )
    spots = data['parking_spots']

    assert spots['park_red']['parking_maneuver_ignore_costmap'] is True
    assert not spots['parking_yellow'].get(
        'parking_maneuver_ignore_costmap', False
    )


def test_parking_maneuver_bypass_keeps_legacy_final_approach_fallback():
    source = (ROOT / 'scripts' / 'parking_new').read_text(encoding='utf-8')

    assert "'parking_maneuver_ignore_costmap'" in source
    assert "spot.get('final_approach_ignore_costmap', False)" in source
    assert 'cmd_vel safety gate remains active' in source
