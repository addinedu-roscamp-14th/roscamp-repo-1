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
    # AMR1 keeps the calibrated node default (0.658096 m). A recent 0.58 m
    # override made the timed reverse stop roughly 7.8 cm too early.
    assert 'reverse_distance_m' not in spots['park_red']
    assert spots['parking_yellow']['reverse_distance_m'] == 0.451511


def test_parking_maneuver_bypass_keeps_legacy_final_approach_fallback():
    source = (ROOT / 'scripts' / 'parking_new').read_text(encoding='utf-8')

    assert "'parking_maneuver_ignore_costmap'" in source
    assert "spot.get('final_approach_ignore_costmap', False)" in source
    assert 'cmd_vel safety gate remains active' in source


def test_parking_velocity_uses_a_dedicated_priority_topic():
    vehicle_launch = (
        ROOT.parent / 'porter_bringup' / 'launch' / 'agv_vehicle.launch.py'
    ).read_text(encoding='utf-8')
    drive_launch = (
        ROOT / 'launch' / 'multi_vehicle_nav.launch.py'
    ).read_text(encoding='utf-8')
    safety_gate = (
        ROOT.parent / 'pinky' / 'pinky_nodes' / 'cmd_vel_safety_gate.py'
    ).read_text(encoding='utf-8')

    assert "'cmd_vel_topic': 'cmd_vel_parking'" in vehicle_launch
    assert "'cmd_vel_topic': 'cmd_vel_parking'" in drive_launch
    assert "'parking_input_topic', 'cmd_vel_parking'" in safety_gate
    assert 'parking_age <= self._parking_timeout' in safety_gate


def test_parking_velocity_topic_is_allowed_across_zenoh_bridges():
    network = ROOT.parents[1] / 'config' / 'network'
    central = (network / 'zenoh_central.json5').read_text(encoding='utf-8')
    agv1 = (network / 'zenoh_agv1.json5').read_text(encoding='utf-8')
    agv2 = (network / 'zenoh_agv2.json5').read_text(encoding='utf-8')

    assert '"/agv[12]/cmd_vel_parking"' in central
    assert '"/agv1/cmd_vel_parking"' in agv1
    assert '"/agv2/cmd_vel_parking"' in agv2


def test_parking_always_restores_strict_nav2_yaw_tolerance():
    source = (ROOT / 'scripts' / 'parking_new').read_text(encoding='utf-8')

    execute_wrapper = source.split(
        '    def execute_callback(self, goal_handle):', 1
    )[1].split('    def _execute_parking(self, goal_handle):', 1)[0]
    assert 'try:' in execute_wrapper
    assert 'finally:' in execute_wrapper
    assert 'self.strict_yaw_goal_tolerance' in execute_wrapper
    assert 'self.set_yaw_goal_tolerance(' in execute_wrapper


def test_parking_success_requires_verified_final_xy_pose():
    source = (ROOT / 'scripts' / 'parking_new').read_text(encoding='utf-8')
    reverse_index = source.index(
        'if not self.reverse_to_parked(goal_handle, parked, reverse_distance):'
    )
    success_index = source.index('goal_handle.succeed()', reverse_index)
    verification = source[reverse_index:success_index]

    assert 'self.wait_for_pose(goal_handle, timeout_sec=2.0)' in verification
    assert 'self.distance_to_parked(parked)' in verification
    assert 'self.parked_xy_tolerance' in verification
