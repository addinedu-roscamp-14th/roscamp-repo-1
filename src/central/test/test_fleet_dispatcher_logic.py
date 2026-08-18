import json
import math
import threading
import time

from central.fleet_dispatcher import (
    choose_lidar_escape_turn,
    classify_motion_stall,
    FleetDispatcher,
    laser_sector_clearance,
    lidar_free_space_center,
    STALL_EXHAUSTED,
    STALL_HELD,
    STALL_MOVING,
    STALL_RESEND,
    VehicleRuntime,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import pytest
from std_msgs.msg import String


class ReadyActionClient:
    def server_is_ready(self):
        return True


class FakeExecutor:
    def __init__(self):
        self.tasks = []

    def create_task(self, coro):
        self.tasks.append(coro)
        coro.close()  # avoid "coroutine was never awaited" warnings


class PermissiveLogger:
    def warning(self, _message, **_kwargs):
        pass

    def info(self, _message, **_kwargs):
        pass

    def error(self, _message, **_kwargs):
        pass


def make_dispatcher():
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher._lock = threading.RLock()
    dispatcher._vehicle_condition = threading.Condition(dispatcher._lock)
    dispatcher._vehicle_queue = {
        vehicle_id: [] for vehicle_id in ('agv1', 'agv2')
    }
    dispatcher._preempted_commands = set()
    dispatcher._command_condition = threading.Condition(dispatcher._lock)
    dispatcher._command_outcomes = {}
    dispatcher._zone_condition = threading.Condition(dispatcher._lock)
    dispatcher._zone_owner = {}
    dispatcher.exclusive_zone_ids = ()
    dispatcher.sequence_dependency_timeout_sec = 1.0
    dispatcher.duplicate_goal_distance_m = 0.12
    dispatcher.duplicate_goal_yaw_tolerance_rad = math.radians(20.0)
    dispatcher.subscribe_odom_fallback = True
    dispatcher.motion_threshold_mps = 0.015
    dispatcher.motion_yaw_threshold_rps = 0.05
    dispatcher.motion_stall_timeout_sec = 8.0
    dispatcher.max_motion_resends = 2
    dispatcher._collision_held_vehicle = ''
    dispatcher.max_park_retries = 2
    dispatcher.park_retry_backoff_sec = 0.0
    dispatcher.vehicles = {
        vehicle_id: VehicleRuntime(vehicle_id)
        for vehicle_id in ('agv1', 'agv2')
    }
    dispatcher._vehicle_ready = lambda vehicle_id, _waypoints=False: (
        not dispatcher.vehicles[vehicle_id].busy
    )
    dispatcher._vehicle_operational = (
        lambda _vehicle_id, _waypoints=False: True
    )
    # FleetDispatcher.executor is a real rclpy.Node property backed by a
    # name-mangled weakref slot; set that slot directly so the getter works
    # without routing through the setter's live-executor bookkeeping.
    fake_executor = FakeExecutor()
    dispatcher._Node__executor_weakref = lambda: fake_executor
    dispatcher._idle_since = {vehicle_id: None for vehicle_id in ('agv1', 'agv2')}
    dispatcher.auto_park_idle_sec = 20.0
    dispatcher.park_zone_ids = {'agv1': 'PARK1', 'agv2': 'PARK2'}
    dispatcher.park_zone_map_xy = {
        'agv1': (1.0, 0.0),
        'agv2': (2.0, 0.0),
    }
    dispatcher.park_exit_detection_radius_m = 0.25
    dispatcher.get_logger = lambda: PermissiveLogger()
    return dispatcher


def target_pose(x, y):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


def synthetic_scan(default=2.0):
    """Return a one-degree 360-degree scan tuple for recovery tests."""
    ranges = [default] * 360
    return ranges, -math.pi, math.radians(1.0), 0.05, 3.5


def test_adaptive_recovery_all_applies_to_both_vehicles():
    dispatcher = make_dispatcher()
    dispatcher.adaptive_recovery_vehicle_id = 'all'

    assert dispatcher._adaptive_recovery_applies('agv1')
    assert dispatcher._adaptive_recovery_applies('agv2')


def block_sector(ranges, center_deg, width_deg, distance):
    for index in range(len(ranges)):
        angle_deg = -180.0 + index
        delta = (angle_deg - center_deg + 180.0) % 360.0 - 180.0
        if abs(delta) <= width_deg * 0.5:
            ranges[index] = distance


def test_lidar_escape_turn_selects_roomier_right_swept_path():
    scan = synthetic_scan()
    ranges = scan[0]
    # A left turn sweeps front-left, so make that side narrow. Right-turn
    # front-right/rear-left sectors remain open.
    block_sector(ranges, 45.0, 50.0, 0.22)

    direction, left_score, right_score = choose_lidar_escape_turn(*scan)

    assert direction == -1.0
    assert right_score > left_score


def test_lidar_escape_turn_selects_roomier_left_swept_path():
    scan = synthetic_scan()
    ranges = scan[0]
    block_sector(ranges, -45.0, 50.0, 0.20)

    direction, left_score, right_score = choose_lidar_escape_turn(*scan)

    assert direction == 1.0
    assert left_score > right_score


def test_lidar_sector_clearance_uses_continuous_space_not_longest_ray():
    scan = synthetic_scan(default=0.30)
    ranges = scan[0]
    # One long return in a narrow sector must not make the corridor look open.
    ranges[180] = 3.0

    clearance = laser_sector_clearance(
        *scan, center_rad=0.0, width_rad=math.radians(40.0)
    )

    assert math.isclose(clearance, 0.30)


def test_lidar_free_space_turns_to_the_middle_of_the_opening():
    scan = synthetic_scan(default=0.10)
    ranges = scan[0]
    # Left opening spans 20..60 degrees, so its center is 40 degrees.
    for degree in range(20, 61):
        ranges[180 + degree] = 2.0

    gap = lidar_free_space_center(
        *scan,
        side=1.0,
        minimum_clearance_m=0.18,
    )

    assert gap is not None
    assert math.degrees(gap[0]) == pytest.approx(40.0, abs=2.6)
    assert math.degrees(gap[1]) == pytest.approx(40.0, abs=5.1)


def test_lidar_free_space_rejects_a_gap_that_is_too_narrow():
    scan = synthetic_scan(default=0.10)
    ranges = scan[0]
    for degree in range(-20, -11):
        ranges[180 + degree] = 2.0

    gap = lidar_free_space_center(
        *scan,
        side=-1.0,
        minimum_clearance_m=0.18,
        minimum_gap_width_rad=math.radians(15.0),
    )

    assert gap is None


def test_auto_selects_nearest_vehicle_and_reserves_it():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].pose.pose.position.x = 0.0
    dispatcher.vehicles['agv2'].pose.pose.position.x = 2.0

    selected = dispatcher._select_and_reserve_vehicle(
        '',
        target_pose(1.8, 0.0),
        'command-1',
    )

    assert selected == 'agv2'
    assert dispatcher.vehicles['agv2'].busy
    assert dispatcher.vehicles['agv2'].current_command_id == 'command-1'


def test_auto_tie_prefers_agv1():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].pose.pose.position.x = -1.0
    dispatcher.vehicles['agv2'].pose.pose.position.x = 1.0

    selected = dispatcher._select_and_reserve_vehicle(
        '',
        target_pose(0.0, 0.0),
        'command-2',
    )

    assert selected == 'agv1'


def test_explicit_busy_vehicle_is_not_reassigned():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv2'].busy = True

    selected = dispatcher._select_and_reserve_vehicle(
        'agv2',
        target_pose(0.0, 0.0),
        'command-3',
    )

    assert selected == ''
    assert not dispatcher.vehicles['agv1'].busy


def test_explicit_busy_vehicle_command_waits_in_fifo_queue():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].busy = True
    reserved = {}

    def reserve():
        reserved['result'] = dispatcher._wait_and_reserve_explicit_vehicle(
            _ImmediateGoalHandle(),
            'agv1',
            'queued-command',
        )

    worker = threading.Thread(target=reserve)
    worker.start()
    time.sleep(0.02)

    assert dispatcher._vehicle_queue['agv1'] == ['queued-command']
    with dispatcher._vehicle_condition:
        dispatcher.vehicles['agv1'].busy = False
        dispatcher._vehicle_condition.notify_all()
    worker.join(timeout=1.0)

    assert not worker.is_alive()
    assert reserved['result'] == ('agv1', 'reserved')
    assert dispatcher.vehicles['agv1'].busy


def test_new_immediate_command_preempts_active_and_queued_commands():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.busy = True
    runtime.current_command_id = 'active-command'
    dispatcher._vehicle_queue['agv1'] = ['queued-command']

    dispatcher._preempt_vehicle_commands('agv1')

    assert 'active-command' in dispatcher._preempted_commands
    assert 'queued-command' in dispatcher._preempted_commands
    assert dispatcher._vehicle_queue['agv1'] == []


def test_equivalent_active_goal_is_coalesced_for_same_vehicle():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.busy = True
    runtime.current_target_zone = 'A'
    runtime.current_target_pose = target_pose(0.20, -0.10)

    selected = dispatcher._find_equivalent_active_vehicle(
        'agv1', 'A', target_pose(0.25, -0.08)
    )

    assert selected == 'agv1'


def test_different_active_goal_is_not_coalesced():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.busy = True
    runtime.current_target_zone = 'B-1'
    runtime.current_target_pose = target_pose(0.20, -0.10)

    assert dispatcher._find_equivalent_active_vehicle(
        'agv1', 'A', target_pose(0.20, -0.10)
    ) == ''
    assert dispatcher._find_equivalent_active_vehicle(
        'agv1', 'B-1', target_pose(0.50, -0.10)
    ) == ''


def test_preempted_active_goal_is_not_coalesced():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.busy = True
    runtime.current_command_id = 'old-command'
    runtime.current_target_zone = 'A'
    runtime.current_target_pose = target_pose(0.20, -0.10)
    dispatcher._preempted_commands.add('old-command')

    assert dispatcher._find_equivalent_active_vehicle(
        'agv1', 'A', target_pose(0.20, -0.10)
    ) == ''


def test_new_b1_target_resets_exit_sequence_progress():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.b1_exit_turn_completed = True
    runtime.b1_exit_forward_completed = True

    dispatcher._set_active_target('agv1', 'B-1', target_pose(1.0, 0.0))

    assert not runtime.b1_exit_turn_completed
    assert not runtime.b1_exit_forward_completed


def test_auto_preemption_candidate_is_nearest_operational_vehicle():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].busy = True
    dispatcher.vehicles['agv1'].pose.pose.position.x = 0.2
    dispatcher.vehicles['agv2'].busy = True
    dispatcher.vehicles['agv2'].pose.pose.position.x = 2.0

    selected = dispatcher._select_preemption_candidate(
        target_pose(0.0, 0.0),
    )

    assert selected == 'agv1'


def test_dependent_plan_step_waits_for_successful_predecessor():
    dispatcher = make_dispatcher()
    completed = {}

    def wait_for_step():
        completed['result'] = dispatcher._wait_for_predecessor(
            _ImmediateGoalHandle(),
            'step-1',
        )

    worker = threading.Thread(target=wait_for_step)
    worker.start()
    time.sleep(0.02)
    assert worker.is_alive()

    dispatcher._record_command_outcome('step-1', True)
    worker.join(timeout=1.0)

    assert completed['result'] == (True, '')


def test_dependent_plan_step_stops_after_failed_predecessor():
    dispatcher = make_dispatcher()
    dispatcher._record_command_outcome('step-1', False)

    result = dispatcher._wait_for_predecessor(
        _ImmediateGoalHandle(),
        'step-1',
    )

    assert result == (False, 'predecessor_failed')


def test_auto_zone_request_excludes_current_zone_owner():
    dispatcher = make_zoned_dispatcher()
    dispatcher._zone_owner['B-1'] = 'agv1'
    dispatcher.vehicles['agv1'].pose.pose.position.x = 0.0
    dispatcher.vehicles['agv2'].pose.pose.position.x = 2.0

    selected = dispatcher._select_and_reserve_vehicle(
        '',
        target_pose(0.1, 0.0),
        'command-zone',
        target_zone_id='B-1',
    )

    assert selected == 'agv2'


def test_odom_updates_until_amcl_is_available():
    dispatcher = make_dispatcher()
    first = Odometry()
    first.pose.pose.position.x = 1.0
    second = Odometry()
    second.pose.pose.position.x = 2.0

    dispatcher._on_odom('agv1', first)
    dispatcher._on_odom('agv1', second)

    runtime = dispatcher.vehicles['agv1']
    assert math.isclose(runtime.pose.pose.position.x, 2.0)
    assert not runtime.has_amcl_pose
    assert runtime.telemetry_received_at is not None


def test_odom_keeps_vehicle_online_without_overwriting_amcl_pose():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.has_amcl_pose = True
    runtime.pose.pose.position.x = 1.0
    runtime.telemetry_received_at = time.monotonic() - 10.0
    odom = Odometry()
    odom.pose.pose.position.x = 9.0

    dispatcher._on_odom('agv1', odom)

    assert math.isclose(runtime.pose.pose.position.x, 1.0)
    assert time.monotonic() - runtime.telemetry_received_at < 1.0


def test_vehicle_is_not_dispatchable_before_initial_pose():
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher.telemetry_timeout = 3.0
    runtime = VehicleRuntime('agv1')
    runtime.pose_received_at = time.monotonic()
    runtime.telemetry_received_at = time.monotonic()
    dispatcher.vehicles = {'agv1': runtime}
    dispatcher.nav_pose_clients = {'agv1': ReadyActionClient()}
    dispatcher.nav_waypoint_clients = {'agv1': ReadyActionClient()}

    assert not dispatcher._vehicle_ready('agv1')
    runtime.has_amcl_pose = True
    assert dispatcher._vehicle_ready('agv1')


class _NullLogger:
    def warning(self, _message):
        pass

    def info(self, _message):
        pass


def make_zoned_dispatcher():
    dispatcher = make_dispatcher()
    dispatcher._zone_condition = threading.Condition(dispatcher._lock)
    dispatcher.exclusive_zone_ids = ('B-1', 'A')
    dispatcher._zone_owner = {
        zone_id: '' for zone_id in dispatcher.exclusive_zone_ids
    }
    dispatcher._zone_unknown = {
        zone_id: False for zone_id in dispatcher.exclusive_zone_ids
    }
    dispatcher._zone_queue = {
        zone_id: [] for zone_id in dispatcher.exclusive_zone_ids
    }
    dispatcher._zone_target_poses = {}
    dispatcher._startup_zone_recovery_pending = set()
    dispatcher._zone_entered = {
        zone_id: False for zone_id in dispatcher.exclusive_zone_ids
    }
    dispatcher.telemetry_timeout = 3.0
    dispatcher.zone_occupancy_radius_m = 0.18
    dispatcher.zone_release_hysteresis_m = 0.05
    dispatcher.b1_exit_detection_radius_m = 0.35
    dispatcher.get_logger = lambda: _NullLogger()
    return dispatcher


class _ImmediateGoalHandle:
    is_cancel_requested = False


class _AlreadyCanceledGoalHandle:
    is_cancel_requested = True


def test_acquire_zone_reserves_and_release_frees_it():
    dispatcher = make_zoned_dispatcher()
    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()

    acquired = dispatcher._acquire_zone(
        _ImmediateGoalHandle(), 'agv1', 'cmd-a', 'A'
    )

    assert acquired
    assert dispatcher._zone_owner['A'] == 'agv1'
    assert dispatcher.vehicles['agv1'].locked_zone == 'A'
    assert dispatcher._zone_owner['B-1'] == ''  # other zone is unaffected

    dispatcher._release_zone('agv1', 'A')

    assert dispatcher._zone_owner['A'] == ''
    assert dispatcher.vehicles['agv1'].locked_zone == ''


def test_second_vehicle_is_rejected_while_zone_is_owned():
    dispatcher = make_zoned_dispatcher()
    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()

    assert dispatcher._acquire_zone(
        _ImmediateGoalHandle(), 'agv1', 'cmd-1', 'A'
    )

    acquired = dispatcher._acquire_zone(
        _AlreadyCanceledGoalHandle(), 'agv2', 'cmd-2', 'A'
    )

    assert not acquired
    assert dispatcher._zone_owner['A'] == 'agv1'
    assert dispatcher._zone_queue['A'] == []


def test_zone_request_queues_fifo_until_owner_leaves():
    dispatcher = make_zoned_dispatcher()
    dispatcher._zone_owner['B-1'] = 'agv1'
    dispatcher.vehicles['agv1'].locked_zone = 'B-1'

    state = dispatcher._queue_zone_request('agv2', 'cmd-wait', 'B-1')

    assert state == 'queued'
    assert dispatcher._zone_queue['B-1'] == ['cmd-wait']
    assert dispatcher._zone_owner['B-1'] == 'agv1'

    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()
    dispatcher._release_zone('agv1', 'B-1')
    acquired = dispatcher._wait_for_zone(
        _ImmediateGoalHandle(),
        'agv2',
        'cmd-wait',
        'B-1',
    )

    assert acquired
    assert dispatcher._zone_owner['B-1'] == 'agv2'
    assert dispatcher._zone_queue['B-1'] == []


def test_zone_owner_is_recovered_from_amcl_after_dispatcher_restart():
    dispatcher = make_zoned_dispatcher()
    now = time.monotonic()
    agv1 = dispatcher.vehicles['agv1']
    agv1.has_amcl_pose = True
    agv1.telemetry_received_at = now
    agv1.pose.pose.position.x = 1.05
    agv1.pose.pose.position.y = 2.02
    agv2 = dispatcher.vehicles['agv2']
    agv2.has_amcl_pose = True
    agv2.telemetry_received_at = now

    owner = dispatcher._infer_zone_owner_from_pose(
        'B-1',
        target_pose(1.0, 2.0),
    )

    assert owner == 'agv1'
    assert dispatcher._zone_owner['B-1'] == 'agv1'
    assert dispatcher.vehicles['agv1'].locked_zone == 'B-1'


def test_visually_empty_frame_does_not_clear_a_live_owner():
    dispatcher = make_zoned_dispatcher()
    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()
    dispatcher._acquire_zone(_ImmediateGoalHandle(), 'agv1', 'cmd-1', 'A')

    dispatcher._maybe_clear_stale_zone('A', True)

    assert dispatcher._zone_owner['A'] == 'agv1'
    assert dispatcher.vehicles['agv1'].locked_zone == 'A'


def test_visually_empty_frame_clears_an_offline_owners_stale_zone():
    dispatcher = make_zoned_dispatcher()
    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()
    dispatcher._acquire_zone(_ImmediateGoalHandle(), 'agv1', 'cmd-1', 'A')
    dispatcher._zone_unknown['A'] = True  # telemetry already flagged stale

    dispatcher._maybe_clear_stale_zone('A', True)

    assert dispatcher._zone_owner['A'] == ''
    assert dispatcher._zone_unknown['A'] is False
    assert dispatcher.vehicles['agv1'].locked_zone == ''


def test_stale_zone_is_not_cleared_without_visual_confirmation():
    dispatcher = make_zoned_dispatcher()
    dispatcher.vehicles['agv1'].telemetry_received_at = time.monotonic()
    dispatcher._acquire_zone(_ImmediateGoalHandle(), 'agv1', 'cmd-1', 'A')
    dispatcher._zone_unknown['A'] = True

    dispatcher._maybe_clear_stale_zone('A', False)

    assert dispatcher._zone_owner['A'] == 'agv1'
    assert dispatcher._zone_unknown['A'] is True


def test_nav_pose_uses_latest_tf_and_preserves_target():
    source = target_pose(1.2, -0.4)
    source.header.frame_id = 'map'
    source.header.stamp.sec = 123

    normalized = FleetDispatcher._latest_tf_pose(source)

    assert normalized.header.frame_id == 'map'
    assert normalized.header.stamp.sec == 0
    assert normalized.header.stamp.nanosec == 0
    assert math.isclose(normalized.pose.position.x, 1.2)
    assert math.isclose(normalized.pose.position.y, -0.4)


def test_b1_exit_requires_turn_only_when_leaving_b1():
    assert FleetDispatcher._requires_b1_exit_turn('B-1', 'A')
    assert FleetDispatcher._requires_b1_exit_turn('B-1', '')
    assert not FleetDispatcher._requires_b1_exit_turn('B-1', 'B-1')
    assert not FleetDispatcher._requires_b1_exit_turn('A', 'B-1')


def test_b1_exit_turn_pose_rotates_left_90_without_translation():
    current = target_pose(0.42, -0.17)
    current.header.frame_id = 'map'
    yaw = math.radians(30.0)
    current.pose.orientation.z = math.sin(yaw * 0.5)
    current.pose.orientation.w = math.cos(yaw * 0.5)

    target = FleetDispatcher._b1_exit_turn_pose(current, 90.0)
    target_yaw = 2.0 * math.atan2(
        target.pose.orientation.z,
        target.pose.orientation.w,
    )

    assert math.isclose(target.pose.position.x, 0.42)
    assert math.isclose(target.pose.position.y, -0.17)
    assert math.isclose(math.degrees(target_yaw), 120.0)


def test_b1_exit_forward_pose_advances_along_turned_heading():
    turned = target_pose(0.42, -0.17)
    yaw = math.radians(90.0)
    turned.pose.orientation.z = math.sin(yaw * 0.5)
    turned.pose.orientation.w = math.cos(yaw * 0.5)

    target = FleetDispatcher._forward_pose(turned, 0.10)

    assert math.isclose(target.pose.position.x, 0.42, abs_tol=1e-9)
    assert math.isclose(target.pose.position.y, -0.07, abs_tol=1e-9)
    assert target.pose.orientation == turned.pose.orientation


def test_b1_reserved_vehicle_does_not_turn_until_it_reaches_zone():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.pose.pose.position.x = 0.0
    dispatcher._zone_target_poses['B-1'] = target_pose(1.0, 0.0)

    assert not dispatcher._vehicle_is_at_zone(runtime, 'B-1')
    runtime.pose.pose.position.x = 0.9
    assert dispatcher._vehicle_is_at_zone(runtime, 'B-1')


def test_b1_exit_maneuver_survives_early_lock_release_near_zone():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    dispatcher._zone_target_poses['B-1'] = target_pose(1.0, 0.0)
    runtime.pose.pose.position.x = 0.72

    assert runtime.locked_zone == ''
    assert dispatcher._requires_b1_exit_maneuver(runtime, 'A')
    assert not dispatcher._requires_b1_exit_maneuver(runtime, 'B-1')


def test_b1_vehicle_triggers_exit_maneuver_without_a_memory_lock():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv2']
    dispatcher._zone_target_poses['B-1'] = target_pose(1.294, -0.087)
    runtime.pose.pose.position.x = 1.30
    runtime.pose.pose.position.y = -0.09

    assert runtime.locked_zone == ''
    assert dispatcher._requires_b1_exit_maneuver(runtime, 'A')


def test_startup_recovers_b1_owner_from_configured_reference():
    dispatcher = make_zoned_dispatcher()
    dispatcher._zone_target_poses['B-1'] = target_pose(1.294, -0.087)
    dispatcher._startup_zone_recovery_pending = {'B-1'}
    now = time.monotonic()
    for runtime in dispatcher.vehicles.values():
        runtime.has_amcl_pose = True
        runtime.telemetry_received_at = now
    dispatcher.vehicles['agv1'].pose.pose.position.x = 0.2
    dispatcher.vehicles['agv1'].pose.pose.position.y = 0.2
    dispatcher.vehicles['agv2'].pose.pose.position.x = 1.30
    dispatcher.vehicles['agv2'].pose.pose.position.y = -0.09

    dispatcher._recover_startup_zone_owners()

    assert dispatcher._zone_owner['B-1'] == 'agv2'
    assert dispatcher.vehicles['agv2'].locked_zone == 'B-1'
    assert 'B-1' not in dispatcher._startup_zone_recovery_pending


def test_b1_exit_maneuver_does_not_trigger_far_from_zone():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    dispatcher._zone_target_poses['B-1'] = target_pose(1.0, 0.0)
    runtime.pose.pose.position.x = 0.60

    assert not dispatcher._requires_b1_exit_maneuver(runtime, 'A')


def test_park_exit_is_required_when_leaving_own_locked_spot():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.locked_zone = 'PARK1'

    assert dispatcher._requires_park_exit_maneuver(runtime, 'A')
    assert not dispatcher._requires_park_exit_maneuver(runtime, 'PARK1')


def test_park_exit_is_recovered_from_pose_after_dispatcher_restart():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv2']
    dispatcher._zone_target_poses = {'PARK2': target_pose(2.0, 0.0)}
    runtime.pose.pose.position.x = 2.10

    assert dispatcher._requires_park_exit_maneuver(runtime, 'B-1')


def test_park_exit_runs_only_once_until_vehicle_parks_again():
    dispatcher = make_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.locked_zone = 'PARK1'
    runtime.park_exit_forward_completed = True

    assert not dispatcher._requires_park_exit_maneuver(runtime, 'A')


def test_a_zone_releases_as_soon_as_entered_vehicle_exits():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    runtime.has_amcl_pose = True
    runtime.telemetry_received_at = time.monotonic()
    runtime.locked_zone = 'A'
    dispatcher._zone_owner['A'] = 'agv1'
    dispatcher._zone_target_poses['A'] = target_pose(1.0, 2.0)
    runtime.pose.pose.position.x = 1.0
    runtime.pose.pose.position.y = 2.0

    dispatcher._refresh_zone_occupancy()
    assert dispatcher._zone_entered['A']
    assert dispatcher._zone_owner['A'] == 'agv1'

    runtime.pose.pose.position.x = 1.25
    dispatcher._refresh_zone_occupancy()

    assert dispatcher._zone_owner['A'] == ''
    assert runtime.locked_zone == ''


def test_zone_reservation_is_not_released_before_vehicle_enters():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv2']
    runtime.has_amcl_pose = True
    runtime.telemetry_received_at = time.monotonic()
    runtime.locked_zone = 'A'
    runtime.pose.pose.position.x = 0.0
    dispatcher._zone_owner['A'] = 'agv2'
    dispatcher._zone_target_poses['A'] = target_pose(1.0, 0.0)

    dispatcher._refresh_zone_occupancy()

    assert dispatcher._zone_owner['A'] == 'agv2'
    assert not dispatcher._zone_entered['A']


def test_auto_park_starts_the_idle_clock_without_triggering_immediately():
    dispatcher = make_dispatcher()

    dispatcher._check_auto_park()

    assert dispatcher._idle_since['agv1'] is not None
    assert dispatcher._idle_since['agv2'] is not None
    assert not dispatcher.executor.tasks


def test_auto_park_triggers_once_idle_threshold_is_exceeded():
    dispatcher = make_dispatcher()
    dispatcher._idle_since['agv1'] = time.monotonic() - 999.0

    dispatcher._check_auto_park()

    assert len(dispatcher.executor.tasks) == 1


def test_auto_park_never_triggers_for_a_busy_vehicle():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].busy = True
    dispatcher._idle_since['agv1'] = time.monotonic() - 999.0

    dispatcher._check_auto_park()

    assert dispatcher._idle_since['agv1'] is None
    assert not dispatcher.executor.tasks


def test_auto_park_never_retriggers_an_already_parked_vehicle():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv1'].locked_zone = 'PARK1'
    dispatcher._idle_since['agv1'] = time.monotonic() - 999.0

    dispatcher._check_auto_park()

    assert dispatcher._idle_since['agv1'] is None
    assert not dispatcher.executor.tasks


class NotReadyActionClient:
    def server_is_ready(self):
        return False

    def wait_for_server(self, timeout_sec=None):
        return False


class ParkingGoalHandle:
    accepted = True

    def __init__(self, success, message):
        self._success = success
        self._message = message

    async def get_result_async(self):
        result = type('ParkingResult', (), {
            'success': self._success,
            'message': self._message,
        })()
        return type('WrappedParkingResult', (), {'result': result})()


class SequencedParkingClient:
    def __init__(self, outcomes):
        self.outcomes = list(outcomes)

    def wait_for_server(self, timeout_sec=None):
        return True

    async def send_goal_async(self, _goal):
        success, message = self.outcomes.pop(0)
        return ParkingGoalHandle(success, message)


def make_park_dispatcher():
    dispatcher = make_dispatcher()
    dispatcher._adaptive_recovery_states = {}
    dispatcher._zone_owner = {'PARK1': '', 'PARK2': ''}
    dispatcher._zone_unknown = {'PARK1': False, 'PARK2': False}
    dispatcher._zone_queue = {'PARK1': [], 'PARK2': []}
    dispatcher._zone_entered = {'PARK1': False, 'PARK2': False}
    dispatcher._zone_target_poses = {}
    dispatcher.park_spot_ids = {'agv1': 'park_red', 'agv2': 'parking_yellow'}
    dispatcher.park_clients = {
        'agv1': NotReadyActionClient(),
        'agv2': NotReadyActionClient(),
    }
    dispatcher.park_action_wait_timeout_sec = 0.0
    return dispatcher


def _run_to_completion(coro):
    try:
        coro.send(None)
    except StopIteration:
        return
    raise AssertionError('coroutine suspended on an unexpected await')


def test_dispatch_park_reserves_the_vehicle_and_acquires_its_own_zone():
    dispatcher = make_park_dispatcher()

    _run_to_completion(dispatcher._dispatch_park('agv1'))

    # The action server was never ready, so the temporary reservation and
    # zone acquisition must both be cleaned up for the next request.
    assert dispatcher.vehicles['agv1'].locked_zone == ''
    assert dispatcher.vehicles['agv1'].busy is False
    assert dispatcher.vehicles['agv1'].current_command_id == ''
    assert dispatcher._zone_owner['PARK1'] == ''
    assert dispatcher._zone_owner['PARK2'] == ''


def test_dispatch_park_ignores_a_busy_vehicle():
    dispatcher = make_park_dispatcher()
    dispatcher.vehicles['agv1'].busy = True

    _run_to_completion(dispatcher._dispatch_park('agv1'))

    assert dispatcher._zone_owner['PARK1'] == ''


def test_dispatch_park_auto_selects_when_no_vehicle_id_given():
    dispatcher = make_park_dispatcher()
    dispatcher.vehicles['agv1'].busy = True

    _run_to_completion(dispatcher._dispatch_park(''))

    assert dispatcher._zone_owner['PARK2'] == ''


def test_dispatch_park_uses_each_vehicles_own_dedicated_spot():
    """agv1 and agv2 must never compete for the same physical spot."""
    dispatcher = make_park_dispatcher()

    _run_to_completion(dispatcher._dispatch_park('agv2'))

    assert dispatcher.vehicles['agv2'].locked_zone == ''
    assert dispatcher._zone_owner['PARK1'] == ''
    assert dispatcher._zone_owner['PARK2'] == ''


def test_parking_failure_runs_adaptive_recovery_before_retry():
    dispatcher = make_park_dispatcher()
    dispatcher.park_clients['agv1'] = SequencedParkingClient([
        (False, 'FollowPath PATIENCE_EXCEEDED'),
        (True, 'parking completed'),
    ])
    dispatcher.adaptive_recovery_enabled = True
    dispatcher.adaptive_recovery_vehicle_id = 'all'
    dispatcher._adaptive_recovery_states = {}
    recovery_calls = []

    async def recover(_goal_handle, vehicle_id, command_id):
        recovery_calls.append((vehicle_id, command_id))
        return True, 'turned toward free-space center'

    async def no_sleep(_seconds):
        return

    dispatcher._run_adaptive_lidar_recovery = recover
    dispatcher._sleep_async = no_sleep

    _run_to_completion(dispatcher._dispatch_park('agv1'))

    assert len(recovery_calls) == 1
    assert recovery_calls[0][0] == 'agv1'
    assert dispatcher.vehicles['agv1'].locked_zone == 'PARK1'


def test_moving_vehicle_is_left_alone():
    assert classify_motion_stall(
        stalled_for_sec=0.5, stall_timeout_sec=8.0,
        resends_used=0, max_resends=2, held=False,
    ) == STALL_MOVING


def test_stalled_vehicle_gets_the_goal_re_sent():
    assert classify_motion_stall(
        stalled_for_sec=9.0, stall_timeout_sec=8.0,
        resends_used=0, max_resends=2, held=False,
    ) == STALL_RESEND


def test_re_sends_stop_at_the_limit():
    assert classify_motion_stall(
        stalled_for_sec=9.0, stall_timeout_sec=8.0,
        resends_used=2, max_resends=2, held=False,
    ) == STALL_EXHAUSTED


def test_a_held_vehicle_is_not_a_stall():
    """Re-sending through a safety hold cannot help and would burn the budget.

    The vehicle is standing still because something deliberately stopped it,
    so the goal is fine and the stall clock must not run.
    """
    assert classify_motion_stall(
        stalled_for_sec=600.0, stall_timeout_sec=8.0,
        resends_used=0, max_resends=2, held=True,
    ) == STALL_HELD


def test_disabling_re_sends_reports_exhausted_immediately():
    assert classify_motion_stall(
        stalled_for_sec=9.0, stall_timeout_sec=8.0,
        resends_used=0, max_resends=0, held=False,
    ) == STALL_EXHAUSTED


def test_rotating_in_place_counts_as_motion():
    """A recovery spin is progress; re-sending through it would abort it."""
    dispatcher = make_dispatcher()
    odom = Odometry()
    odom.twist.twist.angular.z = 0.6

    dispatcher._on_odom('agv1', odom)

    assert dispatcher.vehicles['agv1'].last_motion_at is not None


def test_a_stationary_vehicle_does_not_refresh_the_stall_clock():
    dispatcher = make_dispatcher()
    odom = Odometry()
    odom.twist.twist.linear.x = 0.001
    odom.twist.twist.angular.z = 0.001

    dispatcher._on_odom('agv1', odom)

    assert dispatcher.vehicles['agv1'].last_motion_at is None


def test_creeping_forward_counts_as_motion():
    dispatcher = make_dispatcher()
    odom = Odometry()
    odom.twist.twist.linear.x = 0.05

    dispatcher._on_odom('agv1', odom)

    assert dispatcher.vehicles['agv1'].last_motion_at is not None


def test_a_collision_hold_marks_the_vehicle_deliberately_stopped():
    """The supervisor holding a vehicle is not a stall the dispatcher can fix."""
    dispatcher = make_dispatcher()
    status = String()
    status.data = json.dumps({'state': 'HOLDING', 'held_vehicle': 'agv1'})

    dispatcher._on_collision_status(status)

    assert dispatcher._vehicle_is_deliberately_stopped('agv1')
    assert not dispatcher._vehicle_is_deliberately_stopped('agv2')


def test_a_released_hold_lets_the_stall_watchdog_run_again():
    dispatcher = make_dispatcher()
    holding = String()
    holding.data = json.dumps({'held_vehicle': 'agv1'})
    released = String()
    released.data = json.dumps({'state': 'MONITORING', 'held_vehicle': ''})

    dispatcher._on_collision_status(holding)
    dispatcher._on_collision_status(released)

    assert not dispatcher._vehicle_is_deliberately_stopped('agv1')


def test_malformed_collision_status_is_ignored():
    dispatcher = make_dispatcher()
    dispatcher._collision_held_vehicle = 'agv2'
    broken = String()
    broken.data = 'not json'

    dispatcher._on_collision_status(broken)

    assert dispatcher._collision_held_vehicle == 'agv2'


def test_emergency_stop_also_counts_as_deliberately_stopped():
    dispatcher = make_dispatcher()
    dispatcher.vehicles['agv2'].emergency = True

    assert dispatcher._vehicle_is_deliberately_stopped('agv2')


def make_park_exit_dispatcher():
    dispatcher = make_dispatcher()
    dispatcher.park_exit_inflation_clear_distance_m = 0.70
    return dispatcher


def _arm_park_exit(dispatcher, vehicle_id, origin=(1.674, 0.408)):
    runtime = dispatcher.vehicles[vehicle_id]
    runtime.park_exit_inflation_restore_m = 0.20
    runtime.park_exit_origin_xy = origin
    return runtime


def _place(runtime, x, y):
    runtime.pose.pose.position.x = x
    runtime.pose.pose.position.y = y


def test_inflation_stays_relaxed_until_the_vehicle_clears_the_pocket():
    """Keep it relaxed past the straight leg.

    The measured pocket still reads as a collision 0.20m out, which is where
    the DriveOnHeading exit ends.
    """
    dispatcher = make_park_exit_dispatcher()
    runtime = _arm_park_exit(dispatcher, 'agv1')

    _place(runtime, 1.474, 0.399)  # 0.20 m out - end of the straight leg
    dispatcher._check_park_exit_inflation()

    assert dispatcher.executor.tasks == []
    assert runtime.park_exit_inflation_restore_m == 0.20


def test_inflation_is_restored_once_clear_of_the_pocket():
    dispatcher = make_park_exit_dispatcher()
    runtime = _arm_park_exit(dispatcher, 'agv1')

    _place(runtime, 0.974, 0.408)  # 0.70 m out
    dispatcher._check_park_exit_inflation()

    assert len(dispatcher.executor.tasks) == 1


def test_a_vehicle_without_a_relaxed_costmap_is_never_restored():
    """Only a vehicle that actually relaxed its inflation may be restored."""
    dispatcher = make_park_exit_dispatcher()
    runtime = _arm_park_exit(dispatcher, 'agv1')
    _place(runtime, 1.674, 0.408)  # still sitting in the spot
    # agv2 never started an exit, so it has nothing recorded to put back.
    _place(dispatcher.vehicles['agv2'], 5.0, 5.0)

    dispatcher._check_park_exit_inflation()

    assert dispatcher.executor.tasks == []
    assert dispatcher.vehicles['agv2'].park_exit_inflation_restore_m is None


class RecordingPublisher:
    def __init__(self):
        self.commands = []
        self.angular_commands = []

    def publish(self, message):
        self.commands.append(float(message.linear.x))
        self.angular_commands.append(float(message.angular.z))


class FakeGoalHandle:
    def __init__(self, cancel=False):
        self.is_cancel_requested = cancel


def make_open_loop_dispatcher():
    dispatcher = make_dispatcher()
    dispatcher.park_exit_open_loop_rate_hz = 20.0
    dispatcher.park_exit_cmd_publishers = {
        'agv1': RecordingPublisher(),
        'agv2': RecordingPublisher(),
    }

    async def _no_sleep(_seconds):
        return

    dispatcher._sleep_async = _no_sleep
    return dispatcher


def test_open_loop_exit_drives_forward_then_stops():
    """Nav2 refuses to leave the pocket, so the exit is a timed cmd_vel move."""
    dispatcher = make_open_loop_dispatcher()

    _run_to_completion(
        dispatcher._drive_straight_open_loop(
            FakeGoalHandle(), 'agv1', 0.20, 0.05, 'parking exit'
        )
    )

    commands = dispatcher.park_exit_cmd_publishers['agv1'].commands
    assert commands, 'no velocity was published'
    assert all(value == 0.05 for value in commands[:-1])
    # The gate times out on its own, but the exit must still stop explicitly.
    assert commands[-1] == 0.0


def test_open_loop_exit_stops_on_an_emergency_latch():
    dispatcher = make_open_loop_dispatcher()
    dispatcher.vehicles['agv1'].emergency = True

    _run_to_completion(
        dispatcher._drive_straight_open_loop(
            FakeGoalHandle(), 'agv1', 0.20, 0.05, 'parking exit'
        )
    )

    commands = dispatcher.park_exit_cmd_publishers['agv1'].commands
    assert commands == [0.0], 'an emergency latch must publish only the stop'


def test_open_loop_exit_stops_when_the_goal_is_canceled():
    dispatcher = make_open_loop_dispatcher()

    _run_to_completion(
        dispatcher._drive_straight_open_loop(
            FakeGoalHandle(cancel=True), 'agv1', 0.20, 0.05, 'parking exit'
        )
    )

    assert dispatcher.park_exit_cmd_publishers['agv1'].commands == [0.0]


def test_b1_forward_uses_safety_gated_open_loop_when_enabled():
    """B-1 must escape a false local-costmap collision before Nav2 planning."""
    dispatcher = make_open_loop_dispatcher()
    dispatcher.b1_exit_open_loop = True
    dispatcher.b1_exit_forward_speed_mps = 0.05

    operation = dispatcher._drive_forward(
        FakeGoalHandle(), 'agv1', 'command-1', 0.30
    )
    try:
        operation.send(None)
    except StopIteration as completed:
        success, message = completed.value
    else:
        raise AssertionError('B-1 exit coroutine unexpectedly suspended')

    assert success
    assert message == ''
    commands = dispatcher.park_exit_cmd_publishers['agv1'].commands
    assert commands
    assert all(value == 0.05 for value in commands[:-1])
    assert commands[-1] == 0.0


def test_b1_manual_turn_waits_for_measured_ninety_degrees_before_success():
    dispatcher = make_open_loop_dispatcher()
    dispatcher.b1_exit_manual_turn = True
    dispatcher.b1_exit_turn_speed_rps = 0.25
    dispatcher.b1_exit_turn_tolerance_rad = math.radians(5.0)
    dispatcher.b1_exit_behavior_timeout_sec = 1.0
    runtime = dispatcher.vehicles['agv1']
    runtime.pose.pose.orientation.w = 1.0
    updates = 0

    async def _turn_pose_after_command(_seconds):
        nonlocal updates
        updates += 1
        yaw = math.radians(90.0)
        runtime.pose.pose.orientation.z = math.sin(yaw * 0.5)
        runtime.pose.pose.orientation.w = math.cos(yaw * 0.5)

    dispatcher._sleep_async = _turn_pose_after_command
    operation = dispatcher._rotate_b1_exit_verified(
        FakeGoalHandle(), 'agv1', 'command-1', math.radians(90.0)
    )
    try:
        operation.send(None)
    except StopIteration as completed:
        success, message = completed.value
    else:
        raise AssertionError('B-1 turn coroutine unexpectedly suspended')

    assert success
    assert message == ''
    assert updates == 1
    angular = dispatcher.park_exit_cmd_publishers['agv1'].angular_commands
    assert angular[0] > 0.0
    assert angular[-1] == 0.0
    assert all(value == 0.0 for value in (
        dispatcher.park_exit_cmd_publishers['agv1'].commands
    ))
