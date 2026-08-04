import math
import threading
import time

from central.fleet_dispatcher import FleetDispatcher, VehicleRuntime

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry


class ReadyActionClient:
    def server_is_ready(self):
        return True


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
    dispatcher.subscribe_odom_fallback = True
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
    return dispatcher


def target_pose(x, y):
    pose = PoseStamped()
    pose.pose.position.x = x
    pose.pose.position.y = y
    return pose


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


def test_b1_exit_maneuver_does_not_trigger_far_from_zone():
    dispatcher = make_zoned_dispatcher()
    runtime = dispatcher.vehicles['agv1']
    dispatcher._zone_target_poses['B-1'] = target_pose(1.0, 0.0)
    runtime.pose.pose.position.x = 0.60

    assert not dispatcher._requires_b1_exit_maneuver(runtime, 'A')


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
