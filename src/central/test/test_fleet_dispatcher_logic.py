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
    dispatcher.vehicles = {
        vehicle_id: VehicleRuntime(vehicle_id)
        for vehicle_id in ('agv1', 'agv2')
    }
    dispatcher._vehicle_ready = lambda vehicle_id, _waypoints=False: (
        not dispatcher.vehicles[vehicle_id].busy
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


def test_vehicle_is_not_dispatchable_before_initial_pose():
    dispatcher = object.__new__(FleetDispatcher)
    dispatcher.telemetry_timeout = 3.0
    runtime = VehicleRuntime('agv1')
    runtime.pose_received_at = time.monotonic()
    dispatcher.vehicles = {'agv1': runtime}
    dispatcher.nav_pose_clients = {'agv1': ReadyActionClient()}
    dispatcher.nav_waypoint_clients = {'agv1': ReadyActionClient()}

    assert not dispatcher._vehicle_ready('agv1')
    runtime.has_amcl_pose = True
    assert dispatcher._vehicle_ready('agv1')
