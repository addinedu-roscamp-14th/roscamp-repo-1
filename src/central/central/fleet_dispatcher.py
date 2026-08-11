#!/usr/bin/env python3

"""Two-vehicle Nav2 dispatcher with exclusive-zone queues and emergency control."""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
import json
import math
import threading
import time

from action_msgs.msg import GoalStatus
from drive.action import ParkInSpot
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped, Twist
from nav2_msgs.action import (
    DriveOnHeading,
    NavigateThroughPoses,
    NavigateToPose,
    Spin,
)
from nav_msgs.msg import Odometry
from porter_interfaces.action import DispatchNavigation
from porter_interfaces.msg import VehicleState
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.task import Future
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class VehicleRuntime:
    vehicle_id: str
    pose: PoseStamped = field(default_factory=PoseStamped)
    pose_received_at: float | None = None
    telemetry_received_at: float | None = None
    has_amcl_pose: bool = False
    battery_percent: float = math.nan
    battery_voltage: float = math.nan
    busy: bool = False
    emergency: bool = False
    current_command_id: str = ''
    current_target_zone: str = ''
    current_target_pose: PoseStamped | None = None
    locked_zone: str = ''
    active_nav_goal: object | None = None
    odom_speed: float = 0.0
    odom_yaw_rate: float = 0.0
    last_motion_at: float | None = None
    b1_exit_turn_completed: bool = False
    b1_exit_forward_completed: bool = False
    park_exit_forward_completed: bool = False
    # Local-costmap inflation is relaxed while the vehicle crawls out of its
    # parking pocket, where the measured wall clearance is below the footprint
    # inscribed radius. Holds the value to put back, so the relaxation can
    # never outlive the manoeuvre.
    park_exit_inflation_restore_m: float | None = None
    park_exit_origin_xy: tuple[float, float] | None = None


STALL_MOVING = 'moving'
STALL_HELD = 'held'
STALL_RESEND = 'resend'
STALL_EXHAUSTED = 'exhausted'


def classify_motion_stall(
    stalled_for_sec,
    stall_timeout_sec,
    resends_used,
    max_resends,
    held,
):
    """Decide what to do about a vehicle that accepted a goal but sits still.

    Nav2 accepting a goal says nothing about the vehicle actually moving: a
    latched safety hold, a wedged controller or a plan that never reaches the
    wheels all look like a goal quietly in progress. `held` covers the case
    where something deliberately stopped the vehicle, which is not a stall and
    must not be answered by re-sending the goal.
    """
    if held:
        return STALL_HELD
    if stalled_for_sec < stall_timeout_sec:
        return STALL_MOVING
    if resends_used >= max_resends:
        return STALL_EXHAUSTED
    return STALL_RESEND


class FleetDispatcher(Node):
    """Assign central goals to two namespaced Nav2 action servers."""

    ERROR_INVALID_REQUEST = 1
    ERROR_NO_VEHICLE = 2
    ERROR_NAV_REJECTED = 3
    ERROR_NAV_FAILED = 4
    ERROR_CANCELED = 5

    def __init__(self):
        super().__init__('fleet_dispatcher')
        self.declare_parameter('vehicle_ids', ['agv1', 'agv2'])
        self.declare_parameter('dispatch_action', '/central/dispatch_navigation')
        self.declare_parameter('telemetry_timeout_sec', 3.0)
        self.declare_parameter('state_publish_rate_hz', 2.0)
        # A goal Nav2 accepted can still leave the vehicle standing still.
        # Watch odometry and fail safely instead of waiting out the action's
        # full timeout. Automatic re-send is opt-in: some Nav2 Jazzy versions
        # abort bt_navigator if a replacement arrives while cancellation is
        # still being finalized, especially through a remote action bridge.
        self.declare_parameter('motion_threshold_mps', 0.015)
        self.declare_parameter('motion_yaw_threshold_rps', 0.05)
        self.declare_parameter('motion_stall_timeout_sec', 20.0)
        self.declare_parameter('max_motion_resends', 0)
        self.declare_parameter('cancel_settle_timeout_sec', 10.0)
        self.declare_parameter('max_park_retries', 2)
        self.declare_parameter('park_retry_backoff_sec', 2.0)
        self.declare_parameter(
            'exclusive_zone_ids', ['B-1', 'A', 'PARK1', 'PARK2']
        )
        self.declare_parameter('zone_occupancy_radius_m', 0.18)
        self.declare_parameter('zone_release_hysteresis_m', 0.05)
        self.declare_parameter('b1_exit_left_turn_deg', 90.0)
        self.declare_parameter('b1_exit_forward_distance_m', 0.30)
        self.declare_parameter('b1_exit_forward_speed_mps', 0.05)
        self.declare_parameter('b1_exit_behavior_timeout_sec', 20.0)
        self.declare_parameter('b1_exit_detection_radius_m', 0.35)
        # Spin overshoots the commanded angle, so feeding the measured error
        # straight back oscillated (+21, -11, +9 deg) and ran out of
        # corrections without ever landing inside the tolerance. Every failure
        # aborted the command before the mandatory forward move could run.
        # Turn once and advance; set true to restore closed-loop correction.
        self.declare_parameter('b1_exit_turn_verify', False)
        self.declare_parameter('b1_exit_turn_tolerance_deg', 5.0)
        self.declare_parameter('b1_exit_turn_max_corrections', 2)
        self.declare_parameter('b1_exit_pose_update_timeout_sec', 3.0)
        self.declare_parameter('b1_exit_turn_settle_sec', 0.5)
        self.declare_parameter('b1_zone_map_x', 1.294)
        self.declare_parameter('b1_zone_map_y', -0.087)
        # Each vehicle has its own dedicated, non-shared parking spot - no
        # contention, so no FIFO queueing is needed between agv1 and agv2.
        self.declare_parameter('agv1_park_spot_id', 'park_red')
        self.declare_parameter('agv1_park_zone_map_x', 1.673782)
        self.declare_parameter('agv1_park_zone_map_y', 0.408066)
        self.declare_parameter('agv2_park_spot_id', 'parking_yellow')
        self.declare_parameter('agv2_park_zone_map_x', 1.635463773844374)
        self.declare_parameter('agv2_park_zone_map_y', 0.16880950610511666)
        self.declare_parameter(
            'park_request_topic', '/central/fleet/park_request'
        )
        # Zero disables idle auto-parking. Parking must be triggered by a
        # current API request unless an operator explicitly enables this.
        self.declare_parameter('auto_park_idle_sec', 0.0)
        self.declare_parameter('auto_park_check_interval_sec', 3.0)
        self.declare_parameter('park_action_wait_timeout_sec', 10.0)
        self.declare_parameter('park_exit_forward_distance_m', 0.20)
        self.declare_parameter('park_exit_forward_speed_mps', 0.05)
        self.declare_parameter('park_exit_behavior_timeout_sec', 20.0)
        self.declare_parameter('park_exit_detection_radius_m', 0.25)
        # The parking pockets are tighter than the footprint's inscribed
        # radius (0.069 m): measured wall clearance is 0.023 m for agv1 and
        # 0.054 m for agv2, so every cell the vehicle occupies is already
        # INSCRIBED_INFLATED_OBSTACLE and DriveOnHeading aborts with
        # COLLISION_AHEAD. Shrink the rolling costmap's inflation until the
        # vehicle is clear of the pocket, then put it straight back.
        self.declare_parameter('park_exit_inflation_radius_m', 0.01)
        self.declare_parameter('park_exit_inflation_clear_distance_m', 0.70)
        # Nav2's DriveOnHeading cannot leave a pocket this tight even with the
        # inflation relaxed, so the exit is an open-loop timed move like the
        # entry (parking_new.reverse_to_parked). Set false to go back to
        # DriveOnHeading once the spots have real clearance.
        self.declare_parameter('park_exit_open_loop', True)
        self.declare_parameter('park_exit_open_loop_rate_hz', 20.0)
        self.declare_parameter('duplicate_goal_distance_m', 0.12)
        self.declare_parameter('duplicate_goal_yaw_tolerance_deg', 20.0)
        self.declare_parameter('sequence_dependency_timeout_sec', 300.0)
        self.declare_parameter('cargo_hold_timeout_sec', 300.0)
        self.declare_parameter('subscribe_odom_fallback', False)

        vehicle_ids = [
            str(value).strip('/')
            for value in self.get_parameter('vehicle_ids').value
        ]
        if vehicle_ids != ['agv1', 'agv2']:
            raise ValueError('vehicle_ids must be [agv1, agv2]')
        self.telemetry_timeout = float(
            self.get_parameter('telemetry_timeout_sec').value
        )
        self.motion_threshold_mps = float(
            self.get_parameter('motion_threshold_mps').value
        )
        if self.motion_threshold_mps <= 0.0:
            raise ValueError('motion_threshold_mps must be positive')
        self.motion_yaw_threshold_rps = float(
            self.get_parameter('motion_yaw_threshold_rps').value
        )
        if self.motion_yaw_threshold_rps <= 0.0:
            raise ValueError('motion_yaw_threshold_rps must be positive')
        self.motion_stall_timeout_sec = float(
            self.get_parameter('motion_stall_timeout_sec').value
        )
        if self.motion_stall_timeout_sec <= 0.0:
            raise ValueError('motion_stall_timeout_sec must be positive')
        self.max_motion_resends = int(
            self.get_parameter('max_motion_resends').value
        )
        if self.max_motion_resends < 0:
            raise ValueError('max_motion_resends must be non-negative')
        self.cancel_settle_timeout_sec = float(
            self.get_parameter('cancel_settle_timeout_sec').value
        )
        if self.cancel_settle_timeout_sec < 0.0:
            raise ValueError('cancel_settle_timeout_sec must be non-negative')
        self.max_park_retries = int(
            self.get_parameter('max_park_retries').value
        )
        if self.max_park_retries < 0:
            raise ValueError('max_park_retries must be non-negative')
        self.park_retry_backoff_sec = float(
            self.get_parameter('park_retry_backoff_sec').value
        )
        if self.park_retry_backoff_sec < 0.0:
            raise ValueError('park_retry_backoff_sec must be non-negative')
        # Zones a single vehicle occupies exclusively (e.g. B-1 ship loading
        # bay, A shared cargo-bin stop): only one vehicle may hold one at a
        # time, others queue until it leaves.
        self.exclusive_zone_ids = tuple(
            str(value) for value in self.get_parameter('exclusive_zone_ids').value
        )
        self.zone_occupancy_radius_m = float(
            self.get_parameter('zone_occupancy_radius_m').value
        )
        if self.zone_occupancy_radius_m <= 0.0:
            raise ValueError('zone_occupancy_radius_m must be positive')
        self.zone_release_hysteresis_m = float(
            self.get_parameter('zone_release_hysteresis_m').value
        )
        if self.zone_release_hysteresis_m < 0.0:
            raise ValueError('zone_release_hysteresis_m must be non-negative')
        self.b1_exit_left_turn_deg = float(
            self.get_parameter('b1_exit_left_turn_deg').value
        )
        if not 0.0 < self.b1_exit_left_turn_deg <= 180.0:
            raise ValueError('b1_exit_left_turn_deg must be in (0, 180]')
        self.b1_exit_forward_distance_m = float(
            self.get_parameter('b1_exit_forward_distance_m').value
        )
        if self.b1_exit_forward_distance_m < 0.0:
            raise ValueError(
                'b1_exit_forward_distance_m must be non-negative'
            )
        self.b1_exit_forward_speed_mps = float(
            self.get_parameter('b1_exit_forward_speed_mps').value
        )
        if self.b1_exit_forward_speed_mps <= 0.0:
            raise ValueError('b1_exit_forward_speed_mps must be positive')
        self.b1_exit_behavior_timeout_sec = float(
            self.get_parameter('b1_exit_behavior_timeout_sec').value
        )
        if self.b1_exit_behavior_timeout_sec <= 0.0:
            raise ValueError('b1_exit_behavior_timeout_sec must be positive')
        self.b1_exit_detection_radius_m = float(
            self.get_parameter('b1_exit_detection_radius_m').value
        )
        if self.b1_exit_detection_radius_m <= 0.0:
            raise ValueError('b1_exit_detection_radius_m must be positive')
        self.b1_exit_turn_verify = bool(
            self.get_parameter('b1_exit_turn_verify').value
        )
        self.b1_exit_turn_tolerance_rad = math.radians(float(
            self.get_parameter('b1_exit_turn_tolerance_deg').value
        ))
        if not 0.0 < self.b1_exit_turn_tolerance_rad < math.pi:
            raise ValueError('b1_exit_turn_tolerance_deg must be in (0, 180)')
        self.b1_exit_turn_max_corrections = int(
            self.get_parameter('b1_exit_turn_max_corrections').value
        )
        if self.b1_exit_turn_max_corrections < 0:
            raise ValueError('b1_exit_turn_max_corrections must be non-negative')
        self.b1_exit_pose_update_timeout_sec = float(
            self.get_parameter('b1_exit_pose_update_timeout_sec').value
        )
        if self.b1_exit_pose_update_timeout_sec <= 0.0:
            raise ValueError('b1_exit_pose_update_timeout_sec must be positive')
        self.b1_exit_turn_settle_sec = float(
            self.get_parameter('b1_exit_turn_settle_sec').value
        )
        if self.b1_exit_turn_settle_sec < 0.0:
            raise ValueError('b1_exit_turn_settle_sec must be non-negative')
        self.b1_zone_map_x = float(
            self.get_parameter('b1_zone_map_x').value
        )
        self.b1_zone_map_y = float(
            self.get_parameter('b1_zone_map_y').value
        )
        self.park_zone_ids = {'agv1': 'PARK1', 'agv2': 'PARK2'}
        self.park_spot_ids = {}
        self.park_zone_map_xy = {}
        for vehicle_id in ('agv1', 'agv2'):
            spot_id = str(
                self.get_parameter(f'{vehicle_id}_park_spot_id').value
            )
            if not spot_id:
                raise ValueError(
                    f'{vehicle_id}_park_spot_id must not be empty'
                )
            self.park_spot_ids[vehicle_id] = spot_id
            self.park_zone_map_xy[vehicle_id] = (
                float(
                    self.get_parameter(f'{vehicle_id}_park_zone_map_x').value
                ),
                float(
                    self.get_parameter(f'{vehicle_id}_park_zone_map_y').value
                ),
            )
        self.park_request_topic = str(
            self.get_parameter('park_request_topic').value
        )
        self.park_exit_forward_distance_m = float(
            self.get_parameter('park_exit_forward_distance_m').value
        )
        if self.park_exit_forward_distance_m <= 0.0:
            raise ValueError('park_exit_forward_distance_m must be positive')
        self.park_exit_forward_speed_mps = float(
            self.get_parameter('park_exit_forward_speed_mps').value
        )
        if self.park_exit_forward_speed_mps <= 0.0:
            raise ValueError('park_exit_forward_speed_mps must be positive')
        self.park_exit_behavior_timeout_sec = float(
            self.get_parameter('park_exit_behavior_timeout_sec').value
        )
        if self.park_exit_behavior_timeout_sec <= 0.0:
            raise ValueError('park_exit_behavior_timeout_sec must be positive')
        self.park_exit_detection_radius_m = float(
            self.get_parameter('park_exit_detection_radius_m').value
        )
        if self.park_exit_detection_radius_m <= 0.0:
            raise ValueError('park_exit_detection_radius_m must be positive')
        self.park_exit_inflation_radius_m = float(
            self.get_parameter('park_exit_inflation_radius_m').value
        )
        if self.park_exit_inflation_radius_m <= 0.0:
            raise ValueError(
                'park_exit_inflation_radius_m must be positive'
            )
        self.park_exit_inflation_clear_distance_m = float(
            self.get_parameter('park_exit_inflation_clear_distance_m').value
        )
        if self.park_exit_inflation_clear_distance_m <= 0.0:
            raise ValueError(
                'park_exit_inflation_clear_distance_m must be positive'
            )
        self.park_exit_open_loop = bool(
            self.get_parameter('park_exit_open_loop').value
        )
        self.park_exit_open_loop_rate_hz = float(
            self.get_parameter('park_exit_open_loop_rate_hz').value
        )
        if self.park_exit_open_loop_rate_hz <= 0.0:
            raise ValueError(
                'park_exit_open_loop_rate_hz must be positive'
            )
        self.auto_park_idle_sec = float(
            self.get_parameter('auto_park_idle_sec').value
        )
        if self.auto_park_idle_sec < 0.0:
            raise ValueError('auto_park_idle_sec must be non-negative')
        self.auto_park_check_interval_sec = float(
            self.get_parameter('auto_park_check_interval_sec').value
        )
        if self.auto_park_check_interval_sec <= 0.0:
            raise ValueError(
                'auto_park_check_interval_sec must be positive'
            )
        self.park_action_wait_timeout_sec = float(
            self.get_parameter('park_action_wait_timeout_sec').value
        )
        if self.park_action_wait_timeout_sec < 0.0:
            raise ValueError(
                'park_action_wait_timeout_sec must be non-negative'
            )
        self.duplicate_goal_distance_m = float(
            self.get_parameter('duplicate_goal_distance_m').value
        )
        if self.duplicate_goal_distance_m <= 0.0:
            raise ValueError('duplicate_goal_distance_m must be positive')
        self.duplicate_goal_yaw_tolerance_rad = math.radians(float(
            self.get_parameter('duplicate_goal_yaw_tolerance_deg').value
        ))
        if not 0.0 < self.duplicate_goal_yaw_tolerance_rad <= math.pi:
            raise ValueError(
                'duplicate_goal_yaw_tolerance_deg must be in (0, 180]'
            )
        self.sequence_dependency_timeout_sec = float(
            self.get_parameter('sequence_dependency_timeout_sec').value
        )
        if self.sequence_dependency_timeout_sec <= 0.0:
            raise ValueError(
                'sequence_dependency_timeout_sec must be positive'
            )
        self.cargo_hold_timeout_sec = float(
            self.get_parameter('cargo_hold_timeout_sec').value
        )
        if self.cargo_hold_timeout_sec <= 0.0:
            raise ValueError('cargo_hold_timeout_sec must be positive')
        self.subscribe_odom_fallback = bool(
            self.get_parameter('subscribe_odom_fallback').value
        )
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        # Vehicles the arm dispatcher still owes a cargo operation. Refreshed
        # wholesale by every snapshot, so it cannot drift out of sync.
        self._cargo_held_vehicles = set()
        self._cargo_condition = threading.Condition()
        self._vehicle_condition = threading.Condition(self._lock)
        self._vehicle_queue = {
            vehicle_id: [] for vehicle_id in vehicle_ids
        }
        self._preempted_commands = set()
        self._command_condition = threading.Condition(self._lock)
        self._command_outcomes = {}
        self._zone_condition = threading.Condition(self._lock)
        self._zone_owner = {zone_id: '' for zone_id in self.exclusive_zone_ids}
        self._zone_unknown = {
            zone_id: False for zone_id in self.exclusive_zone_ids
        }
        self._zone_queue = {zone_id: [] for zone_id in self.exclusive_zone_ids}
        self._zone_target_poses = {}
        if 'B-1' in self.exclusive_zone_ids:
            b1_pose = PoseStamped()
            b1_pose.header.frame_id = 'map'
            b1_pose.pose.position.x = self.b1_zone_map_x
            b1_pose.pose.position.y = self.b1_zone_map_y
            b1_pose.pose.orientation.w = 1.0
            self._zone_target_poses['B-1'] = b1_pose
        for vehicle_id, zone_id in self.park_zone_ids.items():
            if zone_id not in self.exclusive_zone_ids:
                continue
            park_x, park_y = self.park_zone_map_xy[vehicle_id]
            park_pose = PoseStamped()
            park_pose.header.frame_id = 'map'
            park_pose.pose.position.x = park_x
            park_pose.pose.position.y = park_y
            park_pose.pose.orientation.w = 1.0
            self._zone_target_poses[zone_id] = park_pose
        self._startup_zone_recovery_pending = set(
            self._zone_target_poses
        )
        self._zone_entered = {
            zone_id: False for zone_id in self.exclusive_zone_ids
        }

        self.vehicles = {
            vehicle_id: VehicleRuntime(vehicle_id)
            for vehicle_id in vehicle_ids
        }
        self.nav_pose_clients = {}
        self.nav_waypoint_clients = {}
        self.spin_clients = {}
        self.drive_on_heading_clients = {}
        self.park_clients = {}
        self.gate_clients = {}
        self.state_publishers = {}
        self.local_costmap_set_param_clients = {}
        self.local_costmap_get_param_clients = {}
        self.park_exit_cmd_publishers = {}
        for vehicle_id in vehicle_ids:
            # cmd_vel_safety_gate feeds this into the vehicle's cmd_vel, so the
            # emergency and collision latches still gate the exit crawl.
            self.park_exit_cmd_publishers[vehicle_id] = self.create_publisher(
                Twist, f'/{vehicle_id}/cmd_vel_manual', 10
            )
            costmap_node = f'/{vehicle_id}/local_costmap/local_costmap'
            self.local_costmap_set_param_clients[vehicle_id] = (
                self.create_client(
                    SetParameters,
                    f'{costmap_node}/set_parameters',
                    callback_group=self.callback_group,
                )
            )
            self.local_costmap_get_param_clients[vehicle_id] = (
                self.create_client(
                    GetParameters,
                    f'{costmap_node}/get_parameters',
                    callback_group=self.callback_group,
                )
            )
            self.nav_pose_clients[vehicle_id] = ActionClient(
                self,
                NavigateToPose,
                f'/{vehicle_id}/navigate_to_pose',
                callback_group=self.callback_group,
            )
            self.nav_waypoint_clients[vehicle_id] = ActionClient(
                self,
                NavigateThroughPoses,
                f'/{vehicle_id}/navigate_through_poses',
                callback_group=self.callback_group,
            )
            self.spin_clients[vehicle_id] = ActionClient(
                self,
                Spin,
                f'/{vehicle_id}/spin',
                callback_group=self.callback_group,
            )
            self.drive_on_heading_clients[vehicle_id] = ActionClient(
                self,
                DriveOnHeading,
                f'/{vehicle_id}/drive_on_heading',
                callback_group=self.callback_group,
            )
            self.park_clients[vehicle_id] = ActionClient(
                self,
                ParkInSpot,
                f'/{vehicle_id}/park_in_spot',
                callback_group=self.callback_group,
            )
            self.gate_clients[vehicle_id] = self.create_client(
                SetBool,
                f'/{vehicle_id}/emergency_stop',
                callback_group=self.callback_group,
            )
            self.state_publishers[vehicle_id] = self.create_publisher(
                VehicleState,
                f'/central/fleet/{vehicle_id}/state',
                10,
            )
            self._create_vehicle_subscriptions(vehicle_id)
            self.create_service(
                SetBool,
                f'/central/fleet/{vehicle_id}/emergency_stop',
                lambda request, response, vid=vehicle_id:
                    self._set_emergency_service(vid, request, response),
                callback_group=self.callback_group,
            )

        # A vehicle the collision supervisor is holding is standing still on
        # purpose. Without this the stall watchdog reads that as a wedged goal
        # and burns its re-sends waiting out a hold it cannot affect.
        self._collision_held_vehicle = ''
        self.create_subscription(
            String,
            '/central/fleet/collision_status',
            self._on_collision_status,
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            String,
            '/central/arms/vehicle_holds',
            self._on_vehicle_holds,
            10,
            callback_group=self.callback_group,
        )
        self.zone_publisher = self.create_publisher(
            String, '/central/fleet/zones', 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/central/fleet/vehicle_markers', 10
        )
        # Clear cargo-box markers left by older dispatcher versions once.
        # Vehicle geometry is rendered exclusively by the namespaced URDF.
        self._marker_cleanup_pending = True
        self.create_service(
            SetBool,
            '/central/fleet/emergency_stop',
            self._set_all_emergency_service,
            callback_group=self.callback_group,
        )
        zone_clear_service_names = {'B-1': 'clear_b1_lock', 'A': 'clear_a_lock'}
        for zone_id in self.exclusive_zone_ids:
            service_name = zone_clear_service_names.get(
                zone_id, f'clear_{zone_id.lower()}_lock'
            )
            self.create_service(
                Trigger,
                f'/central/fleet/{service_name}',
                lambda request, response, zid=zone_id:
                    self._clear_zone_lock(zid, request, response),
                callback_group=self.callback_group,
            )
        self.action_server = ActionServer(
            self,
            DispatchNavigation,
            str(self.get_parameter('dispatch_action').value),
            execute_callback=self._execute,
            goal_callback=self._accept_goal,
            cancel_callback=self._cancel_goal,
            callback_group=self.callback_group,
        )
        rate = float(self.get_parameter('state_publish_rate_hz').value)
        self.create_timer(1.0 / rate, self._publish_states)
        self.create_timer(
            0.5,
            self._check_park_exit_inflation,
            callback_group=self.callback_group,
        )

        self._idle_since = {vehicle_id: None for vehicle_id in vehicle_ids}
        self.create_subscription(
            String,
            self.park_request_topic,
            self._on_park_request,
            10,
            callback_group=self.callback_group,
        )
        if self.auto_park_idle_sec > 0.0:
            self.create_timer(
                self.auto_park_check_interval_sec,
                self._check_auto_park,
                callback_group=self.callback_group,
            )
        park_summary = ', '.join(
            f'{vehicle_id}={self.park_spot_ids[vehicle_id]}'
            f'@({self.park_zone_map_xy[vehicle_id][0]:.3f}, '
            f'{self.park_zone_map_xy[vehicle_id][1]:.3f})'
            for vehicle_id in ('agv1', 'agv2')
        )
        self.get_logger().info(
            'Fleet dispatcher ready: vehicles=agv1,agv2, B-1 lock enabled; '
            f'B-1 reference=({self.b1_zone_map_x:.3f}, '
            f'{self.b1_zone_map_y:.3f}); park spots: {park_summary}; '
            f'idle auto-parking='
            f'{self.auto_park_idle_sec if self.auto_park_idle_sec > 0.0 else "disabled"}'
        )

    def _create_vehicle_subscriptions(self, vehicle_id):
        telemetry_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{vehicle_id}/amcl_pose',
            lambda message, vid=vehicle_id: self._on_amcl_pose(vid, message),
            telemetry_qos,
            callback_group=self.callback_group,
        )
        # Odom is the vehicle liveness heartbeat even after AMCL is available.
        # A stationary AMCL node may not republish pose until its movement
        # thresholds are crossed, so AMCL freshness cannot indicate whether a
        # vehicle is online.
        self.create_subscription(
            Odometry,
            f'/{vehicle_id}/odom',
            lambda message, vid=vehicle_id: self._on_odom(vid, message),
            telemetry_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            f'/{vehicle_id}/battery/percent',
            lambda message, vid=vehicle_id: self._on_battery(
                vid, 'battery_percent', message
            ),
            telemetry_qos,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            f'/{vehicle_id}/battery/voltage',
            lambda message, vid=vehicle_id: self._on_battery(
                vid, 'battery_voltage', message
            ),
            telemetry_qos,
            callback_group=self.callback_group,
        )

    def _on_amcl_pose(self, vehicle_id, message):
        pose = PoseStamped()
        pose.header = message.header
        pose.pose = message.pose.pose
        with self._lock:
            runtime = self.vehicles[vehicle_id]
            runtime.pose = pose
            runtime.pose_received_at = time.monotonic()
            runtime.telemetry_received_at = runtime.pose_received_at
            runtime.has_amcl_pose = True

    def _on_odom(self, vehicle_id, message):
        with self._lock:
            runtime = self.vehicles[vehicle_id]
            now = time.monotonic()
            runtime.telemetry_received_at = now
            # Rotating counts as moving: a recovery spin is progress, not a
            # stall, and re-sending the goal through it would be wrong.
            twist = message.twist.twist
            runtime.odom_speed = math.hypot(
                float(twist.linear.x), float(twist.linear.y)
            )
            runtime.odom_yaw_rate = abs(float(twist.angular.z))
            if (
                runtime.odom_speed > self.motion_threshold_mps
                or runtime.odom_yaw_rate > self.motion_yaw_threshold_rps
            ):
                runtime.last_motion_at = now
            if runtime.has_amcl_pose or not self.subscribe_odom_fallback:
                return
            runtime.pose.header = message.header
            runtime.pose.pose = message.pose.pose
            runtime.pose_received_at = time.monotonic()

    def _on_collision_status(self, message):
        try:
            held = str(json.loads(message.data).get('held_vehicle', ''))
        except (json.JSONDecodeError, AttributeError, TypeError):
            return
        with self._lock:
            self._collision_held_vehicle = held

    def _vehicle_is_deliberately_stopped(self, vehicle_id):
        """Return whether an external safety mechanism stopped the vehicle."""
        with self._lock:
            return (
                self.vehicles[vehicle_id].emergency
                or self._collision_held_vehicle == vehicle_id
            )

    def _on_battery(self, vehicle_id, attribute, message):
        with self._lock:
            setattr(self.vehicles[vehicle_id], attribute, float(message.data))

    def _accept_goal(self, goal_request):
        if not goal_request.poses:
            return GoalResponse.REJECT
        requested = goal_request.requested_vehicle_id.strip('/')
        if requested and requested not in self.vehicles:
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def _cancel_goal(self, goal_handle):
        command_id = goal_handle.request.command_id
        with self._lock:
            for runtime in self.vehicles.values():
                if runtime.current_command_id == command_id:
                    if runtime.active_nav_goal is not None:
                        runtime.active_nav_goal.cancel_goal_async()
                    break
        return CancelResponse.ACCEPT

    def _vehicle_ready(self, vehicle_id, require_waypoints=False):
        return (
            self._vehicle_operational(vehicle_id, require_waypoints)
            and not self.vehicles[vehicle_id].busy
        )

    def _vehicle_operational(self, vehicle_id, require_waypoints=False):
        """Return whether a vehicle can accept work, ignoring its busy flag."""
        runtime = self.vehicles[vehicle_id]
        telemetry_fresh = (
            runtime.telemetry_received_at is not None
            and time.monotonic() - runtime.telemetry_received_at
            <= self.telemetry_timeout
        )
        action_ready = self.nav_pose_clients[vehicle_id].server_is_ready()
        if require_waypoints:
            action_ready = (
                action_ready
                and self.nav_waypoint_clients[vehicle_id].server_is_ready()
            )
        return (
            telemetry_fresh
            and runtime.has_amcl_pose
            and not runtime.emergency
            and action_ready
        )

    def _vehicle_unready_reasons(self, vehicle_id, require_waypoints=False):
        runtime = self.vehicles[vehicle_id]
        reasons = []
        if runtime.telemetry_received_at is None:
            reasons.append('no odom/amcl telemetry')
        else:
            age = time.monotonic() - runtime.telemetry_received_at
            if age > self.telemetry_timeout:
                reasons.append(f'telemetry stale ({age:.1f}s)')
        if not runtime.has_amcl_pose:
            reasons.append('initial pose/AMCL unavailable')
        if runtime.busy:
            reasons.append('busy')
        if runtime.emergency:
            reasons.append('emergency stopped')
        if not self.nav_pose_clients[vehicle_id].server_is_ready():
            reasons.append('NavigateToPose unavailable')
        elif (
            require_waypoints
            and not self.nav_waypoint_clients[vehicle_id].server_is_ready()
        ):
            reasons.append('NavigateThroughPoses unavailable')
        return reasons or ['unknown readiness failure']

    def _unready_summary(self, requested_vehicle_id, require_waypoints=False):
        requested = requested_vehicle_id.strip('/')
        vehicle_ids = [requested] if requested else sorted(self.vehicles)
        return '; '.join(
            (
                f'{vehicle_id}: '
                + ', '.join(
                    self._vehicle_unready_reasons(
                        vehicle_id,
                        require_waypoints,
                    )
                )
            )
            for vehicle_id in vehicle_ids
        )

    def _find_equivalent_active_vehicle(
        self,
        requested_vehicle_id,
        target_zone_id,
        target_pose,
    ):
        """Return the vehicle already executing the same semantic target."""
        requested = requested_vehicle_id.strip('/')
        vehicle_ids = [requested] if requested else sorted(self.vehicles)
        with self._lock:
            for vehicle_id in vehicle_ids:
                runtime = self.vehicles[vehicle_id]
                active_target = runtime.current_target_pose
                if not runtime.busy or active_target is None:
                    continue
                if runtime.current_command_id in self._preempted_commands:
                    continue
                if runtime.current_target_zone != target_zone_id:
                    continue
                distance = math.hypot(
                    active_target.pose.position.x - target_pose.pose.position.x,
                    active_target.pose.position.y - target_pose.pose.position.y,
                )
                yaw_error = abs(self._normalize_angle(
                    self._pose_yaw(active_target) - self._pose_yaw(target_pose)
                ))
                if (
                    distance <= self.duplicate_goal_distance_m
                    and yaw_error <= self.duplicate_goal_yaw_tolerance_rad
                ):
                    return vehicle_id
        return ''

    def _set_active_target(self, vehicle_id, zone_id, target_pose):
        with self._lock:
            runtime = self.vehicles[vehicle_id]
            runtime.current_target_zone = zone_id
            runtime.current_target_pose = copy.deepcopy(target_pose)
            if zone_id == 'B-1':
                runtime.b1_exit_turn_completed = False
                runtime.b1_exit_forward_completed = False

    def _select_and_reserve_vehicle(
        self,
        requested_vehicle_id,
        target_pose,
        command_id,
        require_waypoints=False,
        target_zone_id='',
    ):
        requested = requested_vehicle_id.strip('/')
        with self._lock:
            if requested:
                selected = (
                    requested
                    if self._vehicle_ready(requested, require_waypoints)
                    else ''
                )
            else:
                exclusive_zone_ids = getattr(
                    self, 'exclusive_zone_ids', ()
                )
                zone_owner = (
                    self._zone_owner.get(target_zone_id, '')
                    if target_zone_id in exclusive_zone_ids
                    else ''
                )
                candidates = [
                    vehicle_id
                    for vehicle_id in sorted(self.vehicles)
                    if self._vehicle_ready(vehicle_id, require_waypoints)
                    and vehicle_id != zone_owner
                ]
                if not candidates:
                    return ''
                target = target_pose.pose.position
                selected = min(
                    candidates,
                    key=lambda vehicle_id: (
                        math.hypot(
                            self.vehicles[
                                vehicle_id
                            ].pose.pose.position.x - target.x,
                            self.vehicles[
                                vehicle_id
                            ].pose.pose.position.y - target.y,
                        ),
                        vehicle_id,
                    ),
                )
            if selected:
                runtime = self.vehicles[selected]
                runtime.busy = True
                runtime.current_command_id = command_id
            return selected

    def _select_preemption_candidate(
        self,
        target_pose,
        require_waypoints=False,
        target_zone_id='',
    ):
        """Select the nearest operational vehicle when no idle vehicle exists."""
        with self._lock:
            zone_owner = (
                self._zone_owner.get(target_zone_id, '')
                if target_zone_id in self.exclusive_zone_ids
                else ''
            )
            candidates = [
                vehicle_id
                for vehicle_id in sorted(self.vehicles)
                if self._vehicle_operational(vehicle_id, require_waypoints)
                and vehicle_id != zone_owner
            ]
            if not candidates:
                return ''
            target = target_pose.pose.position
            return min(
                candidates,
                key=lambda vehicle_id: (
                    math.hypot(
                        self.vehicles[
                            vehicle_id
                        ].pose.pose.position.x - target.x,
                        self.vehicles[
                            vehicle_id
                        ].pose.pose.position.y - target.y,
                    ),
                    vehicle_id,
                ),
            )

    def _wait_and_reserve_explicit_vehicle(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        require_waypoints=False,
    ):
        """Reserve a requested vehicle in FIFO order once it becomes idle."""
        with self._vehicle_condition:
            queue = self._vehicle_queue[vehicle_id]
            if command_id not in queue:
                queue.append(command_id)
            while True:
                if command_id in self._preempted_commands:
                    if command_id in queue:
                        queue.remove(command_id)
                    self._vehicle_condition.notify_all()
                    return '', 'preempted'
                if goal_handle.is_cancel_requested:
                    if command_id in queue:
                        queue.remove(command_id)
                    self._vehicle_condition.notify_all()
                    return '', 'canceled'
                first = queue and queue[0] == command_id
                runtime = self.vehicles[vehicle_id]
                if first and not runtime.busy:
                    if not self._vehicle_ready(
                        vehicle_id,
                        require_waypoints,
                    ):
                        queue.pop(0)
                        self._vehicle_condition.notify_all()
                        return '', 'unready'
                    queue.pop(0)
                    runtime.busy = True
                    runtime.current_command_id = command_id
                    return vehicle_id, 'reserved'
                self._vehicle_condition.wait(timeout=0.1)

    def _preempt_vehicle_commands(self, vehicle_id):
        """Supersede the active and queued commands for one vehicle."""
        with self._vehicle_condition:
            runtime = self.vehicles[vehicle_id]
            if runtime.current_command_id:
                self._preempted_commands.add(runtime.current_command_id)
            for queued_command_id in self._vehicle_queue[vehicle_id]:
                self._preempted_commands.add(queued_command_id)
            self._vehicle_queue[vehicle_id].clear()
            if runtime.active_nav_goal is not None:
                runtime.active_nav_goal.cancel_goal_async()
            self._vehicle_condition.notify_all()
            self._zone_condition.notify_all()

    def _command_preempted(self, command_id):
        with self._lock:
            return command_id in self._preempted_commands

    def _wait_for_predecessor(self, goal_handle, predecessor_command_id):
        if not predecessor_command_id:
            return True, ''
        deadline = (
            time.monotonic() + self.sequence_dependency_timeout_sec
        )
        with self._command_condition:
            while predecessor_command_id not in self._command_outcomes:
                if goal_handle.is_cancel_requested:
                    return False, 'canceled'
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, 'timeout'
                self._command_condition.wait(timeout=min(0.1, remaining))
            if not self._command_outcomes[predecessor_command_id]:
                return False, 'predecessor_failed'
            return True, ''

    def _on_vehicle_holds(self, message):
        """Replace the cargo-hold set from one arm dispatcher snapshot."""
        try:
            payload = json.loads(message.data)
            held = {
                str(vehicle_id).strip('/')
                for vehicle_id in payload.get('held_vehicles', [])
                if str(vehicle_id).strip('/')
            }
        except (AttributeError, TypeError, json.JSONDecodeError) as exc:
            self.get_logger().warning(f'Invalid vehicle hold snapshot: {exc}')
            return
        with self._cargo_condition:
            if held == self._cargo_held_vehicles:
                return
            released = self._cargo_held_vehicles - held
            self._cargo_held_vehicles = held
            self._cargo_condition.notify_all()
        for vehicle_id in sorted(released):
            self.get_logger().info(
                f'{vehicle_id} released by the arm dispatcher'
            )

    def _wait_for_cargo_release(self, goal_handle, vehicle_id):
        """Block a departure until the arm finishes its cargo work.

        The arm dispatcher owns the vehicle while a transfer that declared
        final_for_vehicle is queued or running. Driving away underneath the
        arm would tear the load off the trailer, so the move waits here
        rather than being rejected: the vehicle proceeds on its own once the
        transfer reaches a terminal state, successful or not.
        """
        vehicle_id = str(vehicle_id or '').strip('/')
        if not vehicle_id:
            return True, ''
        deadline = time.monotonic() + self.cargo_hold_timeout_sec
        announced = False
        with self._cargo_condition:
            while vehicle_id in self._cargo_held_vehicles:
                if goal_handle.is_cancel_requested:
                    return False, 'canceled'
                remaining = deadline - time.monotonic()
                if remaining <= 0.0:
                    return False, 'timeout'
                if not announced:
                    announced = True
                    self.get_logger().info(
                        f'{vehicle_id} is holding for an arm cargo operation'
                    )
                self._cargo_condition.wait(timeout=min(0.1, remaining))
        return True, ''

    def _record_command_outcome(self, command_id, succeeded):
        with self._command_condition:
            self._command_outcomes[command_id] = bool(succeeded)
            while len(self._command_outcomes) > 512:
                oldest = next(iter(self._command_outcomes))
                self._command_outcomes.pop(oldest)
            self._command_condition.notify_all()

    def _infer_zone_owner_from_pose(self, zone_id, target_pose):
        """Recover a zone lock after restart from fresh AMCL positions."""
        if zone_id not in self.exclusive_zone_ids:
            return ''
        with self._zone_condition:
            current_owner = self._zone_owner[zone_id]
            if current_owner:
                return current_owner
            target = target_pose.pose.position
            now = time.monotonic()
            candidates = []
            for vehicle_id, runtime in self.vehicles.items():
                telemetry_fresh = (
                    runtime.telemetry_received_at is not None
                    and now - runtime.telemetry_received_at
                    <= self.telemetry_timeout
                )
                if not runtime.has_amcl_pose or not telemetry_fresh:
                    continue
                distance = math.hypot(
                    runtime.pose.pose.position.x - target.x,
                    runtime.pose.pose.position.y - target.y,
                )
                if distance <= self.zone_occupancy_radius_m:
                    candidates.append((distance, vehicle_id))
            if not candidates:
                return ''
            _, owner = min(candidates)
            self._zone_owner[zone_id] = owner
            self._zone_entered[zone_id] = True
            self.vehicles[owner].locked_zone = zone_id
            self.get_logger().info(
                f'Recovered {zone_id} owner={owner} from AMCL pose '
                f'(radius={self.zone_occupancy_radius_m:.2f}m)'
            )
            return owner

    def _recover_startup_zone_owners(self):
        """Recover configured zone occupancy once from fresh AMCL poses."""
        pending = tuple(self._startup_zone_recovery_pending)
        if not pending:
            return
        now = time.monotonic()
        all_vehicles_ready = all(
            runtime.has_amcl_pose
            and runtime.telemetry_received_at is not None
            and now - runtime.telemetry_received_at <= self.telemetry_timeout
            for runtime in self.vehicles.values()
        )
        for zone_id in pending:
            with self._lock:
                owner = self._zone_owner.get(zone_id, '')
                target_pose = self._zone_target_poses.get(zone_id)
            if owner:
                self._startup_zone_recovery_pending.discard(zone_id)
                continue
            if target_pose is None:
                self._startup_zone_recovery_pending.discard(zone_id)
                continue
            owner = self._infer_zone_owner_from_pose(zone_id, target_pose)
            if owner or all_vehicles_ready:
                self._startup_zone_recovery_pending.discard(zone_id)

    def _acquire_zone(self, goal_handle, vehicle_id, command_id, zone_id):
        self._queue_zone_request(vehicle_id, command_id, zone_id)
        return self._wait_for_zone(
            goal_handle,
            vehicle_id,
            command_id,
            zone_id,
        )

    def _queue_zone_request(self, vehicle_id, command_id, zone_id):
        """Join the FIFO queue and acquire immediately when already possible."""
        with self._zone_condition:
            if (
                self._zone_owner[zone_id] == vehicle_id
                and not self._zone_unknown[zone_id]
            ):
                return 'owned'
            queue = self._zone_queue[zone_id]
            if command_id not in queue:
                queue.append(command_id)
            first = queue and queue[0] == command_id
            if (
                first
                and not self._zone_owner[zone_id]
                and not self._zone_unknown[zone_id]
            ):
                queue.pop(0)
                self._zone_owner[zone_id] = vehicle_id
                self._zone_entered[zone_id] = False
                self.vehicles[vehicle_id].locked_zone = zone_id
                return 'acquired'
            return 'queued'

    def _wait_for_zone(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        zone_id,
    ):
        with self._zone_condition:
            while True:
                if (
                    self._zone_owner[zone_id] == vehicle_id
                    and not self._zone_unknown[zone_id]
                ):
                    return True
                queue = self._zone_queue[zone_id]
                first = queue and queue[0] == command_id
                if (
                    first
                    and not self._zone_owner[zone_id]
                    and not self._zone_unknown[zone_id]
                ):
                    queue.pop(0)
                    self._zone_owner[zone_id] = vehicle_id
                    self._zone_entered[zone_id] = False
                    self.vehicles[vehicle_id].locked_zone = zone_id
                    return True
                if goal_handle.is_cancel_requested:
                    if command_id in queue:
                        queue.remove(command_id)
                    return False
                if self._command_preempted(command_id):
                    if command_id in queue:
                        queue.remove(command_id)
                    return False
                self._zone_condition.wait(timeout=0.1)

    def _discard_zone_request(self, command_id, zone_id):
        with self._zone_condition:
            queue = self._zone_queue[zone_id]
            if command_id in queue:
                queue.remove(command_id)
                self._zone_condition.notify_all()

    def _release_zone(self, vehicle_id, zone_id):
        with self._zone_condition:
            if self._zone_owner[zone_id] == vehicle_id:
                runtime = self.vehicles[vehicle_id]
                telemetry_lost = (
                    runtime.telemetry_received_at is None
                    or time.monotonic() - runtime.telemetry_received_at
                    > self.telemetry_timeout
                )
                # A lock acquired before entering the zone is only a
                # reservation. It is safe to release after a failed command
                # even when telemetry is unavailable. Preserve UNKNOWN only
                # for a vehicle that was confirmed inside the zone.
                if telemetry_lost and self._zone_entered[zone_id]:
                    self._zone_unknown[zone_id] = True
                else:
                    self._zone_owner[zone_id] = ''
                    self._zone_entered[zone_id] = False
                    runtime.locked_zone = ''
                self._zone_condition.notify_all()

    def _maybe_clear_stale_zone(self, zone_id, visually_empty):
        """
        Clear a stale zone lock only with telemetry and visual confirmation.

        The owner must already be offline and the current camera frame must
        show no vehicle in the zone.
        """
        if not visually_empty:
            return
        with self._zone_condition:
            if not self._zone_unknown[zone_id]:
                return
            owner = self._zone_owner[zone_id]
            if owner:
                self.vehicles[owner].locked_zone = ''
            self._zone_owner[zone_id] = ''
            self._zone_unknown[zone_id] = False
            self._zone_entered[zone_id] = False
            self.get_logger().warning(
                f'Auto-cleared stale {zone_id} lock (was held by '
                f'{owner or "unknown"}): owner offline and the current '
                'camera frame shows the zone empty'
            )
            self._zone_condition.notify_all()

    async def _execute(self, goal_handle):
        request = goal_handle.request
        result = DispatchNavigation.Result()
        command_id = request.command_id or f'dispatch-{time.time_ns()}'
        self._recover_startup_zone_owners()
        if request.predecessor_command_id:
            self._publish_dispatch_feedback(
                goal_handle,
                request.requested_vehicle_id.strip('/'),
                'WAITING_FOR_PREVIOUS_STEP',
                0,
            )
        predecessor_ok, predecessor_state = self._wait_for_predecessor(
            goal_handle,
            request.predecessor_command_id,
        )
        if not predecessor_ok:
            if predecessor_state == 'canceled':
                goal_handle.canceled()
                result.error_code = self.ERROR_CANCELED
                result.message = 'canceled while waiting for previous step'
            else:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_FAILED
                result.message = (
                    'previous plan step failed'
                    if predecessor_state == 'predecessor_failed'
                    else 'timed out waiting for previous plan step'
                )
            self._record_command_outcome(command_id, False)
            return result
        if request.zone_id in self.exclusive_zone_ids:
            with self._lock:
                self._zone_target_poses[request.zone_id] = copy.deepcopy(
                    request.poses[-1]
                )
        self._infer_zone_owner_from_pose(
            request.zone_id,
            request.poses[-1],
        )
        requested_vehicle_id = request.requested_vehicle_id.strip('/')
        if not request.queue_if_busy and not request.predecessor_command_id:
            duplicate_vehicle_id = self._find_equivalent_active_vehicle(
                requested_vehicle_id,
                request.zone_id,
                request.poses[-1],
            )
            if duplicate_vehicle_id:
                result.assigned_vehicle_id = duplicate_vehicle_id
                result.success = True
                result.message = (
                    'equivalent destination is already active; '
                    'existing command retained'
                )
                self._publish_dispatch_feedback(
                    goal_handle,
                    duplicate_vehicle_id,
                    'DUPLICATE_ACTIVE_GOAL',
                    0,
                )
                goal_handle.succeed()
                self._record_command_outcome(command_id, True)
                self.get_logger().info(
                    f'Coalesced duplicate {command_id} into the active '
                    f'{duplicate_vehicle_id} destination'
                )
                return result
        if requested_vehicle_id:
            if not request.queue_if_busy:
                self._preempt_vehicle_commands(requested_vehicle_id)
            with self._lock:
                waits_for_vehicle = (
                    self.vehicles[requested_vehicle_id].busy
                    or bool(self._vehicle_queue[requested_vehicle_id])
                )
            if waits_for_vehicle:
                self._publish_dispatch_feedback(
                    goal_handle,
                    requested_vehicle_id,
                    'QUEUED_FOR_VEHICLE',
                    0,
                )
            vehicle_id, reservation_state = (
                self._wait_and_reserve_explicit_vehicle(
                    goal_handle,
                    requested_vehicle_id,
                    command_id,
                    require_waypoints=len(request.poses) > 1,
                )
            )
            if reservation_state in ('canceled', 'preempted'):
                if reservation_state == 'canceled':
                    goal_handle.canceled()
                else:
                    goal_handle.abort()
                result.error_code = self.ERROR_CANCELED
                result.message = (
                    'canceled while waiting for requested vehicle'
                    if reservation_state == 'canceled'
                    else 'superseded by a newer vehicle command'
                )
                with self._lock:
                    self._preempted_commands.discard(command_id)
                self._record_command_outcome(command_id, False)
                return result
        else:
            vehicle_id = self._select_and_reserve_vehicle(
                '',
                request.poses[-1],
                command_id,
                require_waypoints=len(request.poses) > 1,
                target_zone_id=request.zone_id,
            )
            if not vehicle_id and not request.queue_if_busy:
                vehicle_id = self._select_preemption_candidate(
                    request.poses[-1],
                    require_waypoints=len(request.poses) > 1,
                    target_zone_id=request.zone_id,
                )
                if vehicle_id:
                    self._preempt_vehicle_commands(vehicle_id)
                    vehicle_id, reservation_state = (
                        self._wait_and_reserve_explicit_vehicle(
                            goal_handle,
                            vehicle_id,
                            command_id,
                            require_waypoints=len(request.poses) > 1,
                        )
                    )
                    if reservation_state in ('canceled', 'preempted'):
                        if reservation_state == 'canceled':
                            goal_handle.canceled()
                        else:
                            goal_handle.abort()
                        result.error_code = self.ERROR_CANCELED
                        result.message = (
                            'canceled while replacing an active AUTO command'
                            if reservation_state == 'canceled'
                            else 'superseded by a newer AUTO command'
                        )
                        self._record_command_outcome(command_id, False)
                        return result
        if not vehicle_id:
            goal_handle.abort()
            result.error_code = self.ERROR_NO_VEHICLE
            details = self._unready_summary(
                request.requested_vehicle_id,
                require_waypoints=len(request.poses) > 1,
            )
            result.message = (
                f'no ready vehicle is available ({details})'
            )
            self.get_logger().warning(result.message)
            self._record_command_outcome(command_id, False)
            return result
        result.assigned_vehicle_id = vehicle_id
        runtime = self.vehicles[vehicle_id]
        self._set_active_target(vehicle_id, request.zone_id, request.poses[-1])
        leaving_zone_id = (
            runtime.locked_zone
            if (
                runtime.locked_zone in self.exclusive_zone_ids
                and request.zone_id != runtime.locked_zone
            )
            else ''
        )
        navigation_succeeded = False
        queued_zone_id = ''
        acquired_target_zone = False
        try:
            if self._requires_park_exit_maneuver(runtime, request.zone_id):
                self._publish_dispatch_feedback(
                    goal_handle,
                    vehicle_id,
                    'DRIVING_STRAIGHT_OUT_OF_PARKING_SPOT',
                    0,
                )
                self.get_logger().info(
                    f'{vehicle_id} leaving its parking spot: driving straight '
                    f'{self.park_exit_forward_distance_m:.2f}m before following '
                    'the destination path'
                )
                # The destination path starts inside the same tight pocket, so
                # the relaxation has to outlast the straight leg. The finally
                # block below and the periodic clearance check both put it back.
                await self._relax_park_exit_inflation(vehicle_id, runtime)
                if self.park_exit_open_loop:
                    exit_success, exit_message = (
                        await self._drive_straight_open_loop(
                            goal_handle,
                            vehicle_id,
                            self.park_exit_forward_distance_m,
                            self.park_exit_forward_speed_mps,
                            'parking exit forward motion',
                        )
                    )
                else:
                    exit_success, exit_message = await self._drive_straight(
                        goal_handle,
                        vehicle_id,
                        command_id,
                        self.park_exit_forward_distance_m,
                        self.park_exit_forward_speed_mps,
                        self.park_exit_behavior_timeout_sec,
                        'parking exit forward motion',
                    )
                if not exit_success:
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                        result.error_code = self.ERROR_CANCELED
                    else:
                        goal_handle.abort()
                        result.error_code = self.ERROR_NAV_FAILED
                    result.message = exit_message
                    return result
                with self._lock:
                    runtime.park_exit_forward_completed = True
                park_zone_id = self.park_zone_ids[vehicle_id]
                self._release_zone(vehicle_id, park_zone_id)
                if leaving_zone_id == park_zone_id:
                    leaving_zone_id = ''

            if self._requires_b1_exit_maneuver(runtime, request.zone_id):
                if not runtime.b1_exit_turn_completed:
                    self._publish_dispatch_feedback(
                        goal_handle,
                        vehicle_id,
                        'ROTATING_LEFT_BEFORE_B1_EXIT',
                        0,
                    )
                    self.get_logger().info(
                        f'{vehicle_id} leaving B-1: rotating left '
                        f'{self.b1_exit_left_turn_deg:.1f}deg in place before '
                        'translation'
                    )
                    turn_success, turn_message = (
                        await self._rotate_b1_exit_verified(
                            goal_handle,
                            vehicle_id,
                            command_id,
                            math.radians(self.b1_exit_left_turn_deg),
                        )
                    )
                    if not turn_success:
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            result.error_code = self.ERROR_CANCELED
                        else:
                            goal_handle.abort()
                            result.error_code = self.ERROR_NAV_FAILED
                        result.message = turn_message
                        return result
                    with self._lock:
                        runtime.b1_exit_turn_completed = True
                else:
                    self.get_logger().info(
                        f'{vehicle_id} B-1 left turn already completed; '
                        'resuming the exit sequence'
                    )

                if (
                    self.b1_exit_forward_distance_m > 0.0
                    and not runtime.b1_exit_forward_completed
                ):
                    self._publish_dispatch_feedback(
                        goal_handle,
                        vehicle_id,
                        'ADVANCING_BEFORE_B1_EXIT',
                        0,
                    )
                    self.get_logger().info(
                        f'{vehicle_id} leaving B-1: advancing '
                        f'{self.b1_exit_forward_distance_m:.2f}m after turn '
                        'before following the destination path'
                    )
                    forward_success, forward_message = (
                        await self._drive_forward(
                            goal_handle,
                            vehicle_id,
                            command_id,
                            self.b1_exit_forward_distance_m,
                        )
                    )
                    if not forward_success:
                        if goal_handle.is_cancel_requested:
                            goal_handle.canceled()
                            result.error_code = self.ERROR_CANCELED
                        else:
                            goal_handle.abort()
                            result.error_code = self.ERROR_NAV_FAILED
                        result.message = forward_message
                        return result
                    with self._lock:
                        runtime.b1_exit_forward_completed = True

            if request.zone_id in self.exclusive_zone_ids:
                self._maybe_clear_stale_zone(
                    request.zone_id, request.zone_visually_empty
                )
                queue_state = self._queue_zone_request(
                    vehicle_id,
                    command_id,
                    request.zone_id,
                )
                acquired_target_zone = queue_state == 'acquired'
                if queue_state == 'queued':
                    queued_zone_id = request.zone_id
                    if request.use_waiting_pose:
                        self._publish_dispatch_feedback(
                            goal_handle,
                            vehicle_id,
                            f'MOVING_TO_{request.zone_id}_WAITING_POSE',
                            0,
                        )
                        waiting_success, waiting_message = (
                            await self._navigate_to_waiting_pose(
                                goal_handle,
                                vehicle_id,
                                command_id,
                                request.waiting_pose,
                            )
                        )
                        if not waiting_success:
                            goal_handle.abort()
                            result.error_code = self.ERROR_NAV_FAILED
                            result.message = waiting_message
                            return result
                        if leaving_zone_id:
                            self.get_logger().info(
                                f'{vehicle_id} reached the waiting pose; '
                                f'releasing departed zone {leaving_zone_id}'
                            )
                            self._release_zone(
                                vehicle_id,
                                leaving_zone_id,
                            )
                            leaving_zone_id = ''
                        self._publish_dispatch_feedback(
                            goal_handle,
                            vehicle_id,
                            f'WAITING_NEAR_{request.zone_id}',
                            0,
                        )
                self._publish_dispatch_feedback(
                    goal_handle,
                    vehicle_id,
                    f'WAITING_FOR_{request.zone_id}',
                    0,
                )
                if not self._wait_for_zone(
                    goal_handle,
                    vehicle_id,
                    command_id,
                    request.zone_id,
                ):
                    if goal_handle.is_cancel_requested:
                        goal_handle.canceled()
                    else:
                        goal_handle.abort()
                    result.error_code = self.ERROR_CANCELED
                    result.message = (
                        f'canceled while waiting for {request.zone_id}'
                        if goal_handle.is_cancel_requested
                        else 'superseded by a newer vehicle command'
                    )
                    return result
                if queue_state == 'queued':
                    acquired_target_zone = True

            if self._command_preempted(command_id):
                goal_handle.abort()
                result.error_code = self.ERROR_CANCELED
                result.message = 'superseded by a newer vehicle command'
                return result
            # Last gate before the wheels turn: the zone is ours, but the
            # arm may still be over this vehicle's trailer.
            self._publish_dispatch_feedback(
                goal_handle,
                vehicle_id,
                'WAITING_FOR_ARM',
                0,
            )
            cargo_ok, cargo_state = self._wait_for_cargo_release(
                goal_handle,
                vehicle_id,
            )
            if not cargo_ok:
                if cargo_state == 'canceled':
                    goal_handle.canceled()
                    result.error_code = self.ERROR_CANCELED
                    result.message = (
                        'canceled while waiting for the arm to release '
                        f'{vehicle_id}'
                    )
                else:
                    goal_handle.abort()
                    result.error_code = self.ERROR_NAV_FAILED
                    result.message = (
                        'timed out waiting for the arm to release '
                        f'{vehicle_id}'
                    )
                self._record_command_outcome(command_id, False)
                return result
            self._publish_dispatch_feedback(
                goal_handle,
                vehicle_id,
                'DISPATCHED',
                0,
            )
            if len(request.poses) == 1:
                nav_goal = NavigateToPose.Goal()
                nav_goal.pose = self._latest_tf_pose(request.poses[0])
                client = self.nav_pose_clients[vehicle_id]
            else:
                nav_goal = NavigateThroughPoses.Goal()
                nav_goal.poses = [
                    self._latest_tf_pose(pose)
                    for pose in request.poses
                ]
                client = self.nav_waypoint_clients[vehicle_id]

            async def send_nav_goal():
                return await client.send_goal_async(
                    nav_goal,
                    feedback_callback=(
                        lambda message, count=len(request.poses):
                        self._relay_feedback(
                            goal_handle,
                            vehicle_id,
                            count,
                            message,
                        )
                    ),
                )

            nav_handle = await send_nav_goal()
            if not nav_handle.accepted:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_REJECTED
                result.message = 'vehicle Nav2 rejected the goal'
                return result
            with self._lock:
                runtime.active_nav_goal = nav_handle
            nav_result, nav_handle, stall_message = (
                await self._await_nav_result_watching_motion(
                    vehicle_id,
                    nav_handle,
                    send_nav_goal,
                )
            )
            if stall_message:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_FAILED
                result.message = stall_message
                return result
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result.error_code = self.ERROR_CANCELED
                result.message = 'navigation canceled'
                return result
            if self._command_preempted(command_id):
                goal_handle.abort()
                result.error_code = self.ERROR_CANCELED
                result.message = 'superseded by a newer vehicle command'
                return result
            if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_FAILED
                result.message = f'Nav2 finished with status {nav_result.status}'
                return result
            if request.zone_id in self.exclusive_zone_ids:
                with self._zone_condition:
                    if self._zone_owner[request.zone_id] == vehicle_id:
                        self._zone_entered[request.zone_id] = True
            goal_handle.succeed()
            navigation_succeeded = True
            result.success = True
            result.message = 'navigation completed'
            return result
        except Exception as exc:
            goal_handle.abort()
            result.error_code = self.ERROR_NAV_FAILED
            result.message = f'navigation exception: {exc}'
            return result
        finally:
            # Backstop for the periodic clearance check: whatever ended this
            # command - success, abort, cancel or exception - the vehicle must
            # not be left driving with a shrunken obstacle buffer.
            await self._restore_park_exit_inflation(vehicle_id, runtime)
            if queued_zone_id:
                self._discard_zone_request(command_id, queued_zone_id)
            with self._vehicle_condition:
                runtime.busy = False
                runtime.current_command_id = ''
                runtime.current_target_zone = ''
                runtime.current_target_pose = None
                runtime.active_nav_goal = None
                self._preempted_commands.discard(command_id)
                self._vehicle_condition.notify_all()
            if leaving_zone_id and navigation_succeeded:
                self._release_zone(vehicle_id, leaving_zone_id)
            if acquired_target_zone and not navigation_succeeded:
                self._release_zone(vehicle_id, request.zone_id)
            self._record_command_outcome(command_id, navigation_succeeded)

    async def _navigate_to_waiting_pose(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        waiting_pose,
    ):
        """Move one vehicle near an occupied zone before waiting for its lock."""
        return await self._navigate_single_pose(
            goal_handle,
            vehicle_id,
            command_id,
            waiting_pose,
            'zone waiting pose',
        )

    async def _navigate_single_pose(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        target_pose,
        phase,
    ):
        """Execute one Nav2 pose while preserving dispatcher cancellation."""
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = self._latest_tf_pose(target_pose)
        client = self.nav_pose_clients[vehicle_id]
        nav_handle = await client.send_goal_async(nav_goal)
        if not nav_handle.accepted:
            return False, f'vehicle Nav2 rejected the {phase}'
        with self._lock:
            self.vehicles[vehicle_id].active_nav_goal = nav_handle
        nav_result = await nav_handle.get_result_async()
        if goal_handle.is_cancel_requested:
            return False, f'canceled during {phase}'
        if self._command_preempted(command_id):
            return False, f'superseded during {phase}'
        if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            return (
                False,
                f'{phase} failed: Nav2 status={nav_result.status}',
            )
        with self._lock:
            self.vehicles[vehicle_id].active_nav_goal = None
        return True, ''

    async def _spin_in_place(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        target_yaw_rad,
    ):
        """Run Nav2 Spin so the B-1 exit turn cannot become a path arc."""
        goal = Spin.Goal()
        goal.target_yaw = float(target_yaw_rad)
        self._set_duration(
            goal.time_allowance,
            self.b1_exit_behavior_timeout_sec,
        )
        return await self._execute_behavior(
            goal_handle,
            vehicle_id,
            command_id,
            self.spin_clients[vehicle_id],
            goal,
            'B-1 exit rotation',
        )

    async def _rotate_b1_exit_verified(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        relative_yaw_rad,
    ):
        """Rotate, verify map-frame yaw, and correct before translation."""
        if not self.b1_exit_turn_verify:
            success, message = await self._spin_in_place(
                goal_handle,
                vehicle_id,
                command_id,
                relative_yaw_rad,
            )
            if (
                goal_handle.is_cancel_requested
                or self._command_preempted(command_id)
            ):
                return False, message or 'canceled during B-1 exit rotation'
            if not success:
                # Report and carry on: the forward move is what actually
                # clears the loading zone, and skipping it strands the
                # vehicle in B-1 holding the lock.
                self.get_logger().warning(
                    f'{vehicle_id} B-1 exit rotation reported "{message}"; '
                    'advancing anyway because turn verification is off'
                )
            return True, ''

        with self._lock:
            runtime = self.vehicles[vehicle_id]
            start_pose_time = runtime.pose_received_at
            start_yaw = self._pose_yaw(runtime.pose)
        target_yaw = self._normalize_angle(start_yaw + relative_yaw_rad)
        correction = float(relative_yaw_rad)

        for attempt in range(self.b1_exit_turn_max_corrections + 1):
            success, message = await self._spin_in_place(
                goal_handle,
                vehicle_id,
                command_id,
                correction,
            )
            if (
                not success
                and (
                    goal_handle.is_cancel_requested
                    or self._command_preempted(command_id)
                )
            ):
                return False, message

            if self.b1_exit_turn_settle_sec > 0.0:
                await self._sleep_async(self.b1_exit_turn_settle_sec)
            measured_pose = await self._wait_for_new_vehicle_pose(
                vehicle_id,
                start_pose_time,
                self.b1_exit_pose_update_timeout_sec,
            )
            if measured_pose is None:
                return (
                    False,
                    'B-1 exit rotation could not be verified: '
                    'no fresh AMCL pose',
                )

            with self._lock:
                start_pose_time = self.vehicles[vehicle_id].pose_received_at
            measured_yaw = self._pose_yaw(measured_pose)
            yaw_error = self._normalize_angle(target_yaw - measured_yaw)
            self.get_logger().info(
                f'{vehicle_id} B-1 turn verification: '
                f'target={math.degrees(target_yaw):.1f}deg, '
                f'measured={math.degrees(measured_yaw):.1f}deg, '
                f'error={math.degrees(yaw_error):.1f}deg, '
                f'attempt={attempt + 1}'
            )
            if abs(yaw_error) <= self.b1_exit_turn_tolerance_rad:
                if not success:
                    self.get_logger().warning(
                        f'{vehicle_id} Spin action reported failure, but '
                        'fresh AMCL heading confirms the requested B-1 '
                        'exit turn; continuing safely'
                    )
                return True, ''
            if not success and abs(yaw_error) > math.radians(45.0):
                return (
                    False,
                    f'{message}; measured heading error remains '
                    f'{math.degrees(yaw_error):.1f}deg',
                )
            correction = yaw_error

        return (
            False,
            'B-1 exit rotation did not reach the required heading: '
            f'error={math.degrees(correction):.1f}deg',
        )

    async def _await_nav_result_watching_motion(
        self,
        vehicle_id,
        nav_handle,
        resend,
    ):
        """Await Nav2's result, re-sending the goal if the vehicle never moves.

        Nav2 reports a goal as executing whether or not the wheels turn, so a
        latched hold or a wedged controller looks identical to normal progress
        until the action times out minutes later. Returns
        (result, handle, stall_message); stall_message is empty unless the
        vehicle stopped moving and the re-sends were used up.
        """
        resends_used = 0
        self._reset_stall_clock(vehicle_id)

        while True:
            result_future = nav_handle.get_result_async()
            resend_pending = False

            while not result_future.done():
                await self._sleep_async(0.2)
                now = time.monotonic()
                with self._lock:
                    runtime = self.vehicles[vehicle_id]
                    stalled_for = now - (runtime.last_motion_at or now)
                held = self._vehicle_is_deliberately_stopped(vehicle_id)
                verdict = classify_motion_stall(
                    stalled_for_sec=stalled_for,
                    stall_timeout_sec=self.motion_stall_timeout_sec,
                    resends_used=resends_used,
                    max_resends=self.max_motion_resends,
                    held=held,
                )
                if verdict == STALL_MOVING:
                    continue
                if verdict == STALL_HELD:
                    # A hold must not burn the stall budget; the clock starts
                    # again from the moment the vehicle is released.
                    self._reset_stall_clock(vehicle_id)
                    continue
                if verdict == STALL_EXHAUSTED:
                    self.get_logger().error(
                        f'{vehicle_id} has not moved for {stalled_for:.1f}s '
                        f'after {resends_used} re-send(s); giving up'
                    )
                    nav_handle.cancel_goal_async()
                    return None, nav_handle, (
                        f'vehicle did not move after {resends_used} re-send(s)'
                    )

                resends_used += 1
                self.get_logger().warning(
                    f'{vehicle_id} accepted the goal but has not moved for '
                    f'{stalled_for:.1f}s; re-sending '
                    f'({resends_used}/{self.max_motion_resends})'
                )
                # Wait for the cancellation to land. bt_navigator muxes its
                # navigators and rejects a new goal while one is still
                # running, so re-sending too early is refused outright.
                # cancel_goal_async only delivers the request; the goal is
                # not gone until its result arrives. Re-sending in between
                # races bt_navigator's teardown, and its action server then
                # dies on an uncaught "Failed to accept new goal".
                cancel_response = await nav_handle.cancel_goal_async()
                if not cancel_response.goals_canceling:
                    return None, nav_handle, (
                        'Nav2 did not accept cancellation; goal was not re-sent'
                    )
                settle_deadline = (
                    time.monotonic() + self.cancel_settle_timeout_sec
                )
                while (
                    not result_future.done()
                    and time.monotonic() < settle_deadline
                ):
                    await self._sleep_async(0.05)
                if not result_future.done():
                    self.get_logger().error(
                        f'{vehicle_id} cancellation did not settle within '
                        f'{self.cancel_settle_timeout_sec:.1f}s; refusing to '
                        're-send because it could abort bt_navigator'
                    )
                    return None, nav_handle, (
                        'Nav2 cancellation did not finish; goal was not re-sent'
                    )
                new_handle = await resend()
                if new_handle is None or not new_handle.accepted:
                    return None, nav_handle, 'Nav2 rejected the re-sent goal'
                nav_handle = new_handle
                with self._lock:
                    self.vehicles[vehicle_id].active_nav_goal = nav_handle
                self._reset_stall_clock(vehicle_id)
                resend_pending = True
                break

            # Only the current handle's result counts; the future belonging to
            # a cancelled handle completes too, and returning it would report
            # the re-sent goal as cancelled.
            if resend_pending:
                continue
            return result_future.result(), nav_handle, ''

    def _reset_stall_clock(self, vehicle_id):
        with self._lock:
            self.vehicles[vehicle_id].last_motion_at = time.monotonic()

    async def _wait_for_new_vehicle_pose(
        self,
        vehicle_id,
        previous_pose_time,
        timeout_sec,
    ):
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            with self._lock:
                runtime = self.vehicles[vehicle_id]
                pose_time = runtime.pose_received_at
                if (
                    pose_time is not None
                    and (
                        previous_pose_time is None
                        or pose_time > previous_pose_time
                    )
                ):
                    return copy.deepcopy(runtime.pose)
            await self._sleep_async(0.05)
        return None

    async def _sleep_async(self, seconds):
        """Yield a coroutine using an rclpy timer, not an asyncio event loop."""
        if seconds <= 0.0:
            return
        future = Future(executor=self.executor)
        timer = None

        def wake():
            if not future.done():
                future.set_result(True)

        timer = self.create_timer(
            float(seconds),
            wake,
            callback_group=self.callback_group,
        )
        try:
            await future
        finally:
            if timer is not None:
                self.destroy_timer(timer)

    async def _drive_forward(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        distance_m,
    ):
        """Drive straight along the vehicle x-axis after the B-1 turn."""
        return await self._drive_straight(
            goal_handle,
            vehicle_id,
            command_id,
            distance_m,
            self.b1_exit_forward_speed_mps,
            self.b1_exit_behavior_timeout_sec,
            'B-1 exit forward motion',
        )

    async def _drive_straight_open_loop(
        self,
        goal_handle,
        vehicle_id,
        distance_m,
        speed_mps,
        phase,
    ):
        """Crawl straight out of the parking pocket without Nav2's checks.

        The pocket is tighter than the footprint's inscribed radius, so
        DriveOnHeading always reports COLLISION_AHEAD there and the vehicle can
        never leave. Parking in is already an open-loop timed move
        (parking_new.reverse_to_parked), so the exit mirrors it.

        This bypasses costmap collision checking. The velocity still goes
        through cmd_vel_safety_gate, so the emergency-stop and collision-hold
        latches keep working.
        """
        publisher = self.park_exit_cmd_publishers[vehicle_id]
        duration_sec = float(distance_m) / max(float(speed_mps), 1e-6)
        command = Twist()
        command.linear.x = float(speed_mps)

        self.get_logger().info(
            f'{vehicle_id} {phase}: open-loop {distance_m:.2f}m at '
            f'{speed_mps:.3f} m/s ({duration_sec:.1f}s), costmap checks '
            'bypassed'
        )
        period = 1.0 / self.park_exit_open_loop_rate_hz
        deadline = time.monotonic() + duration_sec
        try:
            while time.monotonic() < deadline:
                if goal_handle.is_cancel_requested:
                    return False, f'{phase} canceled'
                with self._lock:
                    if self.vehicles[vehicle_id].emergency:
                        return False, f'{phase} stopped by emergency latch'
                publisher.publish(command)
                await self._sleep_async(period)
        finally:
            # The gate also times out on its own, but do not rely on that.
            publisher.publish(Twist())
        return True, ''

    async def _relax_park_exit_inflation(self, vehicle_id, runtime):
        """Shrink the rolling costmap inflation for the parking-pocket exit.

        Reads the live value first and refuses to relax when it cannot be read
        back, so the vehicle never drives with the obstacle buffer removed and
        no recorded value to restore.
        """
        if runtime.park_exit_inflation_restore_m is not None:
            return True

        get_client = self.local_costmap_get_param_clients[vehicle_id]
        set_client = self.local_costmap_set_param_clients[vehicle_id]
        if not get_client.service_is_ready() or not set_client.service_is_ready():
            self.get_logger().warning(
                f'{vehicle_id} local costmap parameter services unavailable; '
                'leaving inflation untouched for the parking exit'
            )
            return False

        request = GetParameters.Request()
        request.names = ['inflation_layer.inflation_radius']
        response = await get_client.call_async(request)
        if not response.values or response.values[0].type != (
            ParameterType.PARAMETER_DOUBLE
        ):
            self.get_logger().warning(
                f'{vehicle_id} could not read inflation_radius; leaving it '
                'untouched for the parking exit'
            )
            return False
        current = float(response.values[0].double_value)

        if not await self._write_local_inflation(
            vehicle_id, self.park_exit_inflation_radius_m
        ):
            return False

        with self._lock:
            runtime.park_exit_inflation_restore_m = current
            position = runtime.pose.pose.position
            runtime.park_exit_origin_xy = (
                float(position.x), float(position.y)
            )
        self.get_logger().warning(
            f'{vehicle_id} parking exit: local costmap inflation '
            f'{current:.3f} -> {self.park_exit_inflation_radius_m:.3f} m '
            'until it clears the pocket'
        )
        return True

    async def _restore_park_exit_inflation(self, vehicle_id, runtime):
        """Put the rolling costmap inflation back after the pocket exit."""
        with self._lock:
            restore_m = runtime.park_exit_inflation_restore_m
        if restore_m is None:
            return
        if await self._write_local_inflation(vehicle_id, restore_m):
            self.get_logger().info(
                f'{vehicle_id} parking exit complete: local costmap '
                f'inflation restored to {restore_m:.3f} m'
            )
        else:
            self.get_logger().error(
                f'{vehicle_id} FAILED to restore local costmap inflation to '
                f'{restore_m:.3f} m; obstacle clearance stays reduced until '
                'the costmap is reconfigured'
            )
        with self._lock:
            runtime.park_exit_inflation_restore_m = None
            runtime.park_exit_origin_xy = None

    async def _write_local_inflation(self, vehicle_id, radius_m):
        client = self.local_costmap_set_param_clients[vehicle_id]
        if not client.service_is_ready():
            return False
        parameter = Parameter()
        parameter.name = 'inflation_layer.inflation_radius'
        parameter.value = ParameterValue()
        parameter.value.type = ParameterType.PARAMETER_DOUBLE
        parameter.value.double_value = float(radius_m)
        request = SetParameters.Request()
        request.parameters = [parameter]
        try:
            response = await client.call_async(request)
        except Exception as exc:  # noqa: BLE001 - ROS future exception
            self.get_logger().error(
                f'{vehicle_id} inflation_radius set failed: {exc}'
            )
            return False
        return bool(response.results and response.results[0].successful)

    async def _drive_straight(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        distance_m,
        speed_mps,
        timeout_sec,
        phase,
    ):
        """Run a forced straight DriveOnHeading motion before Nav2 planning."""
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m)
        goal.speed = float(speed_mps)
        self._set_duration(goal.time_allowance, timeout_sec)
        return await self._execute_behavior(
            goal_handle,
            vehicle_id,
            command_id,
            self.drive_on_heading_clients[vehicle_id],
            goal,
            phase,
        )

    async def _execute_behavior(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        client,
        behavior_goal,
        phase,
    ):
        if not client.server_is_ready():
            return False, f'{phase} action server is unavailable'
        behavior_handle = await client.send_goal_async(behavior_goal)
        if not behavior_handle.accepted:
            return False, f'Nav2 rejected the {phase}'
        with self._lock:
            self.vehicles[vehicle_id].active_nav_goal = behavior_handle
        behavior_result = await behavior_handle.get_result_async()
        if goal_handle.is_cancel_requested:
            return False, f'canceled during {phase}'
        if self._command_preempted(command_id):
            return False, f'superseded during {phase}'
        if behavior_result.status != GoalStatus.STATUS_SUCCEEDED:
            result = behavior_result.result
            detail = getattr(result, 'error_msg', '') or 'no detail'
            error_code = getattr(result, 'error_code', 0)
            return (
                False,
                f'{phase} failed: status={behavior_result.status}, '
                f'error_code={error_code}, message={detail}',
            )
        with self._lock:
            self.vehicles[vehicle_id].active_nav_goal = None
        return True, ''

    @staticmethod
    def _set_duration(duration, seconds):
        whole_seconds = int(seconds)
        duration.sec = whole_seconds
        duration.nanosec = int((float(seconds) - whole_seconds) * 1e9)

    @staticmethod
    def _requires_b1_exit_turn(locked_zone, target_zone):
        return locked_zone == 'B-1' and target_zone != 'B-1'

    def _requires_b1_exit_maneuver(self, runtime, target_zone):
        """Keep the exit maneuver active despite a slightly early lock release."""
        if target_zone == 'B-1':
            return False
        if runtime.locked_zone == 'B-1':
            return self._vehicle_is_at_zone(
                runtime,
                'B-1',
                radius_m=self.b1_exit_detection_radius_m,
            )
        with self._lock:
            has_b1_reference = 'B-1' in self._zone_target_poses
        return (
            has_b1_reference
            and self._vehicle_is_at_zone(
                runtime,
                'B-1',
                radius_m=self.b1_exit_detection_radius_m,
            )
        )

    def _requires_park_exit_maneuver(self, runtime, target_zone):
        """Require one straight exit when leaving this vehicle's own spot."""
        park_zone_id = self.park_zone_ids[runtime.vehicle_id]
        if (
            target_zone == park_zone_id
            or runtime.park_exit_forward_completed
        ):
            return False
        if runtime.locked_zone == park_zone_id:
            return True
        return self._vehicle_is_at_zone(
            runtime,
            park_zone_id,
            radius_m=self.park_exit_detection_radius_m,
        )

    def _check_park_exit_inflation(self):
        """Restore inflation as soon as a vehicle is clear of its pocket.

        The command's finally block is the last resort; this returns the
        obstacle buffer for the rest of a long trip instead of holding it
        reduced all the way to the destination.
        """
        to_restore = []
        with self._lock:
            for vehicle_id, runtime in self.vehicles.items():
                origin = runtime.park_exit_origin_xy
                if (
                    origin is None
                    or runtime.park_exit_inflation_restore_m is None
                ):
                    continue
                travelled = math.hypot(
                    runtime.pose.pose.position.x - origin[0],
                    runtime.pose.pose.position.y - origin[1],
                )
                if travelled >= self.park_exit_inflation_clear_distance_m:
                    to_restore.append((vehicle_id, runtime, travelled))
        for vehicle_id, runtime, travelled in to_restore:
            self.get_logger().info(
                f'{vehicle_id} moved {travelled:.2f}m from its parking spot; '
                'restoring local costmap inflation'
            )
            self.executor.create_task(
                self._restore_park_exit_inflation(vehicle_id, runtime)
            )

    def _vehicle_is_at_zone(self, runtime, zone_id, radius_m=None):
        """Reject a false exit turn when a reserved vehicle is still en route."""
        with self._lock:
            target_pose = self._zone_target_poses.get(zone_id)
        if target_pose is None:
            # Preserve the mandatory exit behavior after a dispatcher restart,
            # where an operator or recovered lock may not have a cached pose.
            return True
        radius = (
            self.zone_occupancy_radius_m
            if radius_m is None
            else float(radius_m)
        )
        return math.hypot(
            runtime.pose.pose.position.x - target_pose.pose.position.x,
            runtime.pose.pose.position.y - target_pose.pose.position.y,
        ) <= radius

    @staticmethod
    def _pose_yaw(pose_stamped):
        orientation = pose_stamped.pose.orientation
        return math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )

    @staticmethod
    def _normalize_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _b1_exit_turn_pose(current_pose, left_turn_deg=90.0):
        """Create a same-position target with a positive (left/CCW) yaw turn."""
        pose = copy.deepcopy(current_pose)
        pose.header.frame_id = pose.header.frame_id or 'map'
        orientation = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        target_yaw = yaw + math.radians(left_turn_deg)
        orientation.x = 0.0
        orientation.y = 0.0
        orientation.z = math.sin(target_yaw * 0.5)
        orientation.w = math.cos(target_yaw * 0.5)
        return pose

    @staticmethod
    def _forward_pose(current_pose, distance_m):
        """Move a pose forward along its current yaw without changing yaw."""
        pose = copy.deepcopy(current_pose)
        orientation = pose.pose.orientation
        yaw = math.atan2(
            2.0 * (
                orientation.w * orientation.z
                + orientation.x * orientation.y
            ),
            1.0 - 2.0 * (
                orientation.y * orientation.y
                + orientation.z * orientation.z
            ),
        )
        pose.pose.position.x += math.cos(yaw) * float(distance_m)
        pose.pose.position.y += math.sin(yaw) * float(distance_m)
        return pose

    @staticmethod
    def _latest_tf_pose(source):
        pose = PoseStamped()
        pose.header.frame_id = source.header.frame_id or 'map'
        pose.pose = source.pose
        return pose

    def _on_park_request(self, message):
        """Kick off an async park dispatch from a synchronous topic callback."""
        vehicle_id = str(message.data).strip().strip('/')
        self.executor.create_task(self._dispatch_park(vehicle_id))

    def _check_auto_park(self):
        """Send any vehicle that has been idle long enough to its own spot."""
        now = time.monotonic()
        to_park = []
        with self._lock:
            for vehicle_id, runtime in self.vehicles.items():
                ready = self._vehicle_ready(vehicle_id)
                already_parked = (
                    runtime.locked_zone == self.park_zone_ids[vehicle_id]
                )
                if not ready or already_parked:
                    self._idle_since[vehicle_id] = None
                    continue
                if self._idle_since[vehicle_id] is None:
                    self._idle_since[vehicle_id] = now
                    continue
                if now - self._idle_since[vehicle_id] >= self.auto_park_idle_sec:
                    to_park.append(vehicle_id)
        for vehicle_id in to_park:
            self.get_logger().info(
                f'{vehicle_id} idle for {self.auto_park_idle_sec:.0f}s; '
                'auto-parking'
            )
            self.executor.create_task(self._dispatch_park(vehicle_id))

    async def _dispatch_park(self, requested_vehicle_id):
        """
        Reserve a ready vehicle and run its own dedicated ParkInSpot action.

        Each vehicle has its own fixed, non-shared spot (self.park_zone_ids /
        self.park_spot_ids), so there is never real contention between agv1
        and agv2 here. Still fire-and-forget: triggered from a topic
        (explicit "park" command) or the idle watchdog, neither of which
        holds a cancelable action goal_handle.
        """
        requested_vehicle_id = requested_vehicle_id.strip('/')
        if requested_vehicle_id and requested_vehicle_id not in self.vehicles:
            self.get_logger().error(
                f'park request ignored: unknown vehicle_id={requested_vehicle_id!r}'
            )
            return

        with self._lock:
            candidates = (
                [requested_vehicle_id]
                if requested_vehicle_id
                else sorted(self.vehicles)
            )
            vehicle_id = next(
                (
                    candidate for candidate in candidates
                    if self._vehicle_ready(candidate)
                    and self.vehicles[candidate].locked_zone
                    != self.park_zone_ids[candidate]
                ),
                '',
            )
            if not vehicle_id:
                detail_items = []
                for candidate in candidates:
                    candidate_runtime = self.vehicles[candidate]
                    reasons = []
                    if candidate_runtime.locked_zone == self.park_zone_ids[candidate]:
                        reasons.append('already parked/zone locked')
                    if candidate_runtime.busy:
                        reasons.append('busy')
                    if not reasons and not self._vehicle_operational(candidate):
                        reasons.extend(self._vehicle_unready_reasons(candidate))
                    detail_items.append(
                        f'{candidate}: {", ".join(reasons or ["unavailable"])}'
                    )
                details = '; '.join(detail_items)
                self.get_logger().warning(
                    'Park request ignored: no ready, not-already-parked '
                    f'vehicle available ({details})',
                    throttle_duration_sec=5.0,
                )
                return
            runtime = self.vehicles[vehicle_id]
            runtime.busy = True
            command_id = f'park-{time.time_ns()}'
            runtime.current_command_id = command_id

        zone_id = self.park_zone_ids[vehicle_id]
        spot_id = self.park_spot_ids[vehicle_id]
        acquired_zone = False
        parking_succeeded = False
        try:
            queue_state = self._queue_zone_request(vehicle_id, command_id, zone_id)
            acquired_zone = queue_state == 'acquired'
            if queue_state == 'queued':
                with self._lock:
                    queue = self._zone_queue[zone_id]
                    if command_id in queue:
                        queue.remove(command_id)
                self.get_logger().error(
                    f'{vehicle_id} park deferred: {zone_id} unexpectedly '
                    'occupied by another vehicle'
                )
                return

            client = self.park_clients[vehicle_id]
            if not client.wait_for_server(
                timeout_sec=self.park_action_wait_timeout_sec
            ):
                self.get_logger().error(
                    f'{vehicle_id} /park_in_spot action server unavailable '
                    f'after {self.park_action_wait_timeout_sec:.1f}s'
                )
                return
            goal = ParkInSpot.Goal()
            goal.spot_id = spot_id
            # Nav2 aborting once mid-parking is routine -- a recovery cycle
            # runs out, or the controller gives up on a plan -- and the
            # sequence then stops dead until someone re-issues the command by
            # hand. Retry it here instead. Each attempt re-plans from wherever
            # the vehicle actually is, so a partial run is safe to repeat.
            attempts = 1 + self.max_park_retries
            for attempt in range(1, attempts + 1):
                goal_handle = await client.send_goal_async(goal)
                if not goal_handle.accepted:
                    self.get_logger().error(
                        f'{vehicle_id} park_in_spot goal rejected'
                    )
                    return
                with self._lock:
                    runtime.active_nav_goal = goal_handle
                park_result = await goal_handle.get_result_async()
                with self._lock:
                    runtime.active_nav_goal = None
                result = park_result.result

                if result.success:
                    parking_succeeded = True
                    with self._lock:
                        runtime.park_exit_forward_completed = False
                    self.get_logger().info(
                        f'{vehicle_id} parked at {spot_id} on attempt '
                        f'{attempt}/{attempts}: {result.message}'
                    )
                    break

                if self._command_preempted(command_id):
                    self.get_logger().info(
                        f'{vehicle_id} parking superseded by a newer command'
                    )
                    break
                # Retrying into a latched hold just burns the budget; the
                # vehicle is stopped on purpose and will not move either way.
                if self._vehicle_is_deliberately_stopped(vehicle_id):
                    self.get_logger().error(
                        f'{vehicle_id} parking failed while held: '
                        f'{result.message}'
                    )
                    break
                if attempt >= attempts:
                    self.get_logger().error(
                        f'{vehicle_id} parking failed after {attempts} '
                        f'attempt(s): {result.message}'
                    )
                    break
                self.get_logger().warning(
                    f'{vehicle_id} parking attempt {attempt}/{attempts} '
                    f'failed: {result.message}; retrying'
                )
                await self._sleep_async(self.park_retry_backoff_sec)
        finally:
            if acquired_zone and not parking_succeeded:
                self._release_zone(vehicle_id, zone_id)
            with self._lock:
                runtime.busy = False
                runtime.current_command_id = ''

    @staticmethod
    def _publish_dispatch_feedback(
        goal_handle,
        vehicle_id,
        state,
        current_waypoint,
    ):
        feedback = DispatchNavigation.Feedback()
        feedback.assigned_vehicle_id = vehicle_id
        feedback.state = state
        feedback.current_waypoint = current_waypoint
        goal_handle.publish_feedback(feedback)

    def _relay_feedback(
        self,
        goal_handle,
        vehicle_id,
        pose_count,
        nav_feedback,
    ):
        remaining = getattr(
            nav_feedback.feedback,
            'number_of_poses_remaining',
            pose_count,
        )
        waypoint = max(0, min(pose_count - 1, pose_count - remaining))
        self._publish_dispatch_feedback(
            goal_handle,
            vehicle_id,
            'NAVIGATING',
            waypoint,
        )

    def _request_gate(self, vehicle_id, enabled):
        client = self.gate_clients[vehicle_id]
        if not client.service_is_ready():
            return False
        request = SetBool.Request()
        request.data = enabled
        client.call_async(request)
        runtime = self.vehicles[vehicle_id]
        runtime.emergency = enabled
        if enabled and runtime.active_nav_goal is not None:
            runtime.active_nav_goal.cancel_goal_async()
        return True

    def _set_emergency_service(self, vehicle_id, request, response):
        with self._lock:
            accepted = self._request_gate(vehicle_id, bool(request.data))
        response.success = accepted
        response.message = (
            f'{vehicle_id} emergency state={bool(request.data)} accepted'
            if accepted
            else f'{vehicle_id} safety gate is unavailable'
        )
        return response

    def _set_all_emergency_service(self, request, response):
        with self._lock:
            accepted = [
                self._request_gate(vehicle_id, bool(request.data))
                for vehicle_id in self.vehicles
            ]
        response.success = all(accepted)
        response.message = (
            f'fleet emergency state={bool(request.data)}; '
            f'accepted={sum(accepted)}/{len(accepted)}'
        )
        return response

    def _clear_zone_lock(self, zone_id, _request, response):
        with self._zone_condition:
            owner = self._zone_owner[zone_id]
            if owner and self.vehicles[owner].busy:
                response.success = False
                response.message = f'{zone_id} owner is still executing a command'
                return response
            if owner:
                self.vehicles[owner].locked_zone = ''
            self._zone_owner[zone_id] = ''
            self._zone_unknown[zone_id] = False
            self._zone_entered[zone_id] = False
            self._zone_condition.notify_all()
        response.success = True
        response.message = f'{zone_id} lock cleared by operator'
        return response

    def _publish_states(self):
        self._recover_startup_zone_owners()
        self._refresh_zone_occupancy()
        now = time.monotonic()
        markers = MarkerArray()
        if self._marker_cleanup_pending:
            cleanup = Marker()
            cleanup.action = Marker.DELETEALL
            markers.markers.append(cleanup)
            self._marker_cleanup_pending = False
        with self._lock:
            for marker_id, (vehicle_id, runtime) in enumerate(
                self.vehicles.items()
            ):
                message = VehicleState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = 'map'
                message.vehicle_id = vehicle_id
                age = (
                    math.inf
                    if runtime.telemetry_received_at is None
                    else now - runtime.telemetry_received_at
                )
                for zone_id in self.exclusive_zone_ids:
                    if (
                        self._zone_owner[zone_id] == vehicle_id
                        and age > self.telemetry_timeout
                    ):
                        self._zone_unknown[zone_id] = True
                nav_ready = self.nav_pose_clients[vehicle_id].server_is_ready()
                if runtime.emergency:
                    state = VehicleState.EMERGENCY_STOPPED
                    text = 'EMERGENCY_STOPPED'
                elif age > self.telemetry_timeout:
                    state = VehicleState.OFFLINE
                    text = 'OFFLINE'
                elif not runtime.has_amcl_pose:
                    state = VehicleState.ERROR
                    text = 'WAITING_FOR_INITIAL_POSE'
                elif runtime.busy:
                    state = VehicleState.BUSY
                    text = 'BUSY'
                elif nav_ready:
                    state = VehicleState.READY
                    text = 'READY'
                else:
                    state = VehicleState.ERROR
                    text = 'NAV2_INACTIVE'
                message.state = state
                message.state_text = text
                message.battery_percent = runtime.battery_percent
                message.battery_voltage = runtime.battery_voltage
                message.pose = runtime.pose
                message.current_command_id = runtime.current_command_id
                message.nav2_ready = nav_ready
                message.emergency_stopped = runtime.emergency
                message.locked_zone = runtime.locked_zone
                message.telemetry_age_sec = float(age)
                self.state_publishers[vehicle_id].publish(message)
                if runtime.has_amcl_pose:
                    markers.markers.extend(
                        self._vehicle_markers(
                            marker_id,
                            runtime,
                            message.header.stamp,
                            age <= self.telemetry_timeout,
                        )
                    )
            zone = String()
            zone.data = ';'.join(
                (
                    f'{zone_id}:UNKNOWN:{self._zone_owner[zone_id]}'
                    if self._zone_unknown[zone_id]
                    else f'{zone_id}:{self._zone_owner[zone_id] or "FREE"}'
                )
                for zone_id in self.exclusive_zone_ids
            )
            self.zone_publisher.publish(zone)
            self.marker_publisher.publish(markers)

    def _refresh_zone_occupancy(self):
        """
        Release occupied zones as soon as their vehicle physically exits.

        A zone must first be entered before an outside pose can release it.
        This prevents a reservation made while approaching the zone from being
        cleared immediately. A hysteresis margin avoids lock chatter near the
        occupancy boundary.
        """
        now = time.monotonic()
        release_radius = (
            self.zone_occupancy_radius_m + self.zone_release_hysteresis_m
        )
        with self._zone_condition:
            for zone_id in self.exclusive_zone_ids:
                owner = self._zone_owner[zone_id]
                target_pose = self._zone_target_poses.get(zone_id)
                if not owner or target_pose is None:
                    continue
                runtime = self.vehicles[owner]
                telemetry_fresh = (
                    runtime.telemetry_received_at is not None
                    and now - runtime.telemetry_received_at
                    <= self.telemetry_timeout
                )
                if not telemetry_fresh or not runtime.has_amcl_pose:
                    continue
                distance = math.hypot(
                    runtime.pose.pose.position.x
                    - target_pose.pose.position.x,
                    runtime.pose.pose.position.y
                    - target_pose.pose.position.y,
                )
                if distance <= self.zone_occupancy_radius_m:
                    if not self._zone_entered[zone_id]:
                        if zone_id == 'B-1':
                            runtime.b1_exit_turn_completed = False
                            runtime.b1_exit_forward_completed = False
                        if zone_id == self.park_zone_ids.get(owner):
                            runtime.park_exit_forward_completed = False
                    self._zone_entered[zone_id] = True
                    continue
                if (
                    self._zone_entered[zone_id]
                    and distance >= release_radius
                ):
                    self._zone_owner[zone_id] = ''
                    self._zone_unknown[zone_id] = False
                    self._zone_entered[zone_id] = False
                    if runtime.locked_zone == zone_id:
                        runtime.locked_zone = ''
                    self.get_logger().info(
                        f'{owner} exited {zone_id}: released zone lock at '
                        f'distance={distance:.3f}m'
                    )
                    self._zone_condition.notify_all()

    @staticmethod
    def _vehicle_markers(marker_id, runtime, stamp, online):
        alpha = 1.0 if online else 0.35

        label = Marker()
        label.header.stamp = stamp
        label.header.frame_id = 'map'
        label.ns = 'fleet_vehicle_label'
        label.id = marker_id * 2 + 1
        label.type = Marker.TEXT_VIEW_FACING
        label.action = Marker.ADD
        label.pose.position = copy.deepcopy(runtime.pose.pose.position)
        label.pose.position.z = 0.16
        label.pose.orientation.w = 1.0
        label.scale.z = 0.10
        label.color.r = 1.0
        label.color.g = 1.0
        label.color.b = 1.0
        label.color.a = alpha
        label.text = runtime.vehicle_id.upper()
        return (label,)


def main(args=None):
    rclpy.init(args=args)
    node = FleetDispatcher()
    executor = MultiThreadedExecutor(num_threads=6)
    executor.add_node(node)
    try:
        executor.spin()
    except (ExternalShutdownException, KeyboardInterrupt):
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
