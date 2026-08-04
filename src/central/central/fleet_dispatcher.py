#!/usr/bin/env python3

"""Two-vehicle Nav2 dispatcher with exclusive-zone queues and emergency control."""

from __future__ import annotations

import asyncio
import copy
from dataclasses import dataclass, field
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import (
    DriveOnHeading,
    NavigateThroughPoses,
    NavigateToPose,
    Spin,
)
from nav_msgs.msg import Odometry
from porter_interfaces.action import DispatchNavigation
from porter_interfaces.msg import VehicleState
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
    locked_zone: str = ''
    active_nav_goal: object | None = None


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
        self.declare_parameter('exclusive_zone_ids', ['B-1', 'A'])
        self.declare_parameter('zone_occupancy_radius_m', 0.18)
        self.declare_parameter('zone_release_hysteresis_m', 0.05)
        self.declare_parameter('b1_exit_left_turn_deg', 90.0)
        self.declare_parameter('b1_exit_forward_distance_m', 0.10)
        self.declare_parameter('b1_exit_forward_speed_mps', 0.05)
        self.declare_parameter('b1_exit_behavior_timeout_sec', 10.0)
        self.declare_parameter('b1_exit_detection_radius_m', 0.35)
        self.declare_parameter('b1_exit_turn_tolerance_deg', 5.0)
        self.declare_parameter('b1_exit_turn_max_corrections', 2)
        self.declare_parameter('b1_exit_pose_update_timeout_sec', 3.0)
        self.declare_parameter('b1_exit_turn_settle_sec', 0.5)
        self.declare_parameter('sequence_dependency_timeout_sec', 300.0)
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
        self.sequence_dependency_timeout_sec = float(
            self.get_parameter('sequence_dependency_timeout_sec').value
        )
        if self.sequence_dependency_timeout_sec <= 0.0:
            raise ValueError(
                'sequence_dependency_timeout_sec must be positive'
            )
        self.subscribe_odom_fallback = bool(
            self.get_parameter('subscribe_odom_fallback').value
        )
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
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
        self.gate_clients = {}
        self.state_publishers = {}
        for vehicle_id in vehicle_ids:
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

        self.zone_publisher = self.create_publisher(
            String, '/central/fleet/zones', 10
        )
        self.marker_publisher = self.create_publisher(
            MarkerArray, '/central/fleet/vehicle_markers', 10
        )
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
        self.get_logger().info(
            'Fleet dispatcher ready: vehicles=agv1,agv2, B-1 lock enabled'
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
            runtime.telemetry_received_at = time.monotonic()
            if runtime.has_amcl_pose or not self.subscribe_odom_fallback:
                return
            runtime.pose.header = message.header
            runtime.pose.pose = message.pose.pose
            runtime.pose_received_at = time.monotonic()

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
                if telemetry_lost:
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
            if self._requires_b1_exit_maneuver(runtime, request.zone_id):
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
                turn_success, turn_message = await self._rotate_b1_exit_verified(
                    goal_handle,
                    vehicle_id,
                    command_id,
                    math.radians(self.b1_exit_left_turn_deg),
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

                if self.b1_exit_forward_distance_m > 0.0:
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

            send_future = client.send_goal_async(
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
            nav_handle = await send_future
            if not nav_handle.accepted:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_REJECTED
                result.message = 'vehicle Nav2 rejected the goal'
                return result
            with self._lock:
                runtime.active_nav_goal = nav_handle
            nav_result = await nav_handle.get_result_async()
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
            if queued_zone_id:
                self._discard_zone_request(command_id, queued_zone_id)
            with self._vehicle_condition:
                runtime.busy = False
                runtime.current_command_id = ''
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
            if not success:
                return False, message

            if self.b1_exit_turn_settle_sec > 0.0:
                await asyncio.sleep(self.b1_exit_turn_settle_sec)
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
                return True, ''
            correction = yaw_error

        return (
            False,
            'B-1 exit rotation did not reach the required heading: '
            f'error={math.degrees(correction):.1f}deg',
        )

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
            await asyncio.sleep(0.05)
        return None

    async def _drive_forward(
        self,
        goal_handle,
        vehicle_id,
        command_id,
        distance_m,
    ):
        """Drive straight along the vehicle x-axis after the B-1 turn."""
        goal = DriveOnHeading.Goal()
        goal.target.x = float(distance_m)
        goal.speed = float(self.b1_exit_forward_speed_mps)
        self._set_duration(
            goal.time_allowance,
            self.b1_exit_behavior_timeout_sec,
        )
        return await self._execute_behavior(
            goal_handle,
            vehicle_id,
            command_id,
            self.drive_on_heading_clients[vehicle_id],
            goal,
            'B-1 exit forward motion',
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
        self._refresh_zone_occupancy()
        now = time.monotonic()
        markers = MarkerArray()
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
        color = (
            (0.90, 0.20, 0.16)
            if runtime.vehicle_id == 'agv1'
            else (0.12, 0.42, 0.92)
        )
        alpha = 1.0 if online else 0.35

        body = Marker()
        body.header.stamp = stamp
        body.header.frame_id = 'map'
        body.ns = 'fleet_vehicle'
        body.id = marker_id * 2
        body.type = Marker.CUBE
        body.action = Marker.ADD
        body.pose = copy.deepcopy(runtime.pose.pose)
        body.pose.position.z = 0.04
        body.scale.x = 0.34
        body.scale.y = 0.14
        body.scale.z = 0.08
        body.color.r, body.color.g, body.color.b = color
        body.color.a = alpha

        label = Marker()
        label.header = body.header
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
        return body, label


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
