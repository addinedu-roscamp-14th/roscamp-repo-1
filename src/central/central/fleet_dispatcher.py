#!/usr/bin/env python3

"""Two-vehicle Nav2 dispatcher with B-1 locking and emergency control."""

from __future__ import annotations

from dataclasses import dataclass, field
import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav2_msgs.action import NavigateThroughPoses, NavigateToPose
from nav_msgs.msg import Odometry
from porter_interfaces.action import DispatchNavigation
from porter_interfaces.msg import VehicleState
import rclpy
from rclpy.action import ActionClient, ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import Float32, String
from std_srvs.srv import SetBool, Trigger


@dataclass
class VehicleRuntime:
    vehicle_id: str
    pose: PoseStamped = field(default_factory=PoseStamped)
    pose_received_at: float | None = None
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
        self.declare_parameter('b1_zone_id', 'B-1')

        vehicle_ids = [
            str(value).strip('/')
            for value in self.get_parameter('vehicle_ids').value
        ]
        if vehicle_ids != ['agv1', 'agv2']:
            raise ValueError('vehicle_ids must be [agv1, agv2]')
        self.telemetry_timeout = float(
            self.get_parameter('telemetry_timeout_sec').value
        )
        self.b1_zone_id = str(self.get_parameter('b1_zone_id').value)
        self.callback_group = ReentrantCallbackGroup()
        self._lock = threading.RLock()
        self._zone_condition = threading.Condition(self._lock)
        self._b1_owner = ''
        self._b1_unknown = False
        self._b1_queue = []

        self.vehicles = {
            vehicle_id: VehicleRuntime(vehicle_id)
            for vehicle_id in vehicle_ids
        }
        self.nav_pose_clients = {}
        self.nav_waypoint_clients = {}
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
        self.create_service(
            SetBool,
            '/central/fleet/emergency_stop',
            self._set_all_emergency_service,
            callback_group=self.callback_group,
        )
        self.create_service(
            Trigger,
            '/central/fleet/clear_b1_lock',
            self._clear_b1_lock,
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
        self.create_subscription(
            PoseWithCovarianceStamped,
            f'/{vehicle_id}/amcl_pose',
            lambda message, vid=vehicle_id: self._on_amcl_pose(vid, message),
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Odometry,
            f'/{vehicle_id}/odom',
            lambda message, vid=vehicle_id: self._on_odom(vid, message),
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            f'/{vehicle_id}/battery/percent',
            lambda message, vid=vehicle_id: self._on_battery(
                vid, 'battery_percent', message
            ),
            10,
            callback_group=self.callback_group,
        )
        self.create_subscription(
            Float32,
            f'/{vehicle_id}/battery/voltage',
            lambda message, vid=vehicle_id: self._on_battery(
                vid, 'battery_voltage', message
            ),
            10,
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
            runtime.has_amcl_pose = True

    def _on_odom(self, vehicle_id, message):
        with self._lock:
            runtime = self.vehicles[vehicle_id]
            if runtime.has_amcl_pose:
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
        runtime = self.vehicles[vehicle_id]
        pose_fresh = (
            runtime.pose_received_at is not None
            and time.monotonic() - runtime.pose_received_at
            <= self.telemetry_timeout
        )
        action_ready = self.nav_pose_clients[vehicle_id].server_is_ready()
        if require_waypoints:
            action_ready = (
                action_ready
                and self.nav_waypoint_clients[vehicle_id].server_is_ready()
            )
        return (
            pose_fresh
            and runtime.has_amcl_pose
            and not runtime.busy
            and not runtime.emergency
            and action_ready
        )

    def _select_and_reserve_vehicle(
        self,
        requested_vehicle_id,
        target_pose,
        command_id,
        require_waypoints=False,
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
                candidates = [
                    vehicle_id
                    for vehicle_id in sorted(self.vehicles)
                    if self._vehicle_ready(vehicle_id, require_waypoints)
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

    def _acquire_b1(self, goal_handle, vehicle_id, command_id):
        with self._zone_condition:
            if self._b1_owner == vehicle_id and not self._b1_unknown:
                return True
            self._b1_queue.append(command_id)
            while True:
                first = self._b1_queue and self._b1_queue[0] == command_id
                if first and not self._b1_owner and not self._b1_unknown:
                    self._b1_queue.pop(0)
                    self._b1_owner = vehicle_id
                    self.vehicles[vehicle_id].locked_zone = self.b1_zone_id
                    return True
                if goal_handle.is_cancel_requested:
                    if command_id in self._b1_queue:
                        self._b1_queue.remove(command_id)
                    return False
                self._zone_condition.wait(timeout=0.1)

    def _release_b1(self, vehicle_id):
        with self._zone_condition:
            if self._b1_owner == vehicle_id:
                runtime = self.vehicles[vehicle_id]
                telemetry_lost = (
                    runtime.pose_received_at is None
                    or time.monotonic() - runtime.pose_received_at
                    > self.telemetry_timeout
                )
                if telemetry_lost:
                    self._b1_unknown = True
                else:
                    self._b1_owner = ''
                    runtime.locked_zone = ''
                self._zone_condition.notify_all()

    async def _execute(self, goal_handle):
        request = goal_handle.request
        result = DispatchNavigation.Result()
        command_id = request.command_id or f'dispatch-{time.time_ns()}'
        vehicle_id = self._select_and_reserve_vehicle(
            request.requested_vehicle_id,
            request.poses[-1],
            command_id,
            require_waypoints=len(request.poses) > 1,
        )
        if not vehicle_id:
            goal_handle.abort()
            result.error_code = self.ERROR_NO_VEHICLE
            result.message = 'no ready vehicle is available'
            return result
        result.assigned_vehicle_id = vehicle_id
        runtime = self.vehicles[vehicle_id]
        leaving_b1 = (
            runtime.locked_zone == self.b1_zone_id
            and request.zone_id != self.b1_zone_id
        )
        navigation_succeeded = False
        try:
            if request.zone_id == self.b1_zone_id:
                self._publish_dispatch_feedback(
                    goal_handle,
                    vehicle_id,
                    'WAITING_FOR_B1',
                    0,
                )
                if not self._acquire_b1(
                    goal_handle, vehicle_id, command_id
                ):
                    goal_handle.canceled()
                    result.error_code = self.ERROR_CANCELED
                    result.message = 'canceled while waiting for B-1'
                    return result

            self._publish_dispatch_feedback(
                goal_handle,
                vehicle_id,
                'DISPATCHED',
                0,
            )
            if len(request.poses) == 1:
                nav_goal = NavigateToPose.Goal()
                nav_goal.pose = request.poses[0]
                client = self.nav_pose_clients[vehicle_id]
            else:
                nav_goal = NavigateThroughPoses.Goal()
                nav_goal.poses = list(request.poses)
                client = self.nav_waypoint_clients[vehicle_id]

            send_future = client.send_goal_async(
                nav_goal,
                feedback_callback=lambda message, count=len(request.poses):
                    self._relay_feedback(
                        goal_handle,
                        vehicle_id,
                        count,
                        message,
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
            if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
                goal_handle.abort()
                result.error_code = self.ERROR_NAV_FAILED
                result.message = f'Nav2 finished with status {nav_result.status}'
                return result
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
            with self._lock:
                runtime.busy = False
                runtime.current_command_id = ''
                runtime.active_nav_goal = None
            if leaving_b1 and navigation_succeeded:
                self._release_b1(vehicle_id)

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

    def _clear_b1_lock(self, _request, response):
        with self._zone_condition:
            if self._b1_owner and self.vehicles[self._b1_owner].busy:
                response.success = False
                response.message = 'B-1 owner is still executing a command'
                return response
            if self._b1_owner:
                self.vehicles[self._b1_owner].locked_zone = ''
            self._b1_owner = ''
            self._b1_unknown = False
            self._zone_condition.notify_all()
        response.success = True
        response.message = 'B-1 lock cleared by operator'
        return response

    def _publish_states(self):
        now = time.monotonic()
        with self._lock:
            for vehicle_id, runtime in self.vehicles.items():
                message = VehicleState()
                message.header.stamp = self.get_clock().now().to_msg()
                message.header.frame_id = 'map'
                message.vehicle_id = vehicle_id
                age = (
                    math.inf
                    if runtime.pose_received_at is None
                    else now - runtime.pose_received_at
                )
                if (
                    self._b1_owner == vehicle_id
                    and age > self.telemetry_timeout
                ):
                    self._b1_unknown = True
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
            zone = String()
            zone.data = (
                f'B-1:UNKNOWN:{self._b1_owner}'
                if self._b1_unknown
                else f'B-1:{self._b1_owner or "FREE"}'
            )
            self.zone_publisher.publish(zone)


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
