"""Threaded ROS 2 bridge for the desktop control dashboard.

The GUI remains usable without ROS. When ROS is available, this bridge keeps
all rclpy work on a background executor and exposes thread-safe command and
telemetry methods to the CustomTkinter views.
"""

from __future__ import annotations

import math
import os
import re
import threading
import time
from dataclasses import dataclass, replace
from typing import Iterable, Optional, Sequence, Tuple


# Operator-facing vehicle names, keyed by ROS namespace. AMR 1 carries the
# blue cargo box and AMR 2 the yellow one, matching pinky.urdf.xacro. Only the
# displayed name is AMR: topics and services still use agv1/agv2.
AMR_DISPLAY_NAMES = {
    "agv1": "AMR 1 (파랑)",
    "agv2": "AMR 2 (노랑)",
}

AMR_SHORT_NAMES = {
    "agv1": "amr1",
    "agv2": "amr2",
}


def operator_vehicle_id(vehicle_id: str) -> str:
    """Return an AMR name for UI text without changing the ROS identifier."""
    value = str(vehicle_id or "")
    return AMR_SHORT_NAMES.get(value.lower(), value)


def operator_vehicle_text(value: str) -> str:
    """Replace embedded agv1/agv2 identifiers in operator-facing text."""
    return re.sub(
        r"\bagv([12])\b",
        lambda match: f"amr{match.group(1)}",
        str(value or ""),
        flags=re.IGNORECASE,
    )

ARM_DISPLAY_NAMES = {
    "arm1": "로봇팔 1 (ARM1)",
    "arm2": "로봇팔 2 (ARM2)",
}

# Latch-clearing surface, kept in step with scripts/clear_all_holds.sh.
COLLISION_SUPERVISOR_SERVICE = "/central/fleet/collision_supervisor/enabled"
FLEET_EMERGENCY_SERVICE = "/central/fleet/emergency_stop"
# Derived from fleet_dispatcher's exclusive_zone_ids ['B-1','A','PARK1','PARK2'].
ZONE_CLEAR_SERVICES = (
    "clear_b1_lock",
    "clear_a_lock",
    "clear_park1_lock",
    "clear_park2_lock",
)
CANCELLABLE_ACTIONS = (
    "navigate_to_pose",
    "navigate_through_poses",
    "park_in_spot",
)


@dataclass(frozen=True)
class FleetVehicleState:
    """One vehicle's telemetry as published on /central/fleet/<id>/state.

    emergency_stopped is only the operator/gateway latch. The collision
    supervisor stops a vehicle through a separate safety_hold service that
    never reaches VehicleState, so that reason arrives via RosSnapshot's
    collision_* fields instead.
    """

    vehicle_id: str = ""
    state_text: str = ""
    battery_percent: float = 0.0
    emergency_stopped: bool = False
    x: float = 0.0
    y: float = 0.0
    battery_voltage: float = 0.0
    nav2_ready: bool = False
    locked_zone: str = ""
    telemetry_age_sec: float = 0.0
    current_command_id: str = ""


@dataclass(frozen=True)
class RobotArmState:
    """One arm's structured state from the central ARM dispatcher."""

    arm_id: str = ""
    state: int = 1
    state_text: str = ""
    ready: bool = False
    current_command_id: str = ""
    current_mission_id: str = ""
    current_operation: str = ""
    operation_id: str = ""
    phase: str = ""
    progress: float = 0.0
    last_error: str = ""
    telemetry_age_sec: float = float("inf")


@dataclass(frozen=True)
class RosSnapshot:
    ready: bool = False
    error: Optional[str] = None
    battery_percent: Optional[float] = None
    battery_voltage: Optional[float] = None
    odom_xy: Optional[Tuple[float, float]] = None
    odom_yaw_deg: Optional[float] = None
    arm_status: Optional[str] = None
    arm2_status: Optional[str] = None
    emergency_active: bool = False
    last_command: Optional[str] = None
    fleet_states: tuple = ()
    arm_states: tuple = ()
    b1_zone: str = "B-1:UNKNOWN"
    # Mirrors /central/fleet/collision_status from fleet_collision_supervisor.
    collision_state: str = ""
    collision_held_vehicle: str = ""
    collision_distance_m: Optional[float] = None
    collision_transport: str = ""


class RosControlBridge:
    """Singleton ROS node owned by the dashboard process."""

    _instance: Optional["RosControlBridge"] = None
    _instance_lock = threading.Lock()

    @classmethod
    def get_instance(cls) -> "RosControlBridge":
        with cls._instance_lock:
            if cls._instance is None:
                cls._instance = cls()
            return cls._instance

    def __init__(self) -> None:
        self._state_lock = threading.Lock()
        self._snapshot = RosSnapshot()
        self._thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._node = None
        self._publishers = {}
        self._clients = {}
        self._message_types = {}
        self._manual_stop_deadline = 0.0
        self._manual_vehicle_id = ""
        self._emergency_active = False
        self._fleet_emergency = False
        self._emergency_vehicles = set()
        self._fleet_states = {}
        self._arm_states = {}

        self.cmd_vel_topic = os.environ.get(
            "PORT_CONTROL_CMD_VEL_TOPIC", "/cmd_vel"
        )
        self.target_pose_topic = os.environ.get(
            "PORT_CONTROL_TARGET_POSE_TOPIC",
            "/central/target_map_pose",
        )
        self.target_waypoints_topic = os.environ.get(
            "PORT_CONTROL_TARGET_WAYPOINTS_TOPIC",
            "/central/target_map_waypoints",
        )

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop_event.clear()
        self._thread = threading.Thread(
            target=self._run,
            name="port-control-ros",
            daemon=True,
        )
        self._thread.start()

    def stop(self) -> None:
        self._stop_event.set()
        if self._thread is not None and self._thread.is_alive():
            self._thread.join(timeout=3.0)
        self._thread = None

    def snapshot(self) -> RosSnapshot:
        with self._state_lock:
            return replace(self._snapshot)

    def _update_snapshot(self, **changes) -> None:
        with self._state_lock:
            self._snapshot = replace(self._snapshot, **changes)

    def _run(self) -> None:
        try:
            import rclpy
            from geometry_msgs.msg import PoseStamped, Twist
            from nav_msgs.msg import Odometry, Path
            from rclpy.context import Context
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.node import Node
            from rclpy.signals import SignalHandlerOptions
            from action_msgs.srv import CancelGoal
            from std_msgs.msg import Float32, String
            from std_srvs.srv import SetBool, Trigger
            from porter_interfaces.msg import ArmState, VehicleState
        except Exception as exc:
            self._update_snapshot(
                ready=False,
                error=f"ROS import failed: {exc}",
            )
            return

        owner = self
        context = Context()
        executor = None
        node = None

        class DashboardBridgeNode(Node):
            def __init__(self):
                super().__init__("port_control_dashboard_bridge", context=context)
                owner._message_types = {
                    "PoseStamped": PoseStamped,
                    "Twist": Twist,
                    "Path": Path,
                    "Trigger": Trigger,
                    "SetBool": SetBool,
                    "CancelGoal": CancelGoal,
                }
                owner._publishers = {
                    "cmd_vel": self.create_publisher(
                        Twist, owner.cmd_vel_topic, 10
                    ),
                    "target_pose": self.create_publisher(
                        PoseStamped, owner.target_pose_topic, 10
                    ),
                    "target_waypoints": self.create_publisher(
                        Path, owner.target_waypoints_topic, 10
                    ),
                    "agv1_cmd_vel": self.create_publisher(
                        Twist, "/agv1/cmd_vel_manual", 10
                    ),
                    "agv2_cmd_vel": self.create_publisher(
                        Twist, "/agv2/cmd_vel_manual", 10
                    ),
                }
                owner._clients = {
                    "/arm/pick_container": self.create_client(
                        Trigger, "/arm/pick_container"
                    ),
                    "/arm/stack_container": self.create_client(
                        Trigger, "/arm/stack_container"
                    ),
                    "/arm2/pick_container": self.create_client(
                        Trigger, "/arm2/pick_container"
                    ),
                    "/arm2/stack_container": self.create_client(
                        Trigger, "/arm2/stack_container"
                    ),
                }
                # Every latch that can keep a vehicle from moving lives in a
                # different node, so releasing one is not enough. Mirrors
                # scripts/clear_all_holds.sh.
                owner._clients[COLLISION_SUPERVISOR_SERVICE] = (
                    self.create_client(SetBool, COLLISION_SUPERVISOR_SERVICE)
                )
                owner._clients[FLEET_EMERGENCY_SERVICE] = self.create_client(
                    SetBool, FLEET_EMERGENCY_SERVICE
                )
                for vehicle_id in AMR_DISPLAY_NAMES:
                    for service in ("safety_hold", "emergency_stop"):
                        name = f"/{vehicle_id}/{service}"
                        owner._clients[name] = self.create_client(
                            SetBool, name
                        )
                    for action in CANCELLABLE_ACTIONS:
                        name = f"/{vehicle_id}/{action}/_action/cancel_goal"
                        owner._clients[name] = self.create_client(
                            CancelGoal, name
                        )
                for service in ZONE_CLEAR_SERVICES:
                    name = f"/central/fleet/{service}"
                    owner._clients[name] = self.create_client(Trigger, name)
                for arm_id in ARM_DISPLAY_NAMES:
                    name = f"/central/arms/{arm_id}/resume"
                    owner._clients[name] = self.create_client(Trigger, name)

                for vehicle_id in ("agv1", "agv2"):
                    self.create_subscription(
                        VehicleState,
                        f"/central/fleet/{vehicle_id}/state",
                        owner._on_vehicle_state,
                        10,
                    )
                for arm_id in ARM_DISPLAY_NAMES:
                    self.create_subscription(
                        ArmState,
                        f"/central/arms/{arm_id}/state",
                        owner._on_arm_state,
                        10,
                    )
                self.create_subscription(
                    String,
                    "/central/fleet/zones",
                    lambda msg: owner._update_snapshot(b1_zone=msg.data),
                    10,
                )
                self.create_subscription(
                    String,
                    "/central/fleet/collision_status",
                    owner._on_collision_status,
                    10,
                )
                self.create_subscription(
                    Float32,
                    "/battery/percent",
                    lambda msg: owner._update_snapshot(
                        battery_percent=float(msg.data)
                    ),
                    10,
                )
                self.create_subscription(
                    Float32,
                    "/battery/voltage",
                    lambda msg: owner._update_snapshot(
                        battery_voltage=float(msg.data)
                    ),
                    10,
                )
                self.create_subscription(
                    Odometry, "/odom", owner._on_odom, 10
                )
                self.create_subscription(
                    String,
                    "/arm/pick_place/status",
                    lambda msg: owner._update_snapshot(arm_status=msg.data),
                    10,
                )
                self.create_subscription(
                    String,
                    "/arm2/container_pick/status",
                    lambda msg: owner._update_snapshot(arm2_status=msg.data),
                    10,
                )
                self.create_timer(0.01, owner._safety_tick)

        try:
            rclpy.init(
                context=context,
                signal_handler_options=SignalHandlerOptions.NO,
            )
            node = DashboardBridgeNode()
            self._node = node
            executor = SingleThreadedExecutor(context=context)
            executor.add_node(node)
            self._update_snapshot(ready=True, error=None)

            while not self._stop_event.is_set() and context.ok():
                executor.spin_once(timeout_sec=0.1)
        except Exception as exc:
            self._update_snapshot(ready=False, error=f"ROS bridge failed: {exc}")
        finally:
            self._publish_zero()
            self._publishers = {}
            self._clients = {}
            self._message_types = {}
            self._node = None
            if executor is not None:
                executor.shutdown(timeout_sec=1.0)
            if node is not None:
                node.destroy_node()
            if context.ok():
                rclpy.shutdown(context=context)
            self._update_snapshot(ready=False)

    def _on_odom(self, message) -> None:
        position = message.pose.pose.position
        orientation = message.pose.pose.orientation
        siny = 2.0 * (
            orientation.w * orientation.z
            + orientation.x * orientation.y
        )
        cosy = 1.0 - 2.0 * (
            orientation.y * orientation.y
            + orientation.z * orientation.z
        )
        self._update_snapshot(
            odom_xy=(float(position.x), float(position.y)),
            odom_yaw_deg=math.degrees(math.atan2(siny, cosy)),
        )

    def _on_vehicle_state(self, message) -> None:
        if message.emergency_stopped:
            self._emergency_vehicles.add(message.vehicle_id)
        else:
            self._emergency_vehicles.discard(message.vehicle_id)
        self._emergency_active = (
            self._fleet_emergency or bool(self._emergency_vehicles)
        )
        self._fleet_states[message.vehicle_id] = FleetVehicleState(
            vehicle_id=message.vehicle_id,
            state_text=message.state_text,
            battery_percent=float(message.battery_percent),
            emergency_stopped=bool(message.emergency_stopped),
            x=float(message.pose.pose.position.x),
            y=float(message.pose.pose.position.y),
            battery_voltage=float(message.battery_voltage),
            nav2_ready=bool(message.nav2_ready),
            locked_zone=str(message.locked_zone),
            telemetry_age_sec=float(message.telemetry_age_sec),
            current_command_id=str(message.current_command_id),
        )
        self._update_snapshot(
            fleet_states=tuple(
                self._fleet_states[key]
                for key in sorted(self._fleet_states)
            ),
            emergency_active=self._emergency_active,
        )

    def _on_arm_state(self, message) -> None:
        """Store the latest structured connection and work state per arm."""
        arm_id = str(message.arm_id or "").strip().lower()
        if arm_id not in ARM_DISPLAY_NAMES:
            return
        self._arm_states[arm_id] = RobotArmState(
            arm_id=arm_id,
            state=int(message.state),
            state_text=str(message.state_text),
            ready=bool(message.ready),
            current_command_id=str(message.current_command_id),
            current_mission_id=str(message.current_mission_id),
            current_operation=str(message.current_operation),
            operation_id=str(message.operation_id),
            phase=str(message.phase),
            progress=float(message.progress),
            last_error=str(message.last_error),
            telemetry_age_sec=float(message.telemetry_age_sec),
        )
        self._update_snapshot(
            arm_states=tuple(
                self._arm_states[key]
                for key in sorted(self._arm_states)
            )
        )

    def _on_collision_status(self, message) -> None:
        """Track which vehicle the collision supervisor is currently holding."""
        import json

        try:
            status = json.loads(message.data)
        except (ValueError, AttributeError):
            return
        if not isinstance(status, dict):
            return
        distance = status.get("minimum_distance_m")
        self._update_snapshot(
            collision_state=str(status.get("state") or ""),
            collision_held_vehicle=str(status.get("held_vehicle") or ""),
            collision_distance_m=(
                float(distance) if isinstance(distance, (int, float)) else None
            ),
            collision_transport=str(status.get("hold_transport") or ""),
        )

    def _new_twist(self, linear_x: float = 0.0, angular_z: float = 0.0):
        twist_type = self._message_types.get("Twist")
        if twist_type is None:
            return None
        message = twist_type()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        return message

    def _publish_zero(self, vehicle_id: str = "") -> None:
        publisher = self._publishers.get(
            f"{vehicle_id}_cmd_vel" if vehicle_id else "cmd_vel"
        )
        message = self._new_twist()
        if publisher is not None and message is not None:
            publisher.publish(message)

    def _safety_tick(self) -> None:
        if self._fleet_emergency:
            self._publish_zero()
            self._publish_zero("agv1")
            self._publish_zero("agv2")
            return
        for vehicle_id in self._emergency_vehicles:
            self._publish_zero(vehicle_id)
        if self._manual_stop_deadline and time.monotonic() >= self._manual_stop_deadline:
            self._manual_stop_deadline = 0.0
            self._publish_zero(self._manual_vehicle_id)
            self._manual_vehicle_id = ""

    def emergency_stop(self, vehicle_id: str = "fleet") -> bool:
        if not self.snapshot().ready:
            return False
        try:
            from central_control_client import CentralControlClient
            CentralControlClient().set_emergency(True, vehicle_id)
        except Exception:
            return False
        if vehicle_id == "fleet":
            try:
                from realtime_llm_agent import RealtimeLLMAgent
                RealtimeLLMAgent.get_instance().emergency_stop_reset()
            except Exception as exc:
                # The central emergency latch and ARM stops already succeeded;
                # a dashboard bookkeeping error must never report the physical
                # stop itself as failed.
                print(f"[비상정지 자율 상태 초기화 경고] {exc}", flush=True)
        if vehicle_id == "fleet":
            self._fleet_emergency = True
            self._publish_zero()
            self._publish_zero("agv1")
            self._publish_zero("agv2")
        else:
            self._emergency_vehicles.add(vehicle_id)
            self._publish_zero(vehicle_id)
        self._emergency_active = (
            self._fleet_emergency or bool(self._emergency_vehicles)
        )
        self._update_snapshot(
            emergency_active=self._emergency_active,
            last_command="EMERGENCY_STOP",
        )
        return True

    def release_emergency_stop(self, vehicle_id: str = "fleet") -> bool:
        if not self.snapshot().ready:
            return False
        try:
            from central_control_client import CentralControlClient
            CentralControlClient().set_emergency(False, vehicle_id)
        except Exception:
            return False
        if vehicle_id == "fleet":
            self._fleet_emergency = False
            self._emergency_vehicles.clear()
        else:
            self._emergency_vehicles.discard(vehicle_id)
        self._emergency_active = (
            self._fleet_emergency or bool(self._emergency_vehicles)
        )
        self._manual_stop_deadline = 0.0
        self._publish_zero(vehicle_id if vehicle_id != "fleet" else "")
        self._update_snapshot(
            emergency_active=self._emergency_active,
            last_command="EMERGENCY_RELEASED",
        )
        return True

    def send_velocity(
        self,
        linear_x: float,
        angular_z: float,
        timeout_sec: float = 0.3,
        vehicle_id: str = "",
    ) -> bool:
        stopped = (
            self._fleet_emergency
            or vehicle_id in self._emergency_vehicles
            or (not vehicle_id and self._emergency_active)
        )
        if not self.snapshot().ready or stopped:
            return False
        publisher = self._publishers.get(
            f"{vehicle_id}_cmd_vel" if vehicle_id else "cmd_vel"
        )
        message = self._new_twist(linear_x, angular_z)
        if publisher is None or message is None:
            return False
        publisher.publish(message)
        self._manual_stop_deadline = time.monotonic() + max(timeout_sec, 0.05)
        self._manual_vehicle_id = vehicle_id
        self._update_snapshot(
            last_command=f"cmd_vel({linear_x:.3f}, {angular_z:.3f})"
        )
        return True

    def send_target(self, x: float, y: float, yaw_deg: float = 0.0) -> bool:
        return self.send_waypoints(
            [(x, y, yaw_deg)],
            publish_path=False,
        )

    def send_waypoints(
        self,
        waypoints: Iterable[Sequence[float]],
        publish_path: bool = True,
    ) -> bool:
        if not self.snapshot().ready or self._node is None:
            return False
        points = [tuple(float(value) for value in point) for point in waypoints]
        if not points:
            return False

        pose_type = self._message_types.get("PoseStamped")
        path_type = self._message_types.get("Path")
        if pose_type is None or path_type is None:
            return False

        stamp = self._node.get_clock().now().to_msg()
        poses = []
        for point in points:
            if len(point) not in (2, 3):
                raise ValueError("waypoint must contain x, y, and optional yaw")
            yaw = math.radians(point[2] if len(point) == 3 else 0.0)
            pose = pose_type()
            pose.header.stamp = stamp
            pose.header.frame_id = "map"
            pose.pose.position.x = point[0]
            pose.pose.position.y = point[1]
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)
            poses.append(pose)

        if not publish_path:
            if len(poses) != 1:
                raise ValueError("a single target requires exactly one pose")
            publisher = self._publishers["target_pose"]
            if publisher.get_subscription_count() == 0:
                self._update_snapshot(
                    last_command="target_pose: no subscriber"
                )
                return False
            publisher.publish(poses[0])
            command = f"target_pose({points[0][0]:.3f}, {points[0][1]:.3f})"
        else:
            publisher = self._publishers["target_waypoints"]
            if publisher.get_subscription_count() == 0:
                self._update_snapshot(
                    last_command="target_waypoints: no subscriber"
                )
                return False
            path = path_type()
            path.header.stamp = stamp
            path.header.frame_id = "map"
            path.poses = poses
            publisher.publish(path)
            command = f"target_waypoints(count={len(poses)})"
        self._update_snapshot(last_command=command)
        return True

    def _call_service_sync(self, service_name, request, timeout_sec=5.0):
        """Call one service and wait for it, returning (ok, detail).

        The node already spins on its own executor thread, so this polls the
        future instead of spinning it again from here.
        """
        client = self._clients.get(service_name)
        if client is None:
            return False, "클라이언트 없음"
        if not client.service_is_ready():
            return False, "서비스 없음"
        future = client.call_async(request)
        deadline = time.monotonic() + timeout_sec
        while not future.done() and time.monotonic() < deadline:
            time.sleep(0.02)
        if not future.done():
            return False, "응답 시간 초과"
        try:
            response = future.result()
        except Exception as exc:  # noqa: BLE001 - ROS future exception
            return False, str(exc)
        # CancelGoal has no success field; an accepted cancel returns code 0.
        success = getattr(response, "success", None)
        if success is None:
            code = getattr(response, "return_code", 0)
            return code == 0, f"return_code={code}"
        return bool(success), str(getattr(response, "message", ""))

    def _await_services(self, service_names, timeout_sec=4.0):
        """Wait once for every client to discover its service.

        Services reached through the zenoh bridge can take a second or two to
        show up. Waiting per call would multiply that by the number of calls,
        so poll them all together and bound the total wait instead.
        """
        deadline = time.monotonic() + timeout_sec
        pending = list(service_names)
        while pending and time.monotonic() < deadline:
            pending = [
                name for name in pending
                if not (
                    self._clients.get(name) is not None
                    and self._clients[name].service_is_ready()
                )
            ]
            if pending:
                time.sleep(0.05)
        return pending

    def clear_all_holds(self, cancel_goals: bool = True):
        """Release every latch that can keep an AMR stopped.

        Same sequence as scripts/clear_all_holds.sh: the collision supervisor
        is toggled off and back on so it drops whatever it holds without
        leaving collisions unguarded. Returns a list of (label, ok, detail).
        """
        if not self.snapshot().ready:
            return None

        set_bool = self._message_types.get("SetBool")
        trigger = self._message_types.get("Trigger")
        cancel = self._message_types.get("CancelGoal")
        if set_bool is None or trigger is None or cancel is None:
            return None

        def set_bool_request(value):
            request = set_bool.Request()
            request.data = value
            return request

        # Give every client one shared chance to finish discovery first.
        all_services = [COLLISION_SUPERVISOR_SERVICE, FLEET_EMERGENCY_SERVICE]
        for vehicle_id in AMR_DISPLAY_NAMES:
            all_services += [
                f"/{vehicle_id}/safety_hold",
                f"/{vehicle_id}/emergency_stop",
            ]
            if cancel_goals:
                all_services += [
                    f"/{vehicle_id}/{action}/_action/cancel_goal"
                    for action in CANCELLABLE_ACTIONS
                ]
        all_services += [
            f"/central/fleet/{service}" for service in ZONE_CLEAR_SERVICES
        ]
        all_services += [
            f"/central/arms/{arm_id}/resume"
            for arm_id in ARM_DISPLAY_NAMES
        ]
        self._await_services(all_services)

        results = []

        # 1) Collision supervisor: disabling releases its automatic hold.
        results.append((
            "충돌 감시기 해제",
            *self._call_service_sync(
                COLLISION_SUPERVISOR_SERVICE, set_bool_request(False)
            ),
        ))
        time.sleep(1.0)
        results.append((
            "충돌 감시기 재가동",
            *self._call_service_sync(
                COLLISION_SUPERVISOR_SERVICE, set_bool_request(True)
            ),
        ))

        # 2) Per-vehicle latches inside cmd_vel_safety_gate.
        for vehicle_id, display_name in AMR_DISPLAY_NAMES.items():
            for service, label in (
                ("safety_hold", "충돌 정지"),
                ("emergency_stop", "비상정지"),
            ):
                results.append((
                    f"{display_name} {label}",
                    *self._call_service_sync(
                        f"/{vehicle_id}/{service}", set_bool_request(False)
                    ),
                ))

        # 3) Fleet-wide emergency latch in the dispatcher.
        results.append((
            "전체 비상정지",
            *self._call_service_sync(
                FLEET_EMERGENCY_SERVICE, set_bool_request(False)
            ),
        ))

        # Emergency stop also latches both central ARM dispatcher states.
        # Resume only those software gates; no robot motion is commanded.
        for arm_id, display_name in ARM_DISPLAY_NAMES.items():
            results.append((
                f"{display_name} 재활성화",
                *self._call_service_sync(
                    f"/central/arms/{arm_id}/resume", trigger.Request()
                ),
            ))

        # 4) In-flight goals: bt_navigator rejects new ones while one runs.
        #    This must precede the zone locks - _clear_zone_lock refuses while
        #    the owning vehicle is still busy, so clearing first always failed
        #    with "owner is still executing a command".
        if cancel_goals:
            for vehicle_id, display_name in AMR_DISPLAY_NAMES.items():
                for action in CANCELLABLE_ACTIONS:
                    # A default request is all-zero, which cancels every goal.
                    results.append((
                        f"{display_name} {action} 취소",
                        *self._call_service_sync(
                            f"/{vehicle_id}/{action}/_action/cancel_goal",
                            cancel.Request(),
                        ),
                    ))
            # Let the dispatcher observe the cancelled goals and drop `busy`.
            time.sleep(1.5)

        # 5) Zone locks, or the dispatcher refuses to dispatch into them.
        for service in ZONE_CLEAR_SERVICES:
            results.append((
                f"구역 잠금 {service}",
                *self._call_service_sync(
                    f"/central/fleet/{service}", trigger.Request()
                ),
            ))

        self._fleet_emergency = False
        self._emergency_vehicles.clear()
        self._emergency_active = False
        self._manual_stop_deadline = 0.0
        self._update_snapshot(
            emergency_active=False,
            last_command="CLEAR_ALL_HOLDS",
        )
        return results

    def call_trigger(self, service_name: str) -> bool:
        client = self._clients.get(service_name)
        if client is None or not client.service_is_ready():
            return False

        trigger_type = self._message_types.get("Trigger")
        if trigger_type is None:
            return False
        request = trigger_type.Request()
        future = client.call_async(request)

        def complete(done_future) -> None:
            try:
                response = done_future.result()
                result = f"{service_name}: {response.success} {response.message}"
            except Exception as exc:
                result = f"{service_name}: failed: {exc}"
            self._update_snapshot(last_command=result)

        future.add_done_callback(complete)
        self._update_snapshot(last_command=f"{service_name}: requested")
        return True
