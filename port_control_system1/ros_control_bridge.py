"""Threaded ROS 2 bridge for the desktop control dashboard.

The GUI remains usable without ROS. When ROS is available, this bridge keeps
all rclpy work on a background executor and exposes thread-safe command and
telemetry methods to the CustomTkinter views.
"""

from __future__ import annotations

import math
import os
import threading
import time
from dataclasses import dataclass, replace
from typing import Iterable, Optional, Sequence, Tuple


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
        self._emergency_active = False

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
            from std_msgs.msg import Float32, String
            from std_srvs.srv import Trigger
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
                    "/arm/container_pick/status",
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

    def _new_twist(self, linear_x: float = 0.0, angular_z: float = 0.0):
        twist_type = self._message_types.get("Twist")
        if twist_type is None:
            return None
        message = twist_type()
        message.linear.x = float(linear_x)
        message.angular.z = float(angular_z)
        return message

    def _publish_zero(self) -> None:
        publisher = self._publishers.get("cmd_vel")
        message = self._new_twist()
        if publisher is not None and message is not None:
            publisher.publish(message)

    def _safety_tick(self) -> None:
        if self._emergency_active:
            self._publish_zero()
            return
        if self._manual_stop_deadline and time.monotonic() >= self._manual_stop_deadline:
            self._manual_stop_deadline = 0.0
            self._publish_zero()

    def emergency_stop(self) -> bool:
        if not self.snapshot().ready:
            return False
        self._emergency_active = True
        self._update_snapshot(
            emergency_active=True,
            last_command="EMERGENCY_STOP",
        )
        self._publish_zero()
        return True

    def release_emergency_stop(self) -> bool:
        if not self.snapshot().ready:
            return False
        self._emergency_active = False
        self._manual_stop_deadline = 0.0
        self._publish_zero()
        self._update_snapshot(
            emergency_active=False,
            last_command="EMERGENCY_RELEASED",
        )
        return True

    def send_velocity(
        self,
        linear_x: float,
        angular_z: float,
        timeout_sec: float = 0.3,
    ) -> bool:
        if not self.snapshot().ready or self._emergency_active:
            return False
        publisher = self._publishers.get("cmd_vel")
        message = self._new_twist(linear_x, angular_z)
        if publisher is None or message is None:
            return False
        publisher.publish(message)
        self._manual_stop_deadline = time.monotonic() + max(timeout_sec, 0.05)
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
