#!/usr/bin/env python3

"""Latched per-vehicle velocity gate for emergency and automatic holds."""

import threading

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool


class CmdVelSafetyGate(Node):
    def __init__(self):
        super().__init__('cmd_vel_safety_gate')
        self.declare_parameter('input_topic', 'cmd_vel_safe_input')
        self.declare_parameter('manual_input_topic', 'cmd_vel_manual')
        self.declare_parameter('parking_input_topic', 'cmd_vel_parking')
        self.declare_parameter('output_topic', 'cmd_vel')
        self.declare_parameter('emergency_service', 'emergency_stop')
        self.declare_parameter('safety_hold_service', 'safety_hold')
        self.declare_parameter('command_timeout_sec', 0.5)
        self.declare_parameter('parking_priority_timeout_sec', 0.5)
        self.declare_parameter('publish_rate_hz', 100.0)

        self._lock = threading.Lock()
        self._emergency = False
        self._safety_hold = False
        self._last_command = Twist()
        self._last_command_time = self.get_clock().now()
        self._last_parking_command = Twist()
        self._last_parking_command_time = None
        self._timeout = float(self.get_parameter('command_timeout_sec').value)
        self._parking_timeout = float(
            self.get_parameter('parking_priority_timeout_sec').value
        )
        rate = float(self.get_parameter('publish_rate_hz').value)
        if self._timeout <= 0.0 or self._parking_timeout <= 0.0 or rate <= 0.0:
            raise ValueError('timeout and publish rate must be positive')

        output_topic = str(self.get_parameter('output_topic').value)
        self.publisher = self.create_publisher(Twist, output_topic, 10)
        for topic_parameter in ('input_topic', 'manual_input_topic'):
            self.create_subscription(
                Twist,
                str(self.get_parameter(topic_parameter).value),
                self._on_command,
                10,
            )
        parking_input_topic = str(
            self.get_parameter('parking_input_topic').value
        )
        self.create_subscription(
            Twist,
            parking_input_topic,
            self._on_parking_command,
            10,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter('emergency_service').value),
            self._on_emergency,
        )
        self.create_service(
            SetBool,
            str(self.get_parameter('safety_hold_service').value),
            self._on_safety_hold,
        )
        self.create_timer(1.0 / rate, self._publish)
        self.get_logger().info(
            'Velocity safety gate ready: '
            f'parking_priority_input={parking_input_topic}, '
            f'output={output_topic}'
        )

    def _on_command(self, message):
        with self._lock:
            if self._emergency:
                return
            self._last_command = message
            self._last_command_time = self.get_clock().now()

    def _on_parking_command(self, message):
        """Keep parking motion from racing Nav2 velocity-smoother output."""
        with self._lock:
            if self._emergency:
                return
            self._last_parking_command = message
            self._last_parking_command_time = self.get_clock().now()

    def _on_emergency(self, request, response):
        with self._lock:
            self._emergency = bool(request.data)
            self._last_command = Twist()
            self._last_command_time = self.get_clock().now()
            self._last_parking_command = Twist()
            self._last_parking_command_time = None
        self.publisher.publish(Twist())
        response.success = True
        response.message = (
            'emergency stop latched'
            if request.data
            else 'emergency stop released'
        )
        self.get_logger().warning(response.message)
        return response

    def _on_safety_hold(self, request, response):
        with self._lock:
            self._safety_hold = bool(request.data)
            if self._safety_hold:
                self._last_command_time = self.get_clock().now()
                self._last_parking_command = Twist()
                self._last_parking_command_time = None
        self.publisher.publish(Twist())
        response.success = True
        response.message = (
            'automatic safety hold latched'
            if request.data
            else 'automatic safety hold released'
        )
        self.get_logger().warning(response.message)
        return response

    def _publish(self):
        with self._lock:
            now = self.get_clock().now()
            age = (now - self._last_command_time).nanoseconds / 1e9
            parking_age = (
                None
                if self._last_parking_command_time is None
                else (
                    now - self._last_parking_command_time
                ).nanoseconds / 1e9
            )
            if self._emergency or self._safety_hold:
                message = Twist()
            elif parking_age is not None and parking_age <= self._parking_timeout:
                message = self._last_parking_command
            elif age <= self._timeout:
                message = self._last_command
            else:
                message = Twist()
        self.publisher.publish(message)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelSafetyGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
