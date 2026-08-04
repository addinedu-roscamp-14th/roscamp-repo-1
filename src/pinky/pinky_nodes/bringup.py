#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
import time

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_ros import TransformBroadcaster
from tf_transformations import quaternion_from_euler
from std_msgs.msg import Float32

from .dynamixel_driver import DynamixelDriver

TWIST_SUB_TOPIC_NAME = "cmd_vel"
ODOM_PUB_TOPIC_NAME = "odom"
JOINT_PUB_TOPIC_NAME = "joint_states"

SERIAL_PORT_NAME = "/dev/ttyAMA5"
BAUDRATE = 1000000
DYNAMIXEL_IDS = [1, 2] # [왼쪽 바퀴 ID, 오른쪽 바퀴 ID]

PULSE_PER_ROT = 4096 
RPM2RAD = 2 * math.pi / 60

BATTERY_VOLTAGE_TOPIC = "battery/voltage"
LOW_BATTERY_THRESHOLD = 6.8

class Pinky(Node):
    def __init__(self):
        super().__init__('pinky')
        self.is_initialized = False
        
        self.get_logger().info('Initializing Pinky Bringup Node with Dynamixel...')
        
        self.declare_parameter('wheel_radius', 0.027)
        self.declare_parameter('wheel_separation', 0.0961)
        self.declare_parameter('frame_prefix', '')
        self.declare_parameter('left_wheel_joint', 'left_wheel_joint')
        self.declare_parameter('right_wheel_joint', 'right_wheel_joint')
        self.declare_parameter('serial_port', SERIAL_PORT_NAME)
        self.declare_parameter('baud_rate', BAUDRATE)
        
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_separation = self.get_parameter('wheel_separation').get_parameter_value().double_value
        prefix = str(self.get_parameter('frame_prefix').value).strip('/')
        self.frame_prefix = f'{prefix}/' if prefix else ''
        self.odom_frame_id = f'{self.frame_prefix}odom'
        self.odom_child_frame_id = f'{self.frame_prefix}base_footprint'
        self.left_wheel_joint = str(
            self.get_parameter('left_wheel_joint').value
        )
        self.right_wheel_joint = str(
            self.get_parameter('right_wheel_joint').value
        )
        
        self.get_logger().info(f'Wheel radius: {self.wheel_radius}')
        self.get_logger().info(f'Wheel separation: {self.wheel_separation}')
        
        self.circumference = 2 * math.pi * self.wheel_radius
        serial_port = str(self.get_parameter('serial_port').value)
        baud_rate = int(self.get_parameter('baud_rate').value)
        self.driver = DynamixelDriver(serial_port, baud_rate, DYNAMIXEL_IDS)

        self.get_logger().info("1. Opening serial port...")
        if not self.driver.begin():
            self.get_logger().error("Failed to open serial port! Shutting down.")
            return

        self.get_logger().info("2. Initializing motors...")
        if not self.driver.initialize_motors():
            self.get_logger().error("Failed to initialize motors! Shutting down.")
            self.driver.terminate()
            return
        
        self.get_logger().info("Waiting for motors to be ready...")
        time.sleep(1.0)

        self.get_logger().info("3. Setting initial RPM to zero...")
        if not self.driver.set_double_rpm(0, 0):
            self.get_logger().error("Failed to set initial RPM! Shutting down.")
            self.driver.terminate()
            return

        self.get_logger().info("4. Reading initial encoder values...")
        _, _, self.last_encoder_l, self.last_encoder_r = self.driver.get_feedback()
        if self.last_encoder_l is None:
            self.get_logger().error("Failed to read initial encoder position! Shutting down.")
            self.driver.terminate()
            return

        self.get_logger().info(f"Initial Encoder read: L={self.last_encoder_l}, R={self.last_encoder_r}. Controller is responsive.")
            
        self.odom_pub = self.create_publisher(Odometry, ODOM_PUB_TOPIC_NAME, 10)
        self.joint_pub = self.create_publisher(JointState, JOINT_PUB_TOPIC_NAME, 10)
        self.twist_sub = self.create_subscription(Twist, TWIST_SUB_TOPIC_NAME, self.twist_callback, 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.timer = self.create_timer(1.0 / 30.0, self.update_and_publish)

        self.battery_sub = self.create_subscription(
            Float32,
            BATTERY_VOLTAGE_TOPIC,
            self.battery_voltage_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = self.get_clock().now()
        self.is_initialized = True
        self.get_logger().info('Pinky Bringup with Dynamixel has been started successfully.')

    def twist_callback(self, msg: Twist):
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        new_left_v = linear_x - (angular_z * self.wheel_separation / 2.0)
        new_right_v = linear_x + (angular_z * self.wheel_separation / 2.0)

        # The navigation frame has been rotated 180 deg: new +X is the physical rear.
        # Convert the desired new-frame wheel speeds back to the existing motor wiring.
        old_left_v = -new_right_v
        old_right_v = -new_left_v

        wheel_rads_l = old_left_v / self.wheel_radius
        wheel_rads_r = old_right_v / self.wheel_radius

        rpm_l = wheel_rads_l * 60.0 / (2 * math.pi)
        rpm_r = -wheel_rads_r * 60.0 / (2 * math.pi)

        max_val = max(abs(rpm_l), abs(rpm_r))
        MAX_RPM = 100.0
        if max_val > MAX_RPM:
            scale = MAX_RPM / max_val
            rpm_l *= scale
            rpm_r *= scale

        if not self.driver.set_double_rpm(rpm_l, rpm_r):
            self.get_logger().warn("Failed to send motor command.")

    def update_and_publish(self):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        if dt <= 0: return

        feedback = self.driver.get_feedback()
        if feedback[0] is None:
            self.get_logger().warn("Failed to read motor data. Skipping update cycle.")
            return
        rpm_l, rpm_r, encoder_l, encoder_r = feedback

        delta_l = encoder_l - self.last_encoder_l
        delta_r = -(encoder_r - self.last_encoder_r)
        
        self.last_encoder_l = encoder_l
        self.last_encoder_r = encoder_r

        old_dist_l = (delta_l / PULSE_PER_ROT) * self.circumference
        old_dist_r = (delta_r / PULSE_PER_ROT) * self.circumference

        # New +X points toward the physical rear. New left/right are old right/left,
        # with distance signs inverted so rearward physical motion increases odom X.
        dist_l = -old_dist_r
        dist_r = -old_dist_l

        delta_distance = (dist_r + dist_l) / 2.0
        delta_theta = (dist_r - dist_l) / self.wheel_separation
        
        self.theta += delta_theta
        self.x += delta_distance * math.cos(self.theta)
        self.y += delta_distance * math.sin(self.theta)
        
        v_x = delta_distance / dt if dt > 0 else 0.0
        vth = delta_theta / dt if dt > 0 else 0.0

        self._publish_tf(current_time)
        self._publish_odometry(current_time, v_x, vth)
        self._publish_joint_states(current_time, rpm_l, rpm_r)

        self.last_time = current_time

    def _publish_tf(self, current_time):
        t = TransformStamped()
        t.header.stamp = current_time.to_msg()
        t.header.frame_id = self.odom_frame_id
        t.child_frame_id = self.odom_child_frame_id
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = quaternion_from_euler(0, 0, self.theta)
        t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = q
        self.tf_broadcaster.sendTransform(t)

    def _publish_odometry(self, current_time, v_x, vth):
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame_id
        odom_msg.child_frame_id = self.odom_child_frame_id
        odom_msg.pose.pose.position.x, odom_msg.pose.pose.position.y = self.x, self.y
        q = quaternion_from_euler(0, 0, self.theta)
        odom_msg.pose.pose.orientation.x, odom_msg.pose.pose.orientation.y, odom_msg.pose.pose.orientation.z, odom_msg.pose.pose.orientation.w = q
        odom_msg.twist.twist.linear.x, odom_msg.twist.twist.angular.z = v_x, vth
        self.odom_pub.publish(odom_msg)

    def _publish_joint_states(self, current_time, rpm_l, rpm_r):
        joint_msg = JointState()
        joint_msg.header.stamp = current_time.to_msg()
        joint_msg.name = [self.left_wheel_joint, self.right_wheel_joint]
        
        pos_l_rad = (self.last_encoder_l / PULSE_PER_ROT) * (2 * math.pi)
        pos_r_rad = (self.last_encoder_r / PULSE_PER_ROT) * (2 * math.pi)
        joint_msg.position = [pos_l_rad, pos_r_rad]
        joint_msg.velocity = [rpm_l * RPM2RAD, rpm_r * RPM2RAD]

        self.joint_pub.publish(joint_msg)

    def battery_voltage_callback(self, msg):
        self.current_voltage = msg.data
        
        if self.current_voltage is None:
            self.get_logger().warn("Battery voltage data has not been received yet.")
        elif self.current_voltage <= LOW_BATTERY_THRESHOLD:
            self.get_logger().warn(
                f"!!! LOW BATTERY WARNING !!! Voltage: {self.current_voltage:.2f}V. Please charge the robot."
            )


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = Pinky()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.driver.terminate()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
