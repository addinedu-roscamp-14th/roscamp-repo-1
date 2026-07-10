#!/usr/bin/env python3
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node


class TargetMapPoseToNavGoal(Node):
    def __init__(self):
        super().__init__("target_map_pose_to_nav_goal")

        self.declare_parameter("target_topic", "/central/target_map_pose")
        self.declare_parameter("action_name", "navigate_to_pose")
        self.declare_parameter("default_frame_id", "map")
        self.declare_parameter("wait_for_server_timeout_sec", 2.0)
        self.declare_parameter("ignore_empty_frame_id", False)

        target_topic = self.get_parameter("target_topic").value
        action_name = self.get_parameter("action_name").value

        self._default_frame_id = self.get_parameter("default_frame_id").value
        self._wait_timeout = self.get_parameter("wait_for_server_timeout_sec").value
        self._ignore_empty_frame_id = self.get_parameter("ignore_empty_frame_id").value

        self._client = ActionClient(self, NavigateToPose, action_name)
        self._active_goal = None
        self._target_topic = target_topic
        self._action_name = action_name
        self._received_pose_count = 0

        self.create_subscription(PoseStamped, target_topic, self._on_target_pose, 10)
        self.create_timer(5.0, self._log_waiting_status)
        self.get_logger().info(
            f"subscribing to {target_topic} and sending goals to {action_name}"
        )

    def _on_target_pose(self, pose_msg):
        self._received_pose_count += 1

        goal = NavigateToPose.Goal()
        goal.pose = pose_msg

        if not goal.pose.header.frame_id:
            if self._ignore_empty_frame_id:
                self.get_logger().warn("ignored target pose with empty frame_id")
                return
            goal.pose.header.frame_id = self._default_frame_id

        if goal.pose.header.stamp.sec == 0 and goal.pose.header.stamp.nanosec == 0:
            goal.pose.header.stamp = self.get_clock().now().to_msg()

        if not self._client.wait_for_server(timeout_sec=self._wait_timeout):
            self.get_logger().error("navigate_to_pose action server is not available")
            return

        if self._active_goal is not None:
            self.get_logger().info("canceling previous navigation goal")
            self._active_goal.cancel_goal_async()
            self._active_goal = None

        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._on_goal_response)

        position = goal.pose.pose.position
        orientation = goal.pose.pose.orientation
        self.get_logger().info(
            "sent target pose goal: "
            f"frame={goal.pose.header.frame_id}, "
            f"x={position.x:.3f}, y={position.y:.3f}, "
            f"qz={orientation.z:.3f}, qw={orientation.w:.3f}"
        )

    def _log_waiting_status(self):
        if self._received_pose_count > 0:
            return

        publisher_count = self.count_publishers(self._target_topic)
        server_ready = self._client.server_is_ready()
        self.get_logger().info(
            "waiting for target pose: "
            f"topic={self._target_topic}, "
            f"publishers={publisher_count}, "
            f"action={self._action_name}, "
            f"action_server_ready={server_ready}"
        )

    def _on_goal_response(self, future):
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn("navigation goal was rejected")
            return

        self._active_goal = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._on_goal_result)

    def _on_goal_result(self, future):
        result = future.result()
        self._active_goal = None
        self.get_logger().info(
            "navigation finished: "
            f"status={result.status}, "
            f"error_code={result.result.error_code}, "
            f"error_msg={result.result.error_msg!r}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = TargetMapPoseToNavGoal()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
