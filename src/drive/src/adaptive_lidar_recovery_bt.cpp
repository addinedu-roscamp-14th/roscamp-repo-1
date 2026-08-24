#include <cmath>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/condition_node.h"
#include "drive/srv/adaptive_lidar_recovery.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"
#include "nav_msgs/msg/path.hpp"

namespace drive
{

class AdaptiveLidarRecoveryBt
  : public nav2_behavior_tree::BtServiceNode<drive::srv::AdaptiveLidarRecovery>
{
public:
  AdaptiveLidarRecoveryBt(
    const std::string & service_node_name,
    const BT::NodeConfiguration & configuration)
  : BtServiceNode(service_node_name, configuration)
  {
  }

  static BT::PortsList providedPorts()
  {
    return providedBasicPorts({
      BT::InputPort<bool>(
        "opposite", false,
        "Use the opposite side instead of preserving the first direction"),
      BT::InputPort<bool>(
        "reset_direction", false,
        "Select and store a new widest free-space direction"),
      BT::OutputPort<bool>("recovery_anchor_valid"),
      BT::OutputPort<std::string>("recovery_anchor_frame"),
      BT::OutputPort<double>("recovery_anchor_x"),
      BT::OutputPort<double>("recovery_anchor_y")
    });
  }

  void on_tick() override
  {
    request_->opposite = getInput<bool>("opposite").value();
    request_->reset_direction = getInput<bool>("reset_direction").value();
  }

  BT::NodeStatus on_completion(
    std::shared_ptr<drive::srv::AdaptiveLidarRecovery::Response> response) override
  {
    if (!response->success) {
      RCLCPP_ERROR(
        node_->get_logger(), "Adaptive LiDAR recovery failed: %s",
        response->message.c_str());
      return BT::NodeStatus::FAILURE;
    }
    RCLCPP_WARN(
      node_->get_logger(), "Adaptive LiDAR recovery completed: %s",
      response->message.c_str());
    setOutput("recovery_anchor_valid", response->recovery_anchor_valid);
    setOutput("recovery_anchor_frame", response->recovery_anchor_frame);
    setOutput("recovery_anchor_x", response->recovery_anchor_x);
    setOutput("recovery_anchor_y", response->recovery_anchor_y);
    return BT::NodeStatus::SUCCESS;
  }
};

class PathEscapesRecoveryPose : public BT::ConditionNode
{
public:
  PathEscapesRecoveryPose(
    const std::string & name, const BT::NodeConfiguration & configuration)
  : BT::ConditionNode(name, configuration)
  {
  }

  static BT::PortsList providedPorts()
  {
    return {
      BT::InputPort<nav_msgs::msg::Path>("path"),
      BT::InputPort<bool>(
        "recovery_anchor_valid", false, "A recovery anchor is available"),
      BT::InputPort<std::string>(
        "recovery_anchor_frame", std::string{}, "Recovery anchor frame"),
      BT::InputPort<double>("recovery_anchor_x", 0.0, "Recovery anchor X"),
      BT::InputPort<double>("recovery_anchor_y", 0.0, "Recovery anchor Y"),
      BT::InputPort<double>(
        "sample_distance", 0.05, "Initial path distance used for direction"),
      BT::InputPort<double>(
        "toward_cosine_threshold", 0.25,
        "Reject paths whose initial direction points back to the anchor")
    };
  }

  BT::NodeStatus tick() override
  {
    const auto anchor_valid_input = getInput<bool>("recovery_anchor_valid");
    if (!anchor_valid_input || !anchor_valid_input.value()) {
      // Before the first physical recovery the blackboard has no anchor.
      // That is a normal navigation state, not a BehaviorTree exception.
      return BT::NodeStatus::SUCCESS;
    }

    const auto path_input = getInput<nav_msgs::msg::Path>("path");
    const auto anchor_frame_input = getInput<std::string>("recovery_anchor_frame");
    const auto anchor_x_input = getInput<double>("recovery_anchor_x");
    const auto anchor_y_input = getInput<double>("recovery_anchor_y");
    if (!path_input || !anchor_frame_input || !anchor_x_input || !anchor_y_input) {
      // An incomplete anchor must not terminate bt_navigator. Let FollowPath
      // handle the current planner result and trigger normal recovery.
      return BT::NodeStatus::SUCCESS;
    }

    const auto & path = path_input.value();
    if (path.poses.size() < 2) {
      return BT::NodeStatus::SUCCESS;
    }
    const auto & anchor_frame = anchor_frame_input.value();
    const auto path_frame = path.header.frame_id.empty() ?
      path.poses.front().header.frame_id : path.header.frame_id;
    if (anchor_frame.empty() || path_frame != anchor_frame) {
      return BT::NodeStatus::SUCCESS;
    }

    const auto & start = path.poses.front().pose.position;
    const double sample_distance =
      getInput<double>("sample_distance").value_or(0.05);
    const auto * sample = &path.poses.back().pose.position;
    for (const auto & stamped_pose : path.poses) {
      const auto & point = stamped_pose.pose.position;
      if (std::hypot(point.x - start.x, point.y - start.y) >= sample_distance) {
        sample = &point;
        break;
      }
    }

    const double motion_x = sample->x - start.x;
    const double motion_y = sample->y - start.y;
    const double anchor_x = anchor_x_input.value() - start.x;
    const double anchor_y = anchor_y_input.value() - start.y;
    const double motion_norm = std::hypot(motion_x, motion_y);
    const double anchor_norm = std::hypot(anchor_x, anchor_y);
    if (motion_norm < 1e-3 || anchor_norm < 1e-3) {
      return BT::NodeStatus::SUCCESS;
    }
    const double cosine =
      (motion_x * anchor_x + motion_y * anchor_y) /
      (motion_norm * anchor_norm);
    const double toward_cosine_threshold =
      getInput<double>("toward_cosine_threshold").value_or(0.25);
    if (cosine > toward_cosine_threshold) {
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace drive

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<drive::AdaptiveLidarRecoveryBt>(
    "AdaptiveLidarRecovery");
  factory.registerNodeType<drive::PathEscapesRecoveryPose>(
    "PathEscapesRecoveryPose");
}
