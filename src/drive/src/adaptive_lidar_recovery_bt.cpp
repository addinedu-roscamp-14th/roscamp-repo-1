#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "drive/srv/adaptive_lidar_recovery.hpp"
#include "nav2_behavior_tree/bt_service_node.hpp"

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
        "Select and store a new widest free-space direction")
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
    return BT::NodeStatus::SUCCESS;
  }
};

}  // namespace drive

BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<drive::AdaptiveLidarRecoveryBt>(
    "AdaptiveLidarRecovery");
}
