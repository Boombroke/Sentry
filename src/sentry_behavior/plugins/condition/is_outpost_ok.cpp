#include "sentry_behavior/plugins/condition/is_outpost_ok.hpp"

namespace sentry_behavior
{

IsOutpostOkCondition::IsOutpostOkCondition(
const std::string & name, const BT::NodeConfig & config)
: BT::SimpleConditionNode(name, std::bind(&IsOutpostOkCondition::checkOutpost, this), config)
{
}

BT::NodeStatus IsOutpostOkCondition::checkOutpost()
{
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    RCLCPP_ERROR(logger_, "Outpost message is not available");
    return BT::NodeStatus::FAILURE;
  }
  bool team_colour = msg->team_colour;
  if(team_colour==0){
    if(msg->red_outpost_hp>0){
      return BT::NodeStatus::SUCCESS;
    }
    else if(msg->blue_outpost_hp==0){
      return BT::NodeStatus::FAILURE;
    }
  }
  else if(team_colour==1){
    if(msg->blue_outpost_hp>0){
      return BT::NodeStatus::SUCCESS;
    }
    else if(msg->red_outpost_hp==0){
      return BT::NodeStatus::FAILURE;
    }
    
  }
  else{
    return BT::NodeStatus::FAILURE;
  }

  return BT::NodeStatus::FAILURE;
}

BT::PortsList IsOutpostOkCondition::providedPorts()
{
  return {
    BT::InputPort<rm_interfaces::msg::GameRobotHP>(
      "key_port", "{@referee_allRobotHP}", "Outpost port on blackboard")
  };
}
}  // namespace sentry_behavior

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::IsOutpostOkCondition>("IsOutpostOk");
}
