#include "sentry_behavior/plugins/action/battlefield_information.hpp"

namespace sentry_behavior
{
BattlefieldInformationAction::BattlefieldInformationAction(
const std::string& name,const BT::NodeConfiguration& config)
: BT::SyncActionNode(name, config),
logger_(rclcpp::get_logger("BattlefieldInformationAction"))
{}

BT::PortsList BattlefieldInformationAction::providedPorts(){

    return{
      BT::InputPort<rm_interfaces::msg::GameRobotHP>(
        "key_port", "{@referee_allRobotHP}", "allRobotHP port on blackboard"),
        BT::OutputPort<std::string>("weight")
    };

}

BT::NodeStatus BattlefieldInformationAction::tick() {
  auto msg = getInput<rm_interfaces::msg::GameRobotHP>("key_port");
  if (!msg) {
    return BT::NodeStatus::FAILURE;
    RCLCPP_ERROR(logger_, "allRobotHP message is not available");
  } 
  int colour=msg->team_colour;
  int red1=msg->red_1_robot_hp;
  //int red2=msg->red_2_robot_hp;
  int red3=msg->red_3_robot_hp;
  int red4=msg->red_4_robot_hp;
  int red7=msg->red_7_robot_hp;
  //int red_base=msg->red_base_hp;
  //int red_outpost=msg->red_outpost_hp;
  int blue1=msg->blue_1_robot_hp;
  //int blue2=msg->blue_2_robot_hp;
  int blue3=msg->blue_3_robot_hp;
  int blue4=msg->blue_4_robot_hp;
  int blue7=msg->blue_7_robot_hp;
  //int blue_base=msg->blue_base_hp;
  //int blue_outpost=msg->blue_outpost_hp;
  
  float rate=0.0;
  if(colour==1){
    rate =(static_cast<float>(red1+red3+red4+red7)/static_cast<float>(blue1+blue3+blue4+blue7));
    //rmuc
    if(rate<=1.2 && rate>=0.8){
      setOutput("weight","1.0");
    }
    else if (rate>1.2){
      setOutput("weight","2.0");
    }
    else if (rate<0.8){
      setOutput("weight","0.0");
    }
    else{
      RCLCPP_ERROR(logger_,"rate error!");
      return BT::NodeStatus::FAILURE;
    }
    //test
    // if(rate<=1.05 && rate>=0.95){
    //   setOutput("weight","1.0");
    // }
    // else if (rate>1.05){
    //   setOutput("weight","2.0");
    // }
    // else if (rate<0.95){
    //   setOutput("weight","0.0");
    // }
    // else{
    //   RCLCPP_ERROR(logger_,"rate error!");
    //   return BT::NodeStatus::FAILURE;
    // }


    
  }
  else if (colour==0){
    rate =(static_cast<float>(blue1+blue3+blue4+blue7)/static_cast<float>(red1+red3+red4+red7));
    //rmuc
  if(rate<=1.05 && rate>=0.95){
    setOutput("weight","1.0");
  }
  else if (rate>1.05){
    setOutput("weight","2.0");
  }
  else if (rate<0.95){
    setOutput("weight","0.0");
  }
  else{
    RCLCPP_ERROR(logger_,"rate error!");
    return BT::NodeStatus::FAILURE;
  }
  
  //test
  // if(rate<=1.05 && rate>=0.95){
  //   setOutput("weight","1.0");
  // }
  // else if (rate>1.05){
  //   setOutput("weight","2.0");
  // }
  // else if (rate<0.95){
  //   setOutput("weight","0.0");
  // }
  // else{
  //   RCLCPP_ERROR(logger_,"rate error!");
  //   return BT::NodeStatus::FAILURE;
  // }
  }

  return BT::NodeStatus::SUCCESS;

  
}


}

#include "behaviortree_cpp/bt_factory.h"
BT_REGISTER_NODES(factory)
{
  factory.registerNodeType<sentry_behavior::BattlefieldInformationAction>("BattlefieldInformation");
}
