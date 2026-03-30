
#include <tf2/LinearMath/Quaternion.h>

#include "sensor_msgs/msg/joint_state.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/utilities.hpp>
#include <serial_driver/serial_driver.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

               
// C++ system
#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <vector>

#include "rm_serial_driver/crc.hpp"
#include "rm_serial_driver/packet.hpp"
#include "rm_serial_driver/rm_serial_driver.hpp"

namespace rm_serial_driver
{
RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions & options)
: Node("rm_serial_driver", options),
  owned_ctx_{new IoContext(2)},
  serial_driver_{new drivers::serial_driver::SerialDriver(*owned_ctx_)}
{
  RCLCPP_INFO(get_logger(), "Start RMSerialDriver!");
  getParams();
  // Create Subscription
  cmd_vel_sub_ =this->create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel",10,
    std::bind(&RMSerialDriver::sendnavData,this,std::placeholders::_1));
    
  odometry_sub_=this->create_subscription<nav_msgs::msg::Odometry>(
    "/odometry",10,
    std::bind(&RMSerialDriver::odometry_callback,this,std::placeholders::_1));
  // Create Publisher
  latency_pub_ = this->create_publisher<std_msgs::msg::Float64>("/latency", 10);
  joint_state_pub_= this->create_publisher<sensor_msgs::msg::JointState>("serial/gimbal_joint_state",10);
  game_status_pub_= this->create_publisher<rm_interfaces::msg::GameStatus>("referee/game_status",10);
  robot_status_pub_= this->create_publisher<rm_interfaces::msg::RobotStatus>("referee/robot_status",10);
  allHP_pub_= this->create_publisher<rm_interfaces::msg::GameRobotHP>("referee/all_robot_hp",10);
  tracker_pub_ = this->create_publisher<auto_aim_interfaces::msg::Target>("tracker/target", 10);
  rfid_pub_ = this->create_publisher<rm_interfaces::msg::RfidStatus>("referee/rfidStatus", 10);
  try {
    serial_driver_->init_port(device_name_, *device_config_);
    if (!serial_driver_->port()->is_open()) {
      serial_driver_->port()->open();
      receive_thread_ = std::thread(&RMSerialDriver::receiveNavData, this);
    }
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(
      get_logger(), "Error creating serial port: %s - %s", device_name_.c_str(), ex.what());
    throw ex;
  }


}

RMSerialDriver::~RMSerialDriver()
{
  if (receive_thread_.joinable()) {
    receive_thread_.join();
  }

  if (serial_driver_->port()->is_open()) {
    serial_driver_->port()->close();
  }

  if (owned_ctx_) {
    owned_ctx_->waitForExit();
  }
}

void RMSerialDriver::receiveNavData()
{
  std::vector<uint8_t> header(1);
  std::vector<uint8_t> data;
  data.reserve(sizeof(ReceiveNavPacket));
  while(rclcpp::ok()){
    try{
      serial_driver_->port()->receive(header);
      if(header[0]==0x5B){
        data.resize(sizeof(ReceiveNavPacket)-1);
        serial_driver_->port()->receive(data);
        data.insert(data.begin(), header[0]);
        ReceiveNavPacket packet = fromVector(data);
        
        bool crc_ok = crc16::Verify_CRC16_Check_Sum(reinterpret_cast<const uint8_t *>(&packet), sizeof(packet));
        if (crc_ok) {
          RCLCPP_INFO(get_logger(), "CRC OK!");
          //RCLCPP_INFO(get_logger(),"now hp %d",packet.current_hp);
          //RCLCPP_INFO(get_logger(),"tracker_X %f",packet.tracker_x);
          //RCLCPP_INFO(get_logger(),"tracker_y %f",packet.tracker_y);
          
          
          publishNavData(packet);
          publishTreeDate(packet);
        } else {
          RCLCPP_ERROR(get_logger(), "CRC error!");
          continue;
        }
      }else {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 20, "Invalid header: %02X", header[0]);
      }
    }catch (const std::exception & ex) {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 20, "Error while receiving data: %s", ex.what());
      reopenPort();
    }
  }
}

void RMSerialDriver::publishNavData(const ReceiveNavPacket  &   joint_state  )
{
  sensor_msgs::msg::JointState msg;
  msg.position.resize(2);
  msg.name.resize(2);
  msg.header.stamp = now();
  msg.name[0] = "gimbal_pitch_joint";
  msg.position[0] = joint_state.pitch;
  msg.name[1] = "gimbal_yaw_joint";
  msg.position[1] = joint_state.yaw;
  joint_state_pub_->publish(msg);
}

void RMSerialDriver::publishTreeDate(const ReceiveNavPacket  &   tree  )
{
  rm_interfaces::msg::RobotStatus robot;
  rm_interfaces::msg::GameStatus game;
  rm_interfaces::msg::GameRobotHP allHP;
  auto_aim_interfaces::msg::Target target;
  rm_interfaces::msg::RfidStatus rfidStatus;

  target.position.x=tree.tracker_x;
  target.position.y=tree.tracker_y;
  // bool add_ok=0;
  // int last_hp_=0;
  // int now_hp=tree.current_hp;
  // if(!last_hp_){
  //   last_hp_=now_hp;
  // }
  // else{
  //   add_ok=(now_hp>last_hp_);
  //   RCLCPP_INFO(get_logger(),"last_hp_=%d",last_hp_);
  // }
  // last_hp_=now_hp;
  // robot.add_ok=add_ok;

  game.game_progress=tree.game_progress;
  game.stage_remain_time=tree.stage_remain_time;
  // game.behavior_state=tree.behavior_state;
  robot.current_hp=tree.current_hp;
  robot.projectile_allowance_17mm=tree.projectile_allowance_17mm;
  allHP.red_1_robot_hp=tree.red_1_robot_hp;
  allHP.red_2_robot_hp=tree.red_2_robot_hp;
  allHP.red_3_robot_hp=tree.red_3_robot_hp;
  allHP.red_4_robot_hp=tree.red_4_robot_hp;
  allHP.red_7_robot_hp=tree.red_7_robot_hp;
  allHP.red_base_hp=tree.red_base_hp;
  allHP.red_outpost_hp=tree.red_outpost_hp;
  allHP.blue_1_robot_hp=tree.blue_1_robot_hp;
  allHP.blue_2_robot_hp=tree.blue_2_robot_hp;
  allHP.blue_3_robot_hp=tree.blue_3_robot_hp;
  allHP.blue_4_robot_hp=tree.blue_4_robot_hp;
  allHP.blue_7_robot_hp=tree.blue_7_robot_hp;
  allHP.blue_base_hp=tree.blue_base_hp;
  allHP.blue_outpost_hp=tree.blue_outpost_hp;
  allHP.team_colour=tree.team_colour;
  rfidStatus.friendly_supply_zone_non_exchange =tree.base_gain_point;
  game_status_pub_->publish(game);
  robot_status_pub_->publish(robot);
  allHP_pub_->publish(allHP);
  tracker_pub_->publish(target);
  rfid_pub_->publish(rfidStatus);
}


void RMSerialDriver::sendnavData(const geometry_msgs::msg::Twist::SharedPtr msg)
{
  try{
    //头 0xA1
    SendNavPacket packet;
    // packet.aim_lx=msg->linear.x;
    // packet.aim_ly=msg->linear.y;
    // packet.aim_az=msg->angular.z;
    packet.vel_x=msg->linear.x;
    packet.vel_y=msg->linear.y;
    packet.vel_v=msg->angular.z;
    
    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&packet), sizeof(packet));
    
    std::vector<uint8_t> data = toVector(packet);

    serial_driver_->port()->send(data);
    // std_msgs::msg::Float64 latency;
    // latency.data = (this->now() - msg->header.stamp).seconds() * 1000.0;
    // RCLCPP_INFO(get_logger(), "Total latency: %f  ms" ,latency.data);
    // latency_pub_->publish(latency);

  }catch (const  std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while sending data: %s", ex.what());
    reopenPort();
  }
}


void RMSerialDriver::getParams()
{
  using FlowControl = drivers::serial_driver::FlowControl;
  using Parity = drivers::serial_driver::Parity;
  using StopBits = drivers::serial_driver::StopBits;

  uint32_t baud_rate{};
  auto fc = FlowControl::NONE;
  auto pt = Parity::NONE;
  auto sb = StopBits::ONE;

  try {
    device_name_ = declare_parameter<std::string>("device_name", "");
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The device name provided was invalid");
    throw ex;
  }

  try {
    baud_rate = declare_parameter<int>("baud_rate", 0);
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The baud_rate provided was invalid");
    throw ex;
  }

  try {
    const auto fc_string = declare_parameter<std::string>("flow_control", "");
    if (fc_string == "none") {
      fc = FlowControl::NONE;
    } else if (fc_string == "hardware") {
      fc = FlowControl::HARDWARE;
    } else if (fc_string == "software") {
      fc = FlowControl::SOFTWARE;
    } else {
      throw std::invalid_argument{
        "The flow_control parameter must be one of: none, software, or hardware."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The flow_control provided was invalid");
    throw ex;
  }

  try {
    const auto pt_string = declare_parameter<std::string>("parity", "");

    if (pt_string == "none") {
      pt = Parity::NONE;
    } else if (pt_string == "odd") {
      pt = Parity::ODD;
    } else if (pt_string == "even") {
      pt = Parity::EVEN;
    } else {
      throw std::invalid_argument{"The parity parameter must be one of: none, odd, or even."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The parity provided was invalid");
    throw ex;
  }

  try {
    const auto sb_string = declare_parameter<std::string>("stop_bits", "");

    if (sb_string == "1" || sb_string == "1.0") {
      sb = StopBits::ONE;
    } else if (sb_string == "1.5") {
      sb = StopBits::ONE_POINT_FIVE;
    } else if (sb_string == "2" || sb_string == "2.0") {
      sb = StopBits::TWO;
    } else {
      throw std::invalid_argument{"The stop_bits parameter must be one of: 1, 1.5, or 2."};
    }
  } catch (rclcpp::ParameterTypeException & ex) {
    RCLCPP_ERROR(get_logger(), "The stop_bits provided was invalid");
    throw ex;
  }

  device_config_ =
    std::make_unique<drivers::serial_driver::SerialPortConfig>(baud_rate, fc, pt, sb);
}

void RMSerialDriver::reopenPort()
{
  RCLCPP_WARN(get_logger(), "Attempting to reopen port");
  try {
    if (serial_driver_->port()->is_open()) {
      serial_driver_->port()->close();
    }
    serial_driver_->port()->open();
    RCLCPP_INFO(get_logger(), "Successfully reopened port");
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(get_logger(), "Error while reopening port: %s", ex.what());
    if (rclcpp::ok()) {
      rclcpp::sleep_for(std::chrono::seconds(1));
      reopenPort();
    }
  }
}

void RMSerialDriver::odometry_callback(const nav_msgs::msg::Odometry msg)
{
  auto timestamp_=(this->now()-msg.header.stamp).seconds();
  vec_vel=std::sqrt((msg.pose.pose.position.x-last_odom_pos_.pose.pose.position.x)*(msg.pose.pose.position.x-last_odom_pos_.pose.pose.position.x)+
            (msg.pose.pose.position.y-last_odom_pos_.pose.pose.position.y)*(msg.pose.pose.position.y-last_odom_pos_.pose.pose.position.y))/timestamp_;
  vel_x=std::sqrt((msg.pose.pose.position.x-last_odom_pos_.pose.pose.position.x)*(msg.pose.pose.position.x-last_odom_pos_.pose.pose.position.x))/timestamp_;
  vel_y=std::sqrt((msg.pose.pose.position.y-last_odom_pos_.pose.pose.position.y)*(msg.pose.pose.position.y-last_odom_pos_.pose.pose.position.y))/timestamp_;
  last_odom_pos_=msg;
  RCLCPP_INFO(get_logger(),"----------");
  RCLCPP_INFO(get_logger(),"linear x: %f",vel_x);
  RCLCPP_INFO(get_logger(),"linear y: %f",vel_y);
  RCLCPP_INFO(get_logger(),"linear vec: %f",vec_vel);
  RCLCPP_INFO(get_logger(),"----------");
  odometry_.header.stamp=now();
  odometry_=msg;
}

}  // namespace rm_serial_driver

#include "rclcpp_components/register_node_macro.hpp"

// Register the component with class_loader.
// This acts as a sort of entry point, allowing the component to be discoverable when its library
// is being loaded into a running process.
RCLCPP_COMPONENTS_REGISTER_NODE(rm_serial_driver::RMSerialDriver)
