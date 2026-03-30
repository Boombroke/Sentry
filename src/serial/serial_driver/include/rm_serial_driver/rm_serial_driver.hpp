
#ifndef RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
#define RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_

#include <tf2_ros/transform_broadcaster.h>

#include "auto_aim_interfaces/msg/target.hpp"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include "rm_interfaces/msg/game_status.hpp"
#include "rm_interfaces/msg/robot_status.hpp"
#include "rm_interfaces/msg/game_robot_hp.hpp"
#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/subscription.hpp>
#include <serial_driver/serial_driver.hpp>
#include <std_msgs/msg/float64.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <rm_interfaces/msg/rfid_status.hpp>
#include <nav_msgs/msg/odometry.hpp>

// C++ system
#include <future>
#include <memory>
#include <string>
#include <thread>
#include <vector>



namespace rm_serial_driver
{
class RMSerialDriver : public rclcpp::Node
{
public:
  explicit RMSerialDriver(const rclcpp::NodeOptions & options);

  ~RMSerialDriver() override;

private:
  void getParams();

  void receiveData();

  void sendnavData(const geometry_msgs::msg::Twist::SharedPtr msg);
  
  void receiveNavData();

  void odometry_callback(const nav_msgs::msg::Odometry msg);

  void reopenPort();
  void publishNavData(const ReceiveNavPacket & data);
  void publishTreeDate(const ReceiveNavPacket & data);
  // Serial port
  std::unique_ptr<IoContext> owned_ctx_;
  std::string device_name_;
  std::unique_ptr<drivers::serial_driver::SerialPortConfig> device_config_;
  std::unique_ptr<drivers::serial_driver::SerialDriver> serial_driver_;

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_sub_;

  nav_msgs::msg::Odometry odometry_;
  nav_msgs::msg::Odometry last_odom_pos_;
  double vec_vel,vel_x,vel_y;

  // For debug usage
  rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr latency_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameStatus>::SharedPtr game_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RobotStatus>::SharedPtr robot_status_pub_;
  rclcpp::Publisher<rm_interfaces::msg::GameRobotHP>::SharedPtr allHP_pub_;
  rclcpp::Publisher<auto_aim_interfaces::msg::Target>::SharedPtr tracker_pub_;
  rclcpp::Publisher<rm_interfaces::msg::RfidStatus>::SharedPtr rfid_pub_;

  std::thread receive_thread_;
};
}  // namespace rm_serial_driver

#endif  // RM_SERIAL_DRIVER__RM_SERIAL_DRIVER_HPP_
