#ifndef _RM_SERIAL_DRIVER_NODE_HPP
#define _RM_SERIAL_DRIVER_NODE_HPP

#include <rclcpp/rclcpp.hpp>

#include <rcpputils/filesystem_helper.hpp>
#include <serial_driver/serial_driver.hpp>
#include <io_context/io_context.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int8.hpp>

#include <array>
#include <atomic>
#include <chrono>
#include <vector>
#include <mutex>

#include <rm_serial_driver/packet.hpp>

namespace rm::serial
{
class DriverNode : public rclcpp::Node
{
public:
  explicit DriverNode(const rclcpp::NodeOptions& options);
  ~DriverNode() override;

private:
  drivers::serial_driver::SerialPortConfig getDeviceConfig();
  
  void navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg);
  void armCommandCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg);
  void gripperCommandCallback(std_msgs::msg::Int8::ConstSharedPtr msg);
  void timerCallback();
  
  void startSerialReceive();
  void serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size);
  void publishArmState(const ReceivePacket& packet);

  void reopenPort();
  void updateArmCommandLocked(double dt_sec);
  float angleToSerialUnit(float angle_rad, std::size_t joint_index) const;
  float angleFromSerialUnit(float serial_angle, std::size_t joint_index) const;
  float serialAngleRadToUnit(float serial_angle_rad) const;
  float serialAngleRadFromUnit(float serial_angle) const;

  // ******************** ROS Subscription ********************
  // Nav Subscription
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_command_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr gripper_command_sub_;

  // ******************** ROS Publisher ********************
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_state_pub_;

  // ******************** ROS Timer ********************
  rclcpp::TimerBase::SharedPtr timer_;

  // ******************** Send ********************
  NavPacket nav_packet_;
  ArmJointPacket arm_packet_;
  std::array<float, kArmJointCount> arm_target_position_{};
  std::array<float, kArmJointCount> arm_command_position_rad_{};
  std::array<float, kArmJointCount> max_arm_command_velocity_rad_s_{};
  std::vector<std::string> arm_joint_names_;
  uint32_t arm_sequence_{0};
  bool arm_serial_enabled_{true};
  bool include_arm_extension_{true};
  bool suppress_expected_serial_abort_logs_{true};
  bool arm_serial_angles_in_degrees_{false};
  bool have_arm_command_{false};
  bool logged_first_arm_command_{false};
  bool logged_first_enabled_serial_packet_{false};
  int8_t gripper_command_{1};
  double gripper_command_log_interval_sec_{1.0};
  double arm_command_publish_hz_{100.0};
  double arm_command_speed_scale_{1.0};
  double arm_command_goal_tolerance_rad_{0.005};
  std::chrono::steady_clock::time_point last_send_time_{};
  std::chrono::steady_clock::time_point last_gripper_command_log_time_{};

  // ******************** Mutex ********************
  std::mutex nav_mutex_;
  std::mutex arm_mutex_;
  
  // ******************** Serial Port ********************
  std::mutex serial_port_mutex_;
  std::atomic_bool serial_receive_enabled_{false};
  std::atomic_bool serial_reopen_in_progress_{false};
  drivers::common::IoContext owned_ctx_;
  drivers::serial_driver::SerialDriver serial_driver_;
};
}

#endif
