// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#include <rclcpp/rclcpp.hpp>
#include <rcutils/logging.h>
#include <serial_driver/serial_driver.hpp>
#include <rcpputils/filesystem_helper.hpp>

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cstdint>
#include <cmath>
#include <functional>
#include <iterator>
#include <stdexcept>
#include <string>
#include <thread>
#include <unordered_map>

#include <rm_serial_driver/node.hpp>

namespace rm::serial
{
namespace
{
constexpr double kPi = 3.14159265358979323846;

std::string normalizeAngleUnit(std::string unit)
{
  std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return unit;
}

bool isDegreeUnit(const std::string& unit)
{
  return unit == "deg" || unit == "degree" || unit == "degrees";
}

bool isRadianUnit(const std::string& unit)
{
  return unit == "rad" || unit == "radian" || unit == "radians";
}
}  // namespace

DriverNode::DriverNode(const rclcpp::NodeOptions& options)
: Node("serial_driver", options),
  owned_ctx_(2), serial_driver_(owned_ctx_)
{
  using namespace std::placeholders;

  auto device_name = declare_parameter<std::string>("device_name");
  auto device_config = getDeviceConfig();
  arm_serial_enabled_ = declare_parameter<bool>("tzb.arm_serial_enabled", true);
  include_arm_extension_ = declare_parameter<bool>("tzb.include_arm_serial_extension", true);
  suppress_expected_serial_abort_logs_ = declare_parameter<bool>(
    "tzb.suppress_expected_serial_abort_logs", true);
  if (suppress_expected_serial_abort_logs_)
  {
    // serial_driver logs this expected cancellation directly from its dependency.
    const auto logging_result = rcutils_logging_set_logger_level(
      "SerialPort::async_receive_handler", RCUTILS_LOG_SEVERITY_FATAL);
    if (logging_result != RCUTILS_RET_OK)
    {
      rcutils_reset_error();
    }
  }
  const auto arm_serial_angle_unit =
    normalizeAngleUnit(declare_parameter<std::string>("tzb.arm_serial_angle_unit", "rad"));
  if (!isRadianUnit(arm_serial_angle_unit) && !isDegreeUnit(arm_serial_angle_unit))
  {
    throw std::runtime_error("tzb.arm_serial_angle_unit must be 'rad' or 'deg'");
  }
  arm_serial_angles_in_degrees_ = isDegreeUnit(arm_serial_angle_unit);
  arm_command_publish_hz_ =
    std::max(1.0, declare_parameter<double>("tzb.arm_command_publish_hz", 100.0));
  arm_command_speed_scale_ =
    std::max(1.0e-3, declare_parameter<double>("tzb.arm_command_speed_scale", 1.0));
  arm_command_goal_tolerance_rad_ =
    std::max(0.0, declare_parameter<double>("tzb.arm_command_goal_tolerance_rad", 0.005));
  gripper_command_log_interval_sec_ = std::max(
    0.0,
    declare_parameter<double>("tzb.gripper_command_log_interval_sec", 1.0));
  arm_joint_names_ = declare_parameter<std::vector<std::string>>(
    "tzb.arm_joint_names",
    std::vector<std::string>{
      "J1_joint", "J2_joint", "J3_joint", "J4_joint", "J5_joint", "J6_joint"});
  if (arm_joint_names_.size() != kArmJointCount)
  {
    throw std::runtime_error("tzb.arm_joint_names must contain exactly 6 joints");
  }
  const auto max_velocities = declare_parameter<std::vector<double>>(
    "tzb.max_arm_command_velocity_rad_s",
    std::vector<double>{1.2, 1.2, 1.2, 1.8, 1.8, 2.2});
  if (max_velocities.size() != kArmJointCount)
  {
    throw std::runtime_error("tzb.max_arm_command_velocity_rad_s must contain exactly 6 values");
  }
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    max_arm_command_velocity_rad_s_[i] = static_cast<float>(std::max(0.0, max_velocities[i]));
  }
  (void)declare_parameter<bool>("tzb.arm_command_publish_during_replay", false);
  (void)declare_parameter<bool>("tzb.sequence_loop", false);
  (void)declare_parameter<double>("tzb.loop_pause_sec", 0.5);

  nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
    "/cmd_vel", rclcpp::SensorDataQoS(),
    std::bind(&DriverNode::navCallback, this, _1));
  arm_command_sub_ = create_subscription<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("tzb.arm_command_topic", "/tzb_catch/arm_joint_command"),
    rclcpp::SensorDataQoS(),
    std::bind(&DriverNode::armCommandCallback, this, _1));
  gripper_command_sub_ = create_subscription<std_msgs::msg::Int8>(
    declare_parameter<std::string>(
      "tzb.gripper_command_topic", "/tzb_catch/gripper_command"),
    rclcpp::QoS(1).transient_local().reliable(),
    std::bind(&DriverNode::gripperCommandCallback, this, _1));
  arm_state_pub_ = create_publisher<sensor_msgs::msg::JointState>(
    declare_parameter<std::string>("tzb.arm_state_topic", "/serial/arm_joint_state"),
    rclcpp::SensorDataQoS());

  while (rclcpp::ok())
  {
    try
    {
      if (!rcpputils::fs::exists(rcpputils::fs::path(device_name)))
      {
        RCLCPP_WARN(get_logger(), "Serial device %s not found, retrying...", device_name.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      serial_driver_.init_port(device_name, device_config);

      if (!serial_driver_.port()->is_open())
      {
        serial_driver_.port()->open();
      }

      if (serial_driver_.port()->is_open())
      {
        serial_receive_enabled_.store(true);
        startSerialReceive();
        break;
      }
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR(get_logger(), "Error with serial port %s: %s", device_name.c_str(), ex.what());
    }

    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  const auto send_period = std::chrono::duration_cast<std::chrono::nanoseconds>(
    std::chrono::duration<double>(1.0 / arm_command_publish_hz_));
  timer_ = create_wall_timer(send_period, std::bind(&DriverNode::timerCallback, this));
  RCLCPP_INFO(
    get_logger(),
    "Started serial driver: arm_serial=%s arm_extension=%s arm_angle_unit=%s arm_hz=%.2f speed_scale=%.3f packet_size=%zu",
    arm_serial_enabled_ ? "true" : "false",
    include_arm_extension_ ? "true" : "false",
    arm_serial_angles_in_degrees_ ? "deg" : "rad",
    arm_command_publish_hz_,
    arm_command_speed_scale_,
    SendPacketSize(include_arm_extension_ && arm_serial_enabled_));
}

DriverNode::~DriverNode()
{
  {
    serial_receive_enabled_.store(false);
    std::scoped_lock lock{serial_port_mutex_};
    if (serial_driver_.port()->is_open())
    {
      serial_driver_.port()->close();
    }
  }

  owned_ctx_.waitForExit();
}

void DriverNode::navCallback(geometry_msgs::msg::Twist::ConstSharedPtr msg)
{
  try
  {
    std::scoped_lock lock{nav_mutex_};
    nav_packet_.linear_x = static_cast<float>(msg->linear.x);
    nav_packet_.linear_y = static_cast<float>(-msg->linear.y);
    nav_packet_.angular_z = static_cast<float>(msg->angular.z);
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from nav callback: %s", e.what());
  }
}

void DriverNode::armCommandCallback(sensor_msgs::msg::JointState::ConstSharedPtr msg)
{
  if (!arm_serial_enabled_)
  {
    return;
  }

  try
  {
    std::scoped_lock lock{arm_mutex_};
    for (std::size_t i = 0; i < kArmJointCount; ++i)
    {
      const auto it = std::find(msg->name.begin(), msg->name.end(), arm_joint_names_[i]);
      if (it == msg->name.end())
      {
        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          2000,
          "Arm command missing joint [%s]",
          arm_joint_names_[i].c_str());
        continue;
      }

      const auto idx = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (idx < msg->position.size() && std::isfinite(msg->position[idx]))
      {
        arm_target_position_[i] = static_cast<float>(msg->position[idx]);
      }
    }
    have_arm_command_ = true;
    if (!logged_first_arm_command_)
    {
      logged_first_arm_command_ = true;
      RCLCPP_INFO(
        get_logger(),
        "Received first arm command on [%s]; serial packets will set arm_command_enabled=1.",
        arm_command_sub_->get_topic_name());
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR(get_logger(), "Error from arm command callback: %s", e.what());
  }
}

void DriverNode::gripperCommandCallback(std_msgs::msg::Int8::ConstSharedPtr msg)
{
  std::scoped_lock lock{arm_mutex_};
  if (msg->data != 1 && msg->data != -1)
  {
    RCLCPP_WARN(get_logger(), "Ignoring invalid gripper command: %d", static_cast<int>(msg->data));
    return;
  }
  gripper_command_ = msg->data;
  RCLCPP_INFO(
    get_logger(),
    "Gripper serial command updated: %d (%s)",
    static_cast<int>(gripper_command_),
    gripper_command_ == 1 ? "closed" : "open");
}

void DriverNode::timerCallback()
{
  try
  {
    SendPacket send_packet;
    {
      std::scoped_lock lock{nav_mutex_};
      send_packet.nav = nav_packet_;
    }
    {
      std::scoped_lock lock{arm_mutex_};
      const auto now = std::chrono::steady_clock::now();
      double dt_sec = 1.0 / arm_command_publish_hz_;
      if (last_send_time_ != std::chrono::steady_clock::time_point{})
      {
        dt_sec = std::max(1.0e-3, std::chrono::duration<double>(now - last_send_time_).count());
      }
      last_send_time_ = now;
      updateArmCommandLocked(dt_sec);
      send_packet.arm_command_enabled = (arm_serial_enabled_ && have_arm_command_) ? 1 : 0;
      send_packet.arm_sequence = arm_sequence_++;
      send_packet.arm = arm_packet_;
      send_packet.gripper_command = gripper_command_;
      const bool should_log_gripper_command =
        gripper_command_log_interval_sec_ <= 0.0 ||
        last_gripper_command_log_time_ == std::chrono::steady_clock::time_point{} ||
        std::chrono::duration<double>(now - last_gripper_command_log_time_).count() >=
        gripper_command_log_interval_sec_;
      if (should_log_gripper_command)
      {
        last_gripper_command_log_time_ = now;
        RCLCPP_INFO(
          get_logger(),
          "Sending gripper_command to lower controller: %d (%s), arm_sequence=%u, log_interval=%.2f s.",
          static_cast<int>(send_packet.gripper_command),
          send_packet.gripper_command == 1 ? "closed" : "open",
          send_packet.arm_sequence,
          gripper_command_log_interval_sec_);
      }
      if (send_packet.arm_command_enabled != 0 && !logged_first_enabled_serial_packet_)
      {
        logged_first_enabled_serial_packet_ = true;
        RCLCPP_INFO(
          get_logger(),
          "Sending enabled arm serial packets: sequence=%u joints=[%.4f %.4f %.4f %.4f %.4f %.4f] gripper=%d packet_size=%zu.",
          send_packet.arm_sequence,
          send_packet.arm.position[0],
          send_packet.arm.position[1],
          send_packet.arm.position[2],
          send_packet.arm.position[3],
          send_packet.arm.position[4],
          send_packet.arm.position[5],
          static_cast<int>(send_packet.gripper_command),
          SendPacketSize(include_arm_extension_ && arm_serial_enabled_));
      }
    }
    {
      std::scoped_lock lock{serial_port_mutex_};
      if (!serial_driver_.port()->is_open())
      {
        throw std::runtime_error("serial port is closed");
      }
      serial_driver_.port()->send(
        ToVector(send_packet, include_arm_extension_ && arm_serial_enabled_));
    }
  }
  catch (const std::exception& e)
  {
    RCLCPP_ERROR_THROTTLE(
      get_logger(), *get_clock(), 2000,
      "Serial send unavailable; waiting for one reconnect attempt: %s", e.what());
    reopenPort();
  }
}

void DriverNode::startSerialReceive()
{
  if (!serial_receive_enabled_.load() || !rclcpp::ok())
  {
    return;
  }

  using namespace std::placeholders;
  std::scoped_lock lock{serial_port_mutex_};
  if (!serial_receive_enabled_.load() || !serial_driver_.port()->is_open())
  {
    return;
  }
  serial_driver_.port()->async_receive(std::bind(&DriverNode::serialReceiveCallback, this, _1, _2));
}

void DriverNode::serialReceiveCallback(const std::vector<uint8_t>& data, std::size_t size)
{
  if (size < kLegacyReceiveFrameSize)
  {
    RCLCPP_WARN(get_logger(), "Serial receive incomplete");
    startSerialReceive();
    return;
  }

  if (data.front() != kReceiveHeader)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid header: %02X",
      data.front());
    startSerialReceive();
    return;
  }

  const std::size_t arm_offset = ReceiveArmExtensionOffset(data, size);
  if (arm_offset == 0)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid receive frame: size=%zu byte1=0x%02X byte2=0x%02X",
      size,
      size > 1 ? data[1] : 0,
      size > 2 ? data[2] : 0);
    startSerialReceive();
    return;
  }

  const ReceivePacket packet = FromVectorWithArmOffset(data, arm_offset);
  if (packet.end_frame != kLegacyEndFrame)
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Invalid end frame: %02X",
      packet.end_frame);
    startSerialReceive();
    return;
  }

  if (HasValidArmReceiveExtension(packet, size, arm_offset))
  {
    publishArmState(packet);
  }
  else
  {
    RCLCPP_WARN_THROTTLE(
      get_logger(),
      *get_clock(),
      2000,
      "Arm feedback extension missing or invalid: size=%zu expected>=%zu arm_offset=%zu arm_header=0x%02X valid=%u reserved=0x%02X arm_end=0x%02X",
      size,
      ReceivePacketSize(arm_offset),
      arm_offset,
      packet.arm_header,
      static_cast<unsigned int>(packet.arm_state_valid),
      static_cast<unsigned int>(packet.arm_feedback_reserved),
      packet.arm_end_frame);
  }
  startSerialReceive();
}

void DriverNode::publishArmState(const ReceivePacket& packet)
{
  RCLCPP_INFO_THROTTLE(
    get_logger(),
    *get_clock(),
    5000,
    "Raw lower-arm feedback before angle conversion: sequence=%u unit=%s "
    "J1=%.6f J2=%.6f J3=%.6f J4=%.6f J5=%.6f J6=%.6f",
    static_cast<unsigned int>(packet.arm_sequence),
    arm_serial_angles_in_degrees_ ? "deg" : "rad",
    static_cast<double>(packet.arm.position[0]),
    static_cast<double>(packet.arm.position[1]),
    static_cast<double>(packet.arm.position[2]),
    static_cast<double>(packet.arm.position[3]),
    static_cast<double>(packet.arm.position[4]),
    static_cast<double>(packet.arm.position[5]));

  sensor_msgs::msg::JointState msg;
  msg.header.stamp = this->now();
  msg.name = arm_joint_names_;
  msg.position.resize(kArmJointCount);
  {
    std::scoped_lock lock{arm_mutex_};
    for (std::size_t i = 0; i < kArmJointCount; ++i)
    {
      const float measured = angleFromSerialUnit(packet.arm.position[i], i);
      msg.position[i] = measured;
      if (!have_arm_command_)
      {
        arm_target_position_[i] = measured;
        arm_command_position_rad_[i] = measured;
        arm_packet_.position[i] = angleToSerialUnit(measured, i);
      }
    }
  }
  if (arm_state_pub_)
  {
    arm_state_pub_->publish(msg);
  }
}

drivers::serial_driver::SerialPortConfig DriverNode::getDeviceConfig()
{
  using namespace drivers::serial_driver;

  static const std::unordered_map<std::string, FlowControl> fc_map{
    {"none", FlowControl::NONE}, {"hardware", FlowControl::HARDWARE}, {"software", FlowControl::SOFTWARE}};

  static const std::unordered_map<std::string, Parity> pt_map{
    {"none", Parity::NONE}, {"odd", Parity::ODD}, {"even", Parity::EVEN}};

  static const std::unordered_map<std::string, StopBits> sb_map{
    {"1", StopBits::ONE}, {"1.0", StopBits::ONE}, {"1.5", StopBits::ONE_POINT_FIVE}, {"2", StopBits::TWO}, {"2.0", StopBits::TWO}};

  return {
    static_cast<uint32_t>(declare_parameter<int>("baud_rate")),
    fc_map.at(declare_parameter<std::string>("flow_control")),
    pt_map.at(declare_parameter<std::string>("parity")),
    sb_map.at(declare_parameter<std::string>("stop_bits"))};
}

void DriverNode::reopenPort()
{
  if (serial_reopen_in_progress_.exchange(true))
  {
    return;
  }

  serial_receive_enabled_.store(false);
  while (rclcpp::ok())
  {
    try
    {
      {
        std::scoped_lock lock{serial_port_mutex_};
        if (serial_driver_.port()->is_open())
        {
          serial_driver_.port()->close();
        }
        serial_driver_.port()->open();
      }

      serial_receive_enabled_.store(true);
      startSerialReceive();
      serial_reopen_in_progress_.store(false);
      RCLCPP_INFO(get_logger(), "Serial port reopened; receive loop resumed.");
      return;
    }
    catch (const std::exception& ex)
    {
      RCLCPP_ERROR_THROTTLE(
        get_logger(), *get_clock(), 2000,
        "Serial port is unavailable; retrying reconnect: %s", ex.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
  serial_reopen_in_progress_.store(false);
}

void DriverNode::updateArmCommandLocked(double dt_sec)
{
  const double scaled_dt = std::max(1.0e-3, dt_sec) * arm_command_speed_scale_;
  for (std::size_t i = 0; i < kArmJointCount; ++i)
  {
    const float target = arm_target_position_[i];
    const float previous = arm_command_position_rad_[i];
    const float error = target - previous;
    const float abs_error = std::abs(error);
    if (!have_arm_command_ || abs_error <= static_cast<float>(arm_command_goal_tolerance_rad_))
    {
      arm_command_position_rad_[i] = have_arm_command_ ? target : previous;
      arm_packet_.position[i] = angleToSerialUnit(arm_command_position_rad_[i], i);
      continue;
    }

    const float max_step =
      max_arm_command_velocity_rad_s_[i] <= 0.0F ?
      abs_error :
      max_arm_command_velocity_rad_s_[i] * static_cast<float>(scaled_dt);
    const float step = std::clamp(error, -max_step, max_step);
    arm_command_position_rad_[i] = previous + step;
    arm_packet_.position[i] = angleToSerialUnit(arm_command_position_rad_[i], i);
  }
}

float DriverNode::angleToSerialUnit(float angle_rad, std::size_t joint_index) const
{
  // Calibration offsets are expressed in radians.  ROS JointState values are
  // already radians; serialAngleRadToUnit() converts the calibrated value to
  // the wire unit (radians or degrees) after this joint-specific mapping.
  float serial_angle_rad = angle_rad;
  switch (joint_index)
  {
    case 0:
      // J1: serial = ROS - 0.01965
      serial_angle_rad = angle_rad - 0.22473F;
      break;
    case 1:
      // J2: serial = 1.55281 - ROS, with reversed rotation direction.
      serial_angle_rad = - 3.46819F + angle_rad;
      break;
    case 2:
      // J3: serial = ROS + 1.60589
      serial_angle_rad = - angle_rad - 1.62393F;
      break;
    case 3:
      // J4: serial = ROS + 3.12144
      serial_angle_rad = angle_rad - 2.7583F;
      break;
    case 4:
      // J5: serial = ROS + 3.15340
      serial_angle_rad = angle_rad - 0.688806F;
      break;
    case 5:
      // J6: serial = ROS - 1.40514
      serial_angle_rad = angle_rad;
      break;
    case 6:
      // J7 calibration retained for a future 7-joint packet.  The current
      // serial protocol carries kArmJointCount == 6 joints, so this case is
      // not reached by the current driver.
      serial_angle_rad = angle_rad - 0.19149F;
      break;
    default:
      serial_angle_rad = angle_rad;
      break;
  }
  return serialAngleRadToUnit(serial_angle_rad);
}

float DriverNode::angleFromSerialUnit(float serial_angle, std::size_t joint_index) const
{
  const float serial_angle_rad = serialAngleRadFromUnit(serial_angle);
  switch (joint_index)
  {
    case 0:
      // Inverse of J1: ROS = serial + 0.01965
      return serial_angle_rad + 0.22473F;
    case 1:
      // Inverse of J2: ROS = 1.55281 - serial
      return  3.46819F + serial_angle_rad;
    case 2:
      // Inverse of J3: ROS = serial - 1.60589
      return  - serial_angle_rad - 1.62393F;
    case 3:
      // Inverse of J4: ROS = serial - 3.12144
      return serial_angle_rad + 2.7583F;
    case 4:
      // Inverse of J5: ROS = serial + 0.7566
      return serial_angle_rad + 0.688806F;
    case 5:
      // Inverse of J6: ROS = serial + 1.40514
      return serial_angle_rad;
    case 6:
      // Inverse of the retained J7 calibration above.
      return serial_angle_rad + 0.19149F;
    default:
      return serial_angle_rad;
  }
}

float DriverNode::serialAngleRadToUnit(float serial_angle_rad) const
{
  if (!arm_serial_angles_in_degrees_)
  {
    return serial_angle_rad;
  }
  return serial_angle_rad * static_cast<float>(180.0 / kPi);
}

float DriverNode::serialAngleRadFromUnit(float serial_angle) const
{
  if (!arm_serial_angles_in_degrees_)
  {
    return serial_angle;
  }
  return serial_angle * static_cast<float>(kPi / 180.0);
}
}  // namespace rm::serial

#include <rclcpp_components/register_node_macro.hpp>

RCLCPP_COMPONENTS_REGISTER_NODE(rm::serial::DriverNode)
