#include "lite_motion_planner/robot_model.hpp"

#include <lite_motion_msgs/srv/add_scene_object.hpp>
#include <lite_motion_msgs/srv/attach_scene_object.hpp>
#include <lite_motion_msgs/srv/plan_arm_motion.hpp>
#include <lite_motion_msgs/srv/remove_scene_object.hpp>
#include <lite_motion_msgs/srv/set_scene_collision_allowance.hpp>

#include <arm2nav_msgs/msg/arm2_nav.hpp>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <std_msgs/msg/int8.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <Eigen/Geometry>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/kinematics.hpp>

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <future>
#include <limits>
#include <mutex>
#include <memory>
#include <stdexcept>
#include <string>
#include <sstream>
#include <thread>
#include <vector>

namespace lite_motion_planner {
namespace {

constexpr double kPi = 3.14159265358979323846;

std::string normalizeAngleUnit(std::string unit) {
  std::transform(unit.begin(), unit.end(), unit.begin(), [](unsigned char c) {
    return static_cast<char>(std::tolower(c));
  });
  return unit;
}

bool isDegreeUnit(const std::string& unit) {
  return unit == "deg" || unit == "degree" || unit == "degrees";
}

bool isRadianUnit(const std::string& unit) {
  return unit == "rad" || unit == "radian" || unit == "radians";
}

void degreesToRadiansInPlace(std::vector<double>& values) {
  for (double& value : values) {
    value *= kPi / 180.0;
  }
}

geometry_msgs::msg::Quaternion quaternionMsgFromRpy(double roll, double pitch, double yaw) {
  const Eigen::AngleAxisd rx(roll, Eigen::Vector3d::UnitX());
  const Eigen::AngleAxisd ry(pitch, Eigen::Vector3d::UnitY());
  const Eigen::AngleAxisd rz(yaw, Eigen::Vector3d::UnitZ());
  const Eigen::Quaterniond q = rz * ry * rx;

  geometry_msgs::msg::Quaternion out;
  out.x = q.x();
  out.y = q.y();
  out.z = q.z();
  out.w = q.w();
  return out;
}

Eigen::Quaterniond quaternionEigenFromMsg(const geometry_msgs::msg::Quaternion& q_msg) {
  Eigen::Quaterniond q(q_msg.w, q_msg.x, q_msg.y, q_msg.z);
  if (q.norm() < 1e-12) {
    q = Eigen::Quaterniond::Identity();
  }
  q.normalize();
  return q;
}

geometry_msgs::msg::Pose poseMsgFromSE3(const pinocchio::SE3& tf) {
  geometry_msgs::msg::Pose out;
  out.position.x = tf.translation().x();
  out.position.y = tf.translation().y();
  out.position.z = tf.translation().z();
  const Eigen::Quaterniond q(tf.rotation());
  out.orientation.x = q.x();
  out.orientation.y = q.y();
  out.orientation.z = q.z();
  out.orientation.w = q.w();
  return out;
}

pinocchio::SE3 poseMsgToSE3(const geometry_msgs::msg::Pose& pose) {
  return pinocchio::SE3(
    quaternionEigenFromMsg(pose.orientation).toRotationMatrix(),
    Eigen::Vector3d(pose.position.x, pose.position.y, pose.position.z));
}

double trajectoryDurationSec(const trajectory_msgs::msg::JointTrajectory& traj) {
  if (traj.points.empty()) {
    return 0.0;
  }
  const auto& last = traj.points.back().time_from_start;
  return static_cast<double>(last.sec) + static_cast<double>(last.nanosec) * 1e-9;
}

builtin_interfaces::msg::Duration durationFromSec(double seconds) {
  builtin_interfaces::msg::Duration out;
  seconds = std::max(0.0, seconds);
  out.sec = static_cast<int32_t>(std::floor(seconds));
  out.nanosec = static_cast<uint32_t>(
    std::llround((seconds - static_cast<double>(out.sec)) * 1e9));
  if (out.nanosec >= 1000000000u) {
    out.sec += 1;
    out.nanosec -= 1000000000u;
  }
  return out;
}

}  // namespace

class TzbCatchNode : public rclcpp::Node {
public:
  TzbCatchNode()
  : Node("tzb_catch") {
    const auto urdf_path = this->declare_parameter<std::string>("urdf_path", "");
    const auto srdf_path = this->declare_parameter<std::string>("srdf_path", "");
    const auto mesh_package_dirs = this->declare_parameter<std::vector<std::string>>(
      "mesh_package_dirs", std::vector<std::string>());

    planning_frame_ = this->declare_parameter<std::string>("planning_frame", "base_link");
    arm_group_ = this->declare_parameter<std::string>("tzb.arm_group", "tzb_group");
    tcp_frame_ = this->declare_parameter<std::string>("tzb.tcp_frame", "tcp_link");
    e3_object_id_ = this->declare_parameter<std::string>("tzb.e3_object_id", "E3");
    e3_mesh_resource_ = this->declare_parameter<std::string>(
      "tzb.e3_mesh_resource",
      "package://lite_motion_planner/config/E3.STL");
    mesh_scale_ = this->declare_parameter<std::vector<double>>(
      "tzb.mesh_scale", std::vector<double>{0.001, 0.001, 0.001});
    if (!mesh_scale_.empty() && mesh_scale_.size() != 3) {
      throw std::runtime_error("tzb.mesh_scale must contain exactly 3 values");
    }

    e3_target_frame_ = this->declare_parameter<std::string>("tzb.e3_target_frame", "base_link");
    e3_target_xyz_ = declareVec3Param("tzb.e3_target_xyz", {0.35, 0.20, 0.20});
    e3_target_rpy_ = declareVec3Param("tzb.e3_target_rpy", {0.0, 0.0, 0.0});
    use_yaml_e3_target_ = this->declare_parameter<bool>("tzb.use_yaml_e3_target", true);
    force_target_roll_ = this->declare_parameter<bool>("tzb.force_target_roll", false);
    force_target_pitch_ = this->declare_parameter<bool>("tzb.force_target_pitch", false);
    force_target_yaw_ = this->declare_parameter<bool>("tzb.force_target_yaw", false);
    adaptive_rpy_ = this->declare_parameter<bool>("tzb.adaptive_rpy", false);
    position_only_target_ = this->declare_parameter<bool>("tzb.position_only_target", false);
    const bool any_forced_target_rpy =
      force_target_roll_ || force_target_pitch_ || force_target_yaw_;
    if (any_forced_target_rpy && (adaptive_rpy_ || position_only_target_)) {
      throw std::runtime_error(
        "tzb.force_target_roll/pitch/yaw cannot be combined with tzb.adaptive_rpy or "
        "tzb.position_only_target");
    }
    if (adaptive_rpy_ && position_only_target_) {
      throw std::runtime_error(
        "tzb.adaptive_rpy and tzb.position_only_target are mutually exclusive");
    }
    adaptive_rpy_step_rad_ = this->declare_parameter<double>(
      "tzb.adaptive_rpy_step_deg", 15.0) * kPi / 180.0;
    adaptive_rpy_max_rad_ = this->declare_parameter<double>(
      "tzb.adaptive_rpy_max_deg", 45.0) * kPi / 180.0;
    adaptive_rpy_max_candidates_ = this->declare_parameter<int>(
      "tzb.adaptive_rpy_max_candidates", 64);
    if (adaptive_rpy_step_rad_ <= 0.0 || adaptive_rpy_max_rad_ < 0.0 ||
        adaptive_rpy_max_candidates_ <= 0) {
      throw std::runtime_error(
        "adaptive RPY parameters must have positive step/candidate values and nonnegative max angle");
    }
    target_rpy_unit_ = normalizeAngleUnit(
      this->declare_parameter<std::string>("tzb.target_rpy_unit", "rad"));
    if (!isRadianUnit(target_rpy_unit_) && !isDegreeUnit(target_rpy_unit_)) {
      throw std::runtime_error("tzb.target_rpy_unit must be 'rad' or 'deg'");
    }
    forced_target_roll_ = this->declare_parameter<double>("tzb.forced_target_roll", 0.0);
    forced_target_pitch_ = this->declare_parameter<double>("tzb.forced_target_pitch", 0.0);
    forced_target_yaw_ = this->declare_parameter<double>("tzb.forced_target_yaw", 0.0);
    if (isDegreeUnit(target_rpy_unit_)) {
      forced_target_roll_ *= kPi / 180.0;
      forced_target_pitch_ *= kPi / 180.0;
      forced_target_yaw_ *= kPi / 180.0;
    }
    pose_estimator_target_topic_ = this->declare_parameter<std::string>(
      "tzb.pose_estimator_target_topic", "/pose_estimator/object_pose");
    pose_estimator_wait_timeout_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.pose_estimator_wait_timeout_sec", 3.0));
    pose_estimator_continuous_required_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.pose_estimator_continuous_required_sec", 2.0));
    pose_estimator_continuous_max_gap_sec_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.pose_estimator_continuous_max_gap_sec", 0.75));
    pose_estimator_reference_frame_ = this->declare_parameter<std::string>(
      "tzb.pose_estimator_reference_frame", "");
    pose_estimator_reference_transform_ = declareMat4Param(
      "tzb.pose_estimator_reference_transform_matrix",
      {1.0, 0.0, 0.0, 0.0,
       0.0, 1.0, 0.0, 0.0,
       0.0, 0.0, 1.0, 0.0,
       0.0, 0.0, 0.0, 1.0});
    freeze_camera_pointcloud_enabled_ =
      this->declare_parameter<bool>("tzb.freeze_camera_pointcloud_enabled", true);
    camera_pointcloud_topic_ = this->declare_parameter<std::string>(
      "tzb.camera_pointcloud_topic", "/camera/depth/color/points");
    frozen_camera_pointcloud_topic_ = this->declare_parameter<std::string>(
      "tzb.frozen_camera_pointcloud_topic", "/tzb_catch/frozen_observe_camera_points");
    frozen_camera_frame_id_ = this->declare_parameter<std::string>(
      "tzb.frozen_camera_frame_id", "tzb_observe_camera_link");
    apply_reference_transform_to_frozen_frame_ =
      this->declare_parameter<bool>("tzb.apply_reference_transform_to_frozen_frame", true);
    transform_frozen_pointcloud_to_reference_frame_ =
      this->declare_parameter<bool>("tzb.transform_frozen_pointcloud_to_reference_frame", true);
    frozen_camera_pointcloud_log_throttle_sec_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.frozen_camera_pointcloud_log_throttle_sec", 2.0));
    pose_estimator_axis_conversion_ = declareMat3Param(
      "tzb.pose_estimator_axis_conversion_matrix",
      {1.0, 0.0, 0.0,
       0.0, 1.0, 0.0,
       0.0, 0.0, 1.0});
    gripper_joint_names_ = this->declare_parameter<std::vector<std::string>>(
      "tzb.gripper_joint_names",
      std::vector<std::string>{"Claw_left_joint"});
    gripper_home_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_home_positions", std::vector<double>{0.0});
    gripper_open_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_open_positions", std::vector<double>{-0.5236});
    gripper_close_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.gripper_close_positions", std::vector<double>{0.0});
    gripper_command_topic_ = this->declare_parameter<std::string>(
      "tzb.gripper_command_topic", "/tzb_catch/gripper_command");
    grasp_collision_links_ = this->declare_parameter<std::vector<std::string>>(
      "tzb.grasp_collision_links",
      std::vector<std::string>{"Claw_left_link", "claw_right_link"});
    grasp_collision_allowed_penetration_m_ = this->declare_parameter<double>(
      "tzb.grasp_collision_allowed_penetration", -1.0);
    e3_collision_object_enabled_ = this->declare_parameter<bool>(
      "tzb.e3_collision_object_enabled", true);

    joint_planning_timeout_ = std::max(
      0.05,
      this->declare_parameter<double>("tzb.joint_planning_timeout", 2.0));
    cartesian_planning_timeout_ = std::max(
      0.05,
      this->declare_parameter<double>("tzb.cartesian_planning_timeout", 2.0));
    path_check_step_m_ = std::max(
      0.001,
      this->declare_parameter<double>("tzb.path_check_step", 0.05));
    cartesian_max_step_m_ = std::max(
      1e-4,
      this->declare_parameter<double>("tzb.cartesian_max_step_m", 0.02));
    cartesian_min_interpolation_steps_ = std::max<int>(
      1,
      this->declare_parameter<int>("tzb.cartesian_min_interpolation_steps", 2));
    cartesian_refinement_max_steps_ = std::max<int>(
      cartesian_min_interpolation_steps_,
      this->declare_parameter<int>("tzb.cartesian_refinement_max_steps", 16));
    cartesian_use_request_solver_settings_ =
      this->declare_parameter<bool>("tzb.cartesian_use_request_solver_settings", true);
    cartesian_use_sdls_ = this->declare_parameter<bool>("tzb.cartesian_use_sdls", true);
    cartesian_ik_max_iterations_ = std::max<int>(
      50,
      this->declare_parameter<int>("tzb.cartesian_ik_max_iterations", 800));
    cartesian_ik_pos_tolerance_ = std::max(
      1e-5,
      this->declare_parameter<double>("tzb.cartesian_ik_pos_tolerance", 0.002));
    cartesian_ik_rot_tolerance_ = std::max(
      1e-5,
      this->declare_parameter<double>("tzb.cartesian_ik_rot_tolerance", 0.05));
    cartesian_ik_damping_ = std::max(
      1e-12,
      this->declare_parameter<double>("tzb.cartesian_ik_damping", 1.0e-3));
    cartesian_ik_alpha_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.cartesian_ik_alpha", 0.55));
    cartesian_ik_max_step_norm_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.cartesian_ik_max_step_norm", 0.25));
    cartesian_ik_max_seed_count_ = std::max<int>(
      1,
      this->declare_parameter<int>("tzb.cartesian_ik_max_seed_count", 12));
    cartesian_nullspace_gain_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.cartesian_nullspace_gain", 0.0));
    cartesian_singular_threshold_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_singular_threshold", 0.03));
    cartesian_branch_jump_max_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_branch_jump_max_rad", 0.75));
    cartesian_branch_jump_norm_max_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_branch_jump_norm_max", 1.20));
    cartesian_waypoint_pos_tolerance_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_waypoint_pos_tolerance", 0.002));
    cartesian_waypoint_rot_tolerance_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.cartesian_waypoint_rot_tolerance", 0.05));

    post_action_wait_sec_ = this->declare_parameter<double>("tzb.post_action_wait_sec", 0.05);
    service_wait_sec_ = this->declare_parameter<double>("tzb.service_wait_sec", 30.0);
    const double configured_response_wait_sec =
      this->declare_parameter<double>("tzb.response_wait_sec", 30.0);
    response_wait_sec_ = std::max(
      configured_response_wait_sec,
      std::max(joint_planning_timeout_, cartesian_planning_timeout_) + 1.0);
    if (response_wait_sec_ > configured_response_wait_sec) {
      RCLCPP_WARN(
        this->get_logger(),
        "tzb.response_wait_sec %.2f is shorter than the planner timeout; using %.2f s.",
        configured_response_wait_sec,
        response_wait_sec_);
    }
    e3_marker_topic_ = this->declare_parameter<std::string>("tzb.e3_marker_topic", "/tzb_catch/e3_marker");
    publish_e3_marker_ = this->declare_parameter<bool>("tzb.publish_e3_marker", true);
    reference_marker_republish_hz_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.reference_marker_republish_hz", 5.0));
    publish_tcp_trajectory_ =
      this->declare_parameter<bool>("tzb.publish_tcp_trajectory", true);
    tcp_trajectory_line_width_m_ = std::max(
      1.0e-4,
      this->declare_parameter<double>("tzb.tcp_trajectory_line_width_m", 0.006));
    tcp_trajectory_min_distance_m_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.tcp_trajectory_min_distance_m", 0.002));
    tcp_trajectory_max_points_ = static_cast<size_t>(std::max<int64_t>(
      2,
      this->declare_parameter<int64_t>("tzb.tcp_trajectory_max_points", 10000)));
    sequence_loop_ = this->declare_parameter<bool>("tzb.sequence_loop", false);
    loop_pause_sec_ = std::max(0.0, this->declare_parameter<double>("tzb.loop_pause_sec", 0.5));
    replay_after_complete_ = this->declare_parameter<bool>("tzb.replay_after_complete", true);
    replay_loop_ = this->declare_parameter<bool>("tzb.replay_loop", true);
    replay_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.replay_speed_scale", 0.35));
    replay_segment_pause_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.replay_segment_pause_sec", 0.15));
    sentry_mode_ = this->declare_parameter<bool>("tzb.sentry_mode", false);
    sentry_execution_limit_ = std::max<int>(
      1, this->declare_parameter<int>("tzb.sentry_execution_limit", 1));
    arm2nav_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm2nav_state_topic", "/arm2nav");
    arm2nav_arm_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm2nav_arm_state_topic", "/arm2nav/from_arm");
    sentry_trigger_pulse_sec_ = std::max(
      0.1, this->declare_parameter<double>("tzb.sentry_trigger_pulse_sec", 0.2));
    joint_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.joint_goal_tolerance_rad", 0.01));
    arm_command_enabled_ = this->declare_parameter<bool>("tzb.arm_command_enabled", true);
    arm_command_publish_hz_ = std::max(
      1.0,
      this->declare_parameter<double>("tzb.arm_command_publish_hz", 100.0));
    arm_command_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("tzb.arm_command_speed_scale", 1.0));
    arm_command_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.arm_command_goal_tolerance_rad", 0.005));
    arm_command_publish_during_replay_ =
      this->declare_parameter<bool>("tzb.arm_command_publish_during_replay", false);

    enable_visualization_ = this->declare_parameter<bool>("enable_visualization", true);
    visualization_playback_enabled_ =
      enable_visualization_ && this->declare_parameter<bool>("enable_joint_state_playback", true);
    visualization_playback_speed_scale_ = std::max(
      1e-3,
      this->declare_parameter<double>("joint_state_playback_speed_scale", 1.0));

    robot_model_ = std::make_unique<RobotModel>(urdf_path, srdf_path, mesh_package_dirs);
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);

    joint_positions_unit_ =
      normalizeAngleUnit(this->declare_parameter<std::string>("tzb.joint_positions_unit", "rad"));
    if (!isRadianUnit(joint_positions_unit_) && !isDegreeUnit(joint_positions_unit_)) {
      throw std::runtime_error("tzb.joint_positions_unit must be 'rad' or 'deg'");
    }
    initial_arm_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.ini_joint_positions", std::vector<double>(arm_group.joint_names.size(), 0.0));
    place_arm_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.place_joint_positions", initial_arm_positions_);
    observe_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.observe_joint_positions",
      std::vector<double>{0.0, 0.0, 1.7, 0.0, 0.0, 0.0});
    sentry_joint_positions_ = this->declare_parameter<std::vector<double>>(
      "tzb.sentry_joint_positions", observe_joint_positions_);
    max_arm_command_velocity_rad_s_ = this->declare_parameter<std::vector<double>>(
      "tzb.max_arm_command_velocity_rad_s",
      std::vector<double>{1.2, 1.2, 1.2, 1.8, 1.8, 2.2});
    arm_state_topic_ = this->declare_parameter<std::string>(
      "tzb.arm_state_topic", "/serial/arm_joint_state");
    arm_feedback_enabled_ = this->declare_parameter<bool>(
      "tzb.arm_feedback_enabled", true);
    arm_state_wait_timeout_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_state_wait_timeout_sec", 2.0));
    arm_state_max_age_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_state_max_age_sec", 0.5));
    arm_motion_feedback_timeout_sec_ = std::max(
      0.1,
      this->declare_parameter<double>("tzb.arm_motion_feedback_timeout_sec", 30.0));
    arm_motion_goal_tolerance_rad_ = std::max(
      1e-6,
      this->declare_parameter<double>("tzb.arm_motion_goal_tolerance_rad", 0.03));
    arm_motion_stable_duration_sec_ = std::max(
      0.0,
      this->declare_parameter<double>("tzb.arm_motion_stable_duration_sec", 0.25));
    require_arm_feedback_for_completion_ = this->declare_parameter<bool>(
      "tzb.require_arm_feedback_for_completion", true);

    if (initial_arm_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.ini_joint_positions size does not match tzb arm group dof");
    }
    if (place_arm_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.place_joint_positions size does not match tzb arm group dof");
    }
    if (observe_joint_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.observe_joint_positions size does not match tzb arm group dof");
    }
    if (sentry_joint_positions_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.sentry_joint_positions size does not match tzb arm group dof");
    }
    if (isDegreeUnit(joint_positions_unit_)) {
      degreesToRadiansInPlace(initial_arm_positions_);
      degreesToRadiansInPlace(place_arm_positions_);
      degreesToRadiansInPlace(observe_joint_positions_);
      degreesToRadiansInPlace(sentry_joint_positions_);
    }
    if (max_arm_command_velocity_rad_s_.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("tzb.max_arm_command_velocity_rad_s size does not match tzb arm group dof");
    }
    if (gripper_home_positions_.size() != gripper_joint_names_.size() ||
        gripper_open_positions_.size() != gripper_joint_names_.size() ||
        gripper_close_positions_.size() != gripper_joint_names_.size()) {
      throw std::runtime_error("tzb gripper position arrays must match tzb.gripper_joint_names size");
    }

    arm_state_ = initial_arm_positions_;
    gripper_state_ = gripper_open_positions_;

    task_autostart_ = this->declare_parameter<bool>("tzb.autostart", false);
    listen_navigation_goal_status_ = this->declare_parameter<bool>(
      "tzb.listen_navigation_goal_status", true);
    task_start_topic_ = this->declare_parameter<std::string>(
      "tzb.start_topic", "/tzb_catch/start");
    navigation_goal_status_topic_ = this->declare_parameter<std::string>(
      "tzb.navigation_goal_status_topic", "/navigation_goal_status");
    task_result_topic_ = this->declare_parameter<std::string>(
      "tzb.result_topic", "/tzb_catch/result");
    task_start_gate_enabled_ = !task_autostart_ && listen_navigation_goal_status_ && !sentry_mode_;

    plan_client_ = this->create_client<lite_motion_msgs::srv::PlanArmMotion>("plan_arm_motion");
    add_object_client_ = this->create_client<lite_motion_msgs::srv::AddSceneObject>("add_scene_object");
    attach_object_client_ = this->create_client<lite_motion_msgs::srv::AttachSceneObject>("attach_scene_object");
    remove_object_client_ = this->create_client<lite_motion_msgs::srv::RemoveSceneObject>("remove_scene_object");
    set_scene_collision_allowance_client_ =
      this->create_client<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
        "set_scene_collision_allowance");
    trajectory_pub_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(
      this->declare_parameter<std::string>(
        "joint_trajectory_topic", "/joint_trajectory_controller/joint_trajectory"),
      10);
    gripper_command_pub_ = this->create_publisher<std_msgs::msg::Int8>(
      gripper_command_topic_, rclcpp::QoS(1).transient_local().reliable());
    joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
      this->declare_parameter<std::string>("visualization_joint_states_topic", "/joint_states"),
      rclcpp::QoS(10));
    if (arm_command_enabled_) {
      arm_command_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
        this->declare_parameter<std::string>(
          "tzb.arm_command_topic", "/tzb_catch/arm_joint_command"),
        rclcpp::SensorDataQoS());
    }
    if (arm_feedback_enabled_) {
      arm_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        arm_state_topic_,
        rclcpp::SensorDataQoS(),
        [this](sensor_msgs::msg::JointState::SharedPtr msg) {
          onArmState(*msg);
        });
    }
    if (task_start_gate_enabled_) {
      task_result_pub_ = this->create_publisher<std_msgs::msg::Int8>(
        task_result_topic_, rclcpp::QoS(10).reliable());
      task_start_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        task_start_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          if (msg->data != 1) {
            return;
          }
          {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            task_trigger_pending_ = true;
          }
          task_trigger_cv_.notify_all();
        });
      navigation_goal_status_sub_ = this->create_subscription<std_msgs::msg::Int8>(
        navigation_goal_status_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const std_msgs::msg::Int8::SharedPtr msg) {
          if (msg->data == 0) {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            navigation_trigger_armed_ = true;
            return;
          }
          if (msg->data != 1) {
            RCLCPP_WARN(
              this->get_logger(),
              "Ignoring unsupported navigation goal status %d on [%s]; expected 0 or 1.",
              static_cast<int>(msg->data),
              navigation_goal_status_topic_.c_str());
            return;
          }
          bool should_notify = false;
          {
            std::lock_guard<std::mutex> lock(task_trigger_mutex_);
            if (navigation_trigger_armed_) {
              task_trigger_pending_ = true;
              navigation_trigger_armed_ = false;
              should_notify = true;
            }
          }
          if (should_notify) {
            RCLCPP_INFO(
              this->get_logger(),
              "Navigation goal status is 1 on [%s]; triggering the arm task.",
              navigation_goal_status_topic_.c_str());
            task_trigger_cv_.notify_all();
          }
        });
    }
    if (sentry_mode_) {
      arm2nav_state_sub_ = this->create_subscription<arm2nav_msgs::msg::Arm2Nav>(
        arm2nav_state_topic_,
        rclcpp::QoS(10).reliable(),
        [this](const arm2nav_msgs::msg::Arm2Nav::SharedPtr msg) {
          onArm2NavState(*msg);
        });
      arm2nav_arm_state_pub_ = this->create_publisher<arm2nav_msgs::msg::Arm2Nav>(
        arm2nav_arm_state_topic_, rclcpp::QoS(10).reliable());
    }
    if (!use_yaml_e3_target_) {
      pose_estimator_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        pose_estimator_target_topic_,
        rclcpp::QoS(10),
        [this](geometry_msgs::msg::PoseStamped::SharedPtr msg) {
          onPoseEstimatorTarget(*msg);
        });
    }
    if (freeze_camera_pointcloud_enabled_) {
      frozen_camera_tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
      rclcpp::QoS frozen_cloud_qos(2);
      frozen_cloud_qos.reliable().durability_volatile();
      frozen_camera_pointcloud_pub_ =
        this->create_publisher<sensor_msgs::msg::PointCloud2>(
          frozen_camera_pointcloud_topic_, frozen_cloud_qos);
      camera_pointcloud_sub_ =
        this->create_subscription<sensor_msgs::msg::PointCloud2>(
          camera_pointcloud_topic_,
          rclcpp::SensorDataQoS(),
          [this](sensor_msgs::msg::PointCloud2::SharedPtr msg) {
            onCameraPointCloud(*msg);
          });
      frozen_camera_tf_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(100),
        [this]() {
          publishFrozenCameraFrame();
        });
    }

    if (enable_visualization_) {
      rclcpp::QoS latched_qos(1);
      latched_qos.transient_local().reliable();
      ref_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "tzb_catch/reference_frames", latched_qos);
      if (publish_e3_marker_) {
        e3_marker_pub_ = this->create_publisher<visualization_msgs::msg::Marker>(
          e3_marker_topic_, latched_qos);
      }
      reference_marker_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
          std::chrono::duration<double>(1.0 / reference_marker_republish_hz_)),
        [this]() {
          publishReferenceMarkers();
          publishE3Marker();
        });
    }

    RCLCPP_INFO(
      this->get_logger(),
      "tzb_catch configured: arm_group=%s tcp_frame=%s target_source=%s continuous_required=%.2fs arm_state=%s initial_wait=%.2fs arrival_wait=%.2fs navigation_status_listener=%s sentry_mode=%s force_rpy=(roll:%s pitch:%s yaw:%s)",
      arm_group_.c_str(),
      tcp_frame_.c_str(),
      use_yaml_e3_target_ ? "yaml" : pose_estimator_target_topic_.c_str(),
      pose_estimator_continuous_required_sec_,
      arm_state_topic_.c_str(),
      arm_state_wait_timeout_sec_,
      arm_motion_feedback_timeout_sec_,
      task_start_gate_enabled_ ? "enabled" : "disabled",
      sentry_mode_ ? "enabled" : "disabled",
      force_target_roll_ ? "true" : "false",
      force_target_pitch_ ? "true" : "false",
      force_target_yaw_ ? "true" : "false");
    logArmJointState("configured_ini_joint_positions", initial_arm_positions_);
    logArmJointState("configured_place_joint_positions", place_arm_positions_);
    logArmJointState("configured_observe_joint_positions", observe_joint_positions_);

    start_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      [this]() {
        start_timer_->cancel();
        publishCurrentTheoreticalJointState("initial_home");
        publishReferenceMarkers("initial");
        publishE3Marker();
        worker_ = std::thread([this]() { runSequence(); });
      });
  }

  ~TzbCatchNode() override {
    stop_requested_.store(true);
    pose_estimator_cv_.notify_all();
    arm_state_cv_.notify_all();
    task_trigger_cv_.notify_all();
    sentry_cv_.notify_all();
    if (worker_.joinable()) {
      worker_.join();
    }
  }

private:
  std::array<double, 3> declareVec3Param(
    const std::string& name,
    const std::array<double, 3>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name, std::vector<double>{default_value[0], default_value[1], default_value[2]});
    if (values.size() != 3) {
      throw std::runtime_error(name + " must contain exactly 3 values");
    }
    return {values[0], values[1], values[2]};
  }

  Eigen::Matrix3d declareMat3Param(
    const std::string& name,
    const std::array<double, 9>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name,
      std::vector<double>(default_value.begin(), default_value.end()));
    if (values.size() != 9) {
      throw std::runtime_error(name + " must contain exactly 9 values");
    }

    Eigen::Matrix3d out;
    out << values[0], values[1], values[2],
           values[3], values[4], values[5],
           values[6], values[7], values[8];
    const double orthogonality_error =
      (out.transpose() * out - Eigen::Matrix3d::Identity()).norm();
    const double det = out.determinant();
    if (orthogonality_error > 1e-3 || std::abs(det - 1.0) > 1e-3) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s is not a proper rotation matrix: det=%.6f orthogonality_error=%.6f",
        name.c_str(),
        det,
        orthogonality_error);
    }
    return out;
  }

  Eigen::Matrix4d declareMat4Param(
    const std::string& name,
    const std::array<double, 16>& default_value) {
    auto values = this->declare_parameter<std::vector<double>>(
      name,
      std::vector<double>(default_value.begin(), default_value.end()));
    if (values.size() != 16) {
      throw std::runtime_error(name + " must contain exactly 16 values");
    }

    Eigen::Matrix4d out;
    out << values[0], values[1], values[2], values[3],
           values[4], values[5], values[6], values[7],
           values[8], values[9], values[10], values[11],
           values[12], values[13], values[14], values[15];
    const Eigen::Matrix3d rot = out.block<3, 3>(0, 0);
    const double orthogonality_error =
      (rot.transpose() * rot - Eigen::Matrix3d::Identity()).norm();
    const double det = rot.determinant();
    const double bottom_row_error =
      (out.row(3) - Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0)).norm();
    if (orthogonality_error > 1e-3 || std::abs(det - 1.0) > 1e-3 ||
        bottom_row_error > 1e-6) {
      RCLCPP_WARN(
        this->get_logger(),
        "%s is not a proper SE3 transform: det=%.6f orthogonality_error=%.6f bottom_row_error=%.6f",
        name.c_str(),
        det,
        orthogonality_error,
        bottom_row_error);
    }
    return out;
  }

  struct ReplaySegment {
    std::string stage_name;
    trajectory_msgs::msg::JointTrajectory trajectory;
    bool e3_attached_during_segment{false};
  };

  geometry_msgs::msg::PoseStamped makePoseStamped(
    const std::array<double, 3>& xyz,
    const std::array<double, 3>& rpy,
    const std::string& frame_id) const {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = frame_id;
    out.pose.position.x = xyz[0];
    out.pose.position.y = xyz[1];
    out.pose.position.z = xyz[2];
    out.pose.orientation = quaternionMsgFromRpy(rpy[0], rpy[1], rpy[2]);
    return out;
  }

  Eigen::VectorXd currentFullState() const {
    Eigen::VectorXd q_full = robot_model_->neutralConfiguration();
    const Eigen::VectorXd q_arm = Eigen::Map<const Eigen::VectorXd>(
      arm_state_.data(), static_cast<Eigen::Index>(arm_state_.size()));
    q_full = robot_model_->groupToFull(arm_group_, q_arm, q_full);

    for (size_t i = 0; i < gripper_joint_names_.size() && i < gripper_state_.size(); ++i) {
      try {
        q_full[robot_model_->qIndexOfJoint(gripper_joint_names_[i])] = gripper_state_[i];
      } catch (const std::exception&) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Gripper joint [%s] is not an active Pinocchio joint; skipping full-state sync.",
          gripper_joint_names_[i].c_str());
      }
    }
    return q_full;
  }

  pinocchio::SE3 currentFramePose(const std::string& frame_name) {
    const Eigen::VectorXd q_full = currentFullState();
    pinocchio::forwardKinematics(robot_model_->model(), robot_model_->data(), q_full);
    pinocchio::updateFramePlacements(robot_model_->model(), robot_model_->data());
    return robot_model_->data().oMf[robot_model_->getFrameIdChecked(frame_name)];
  }

  geometry_msgs::msg::PoseStamped currentFramePoseStamped(const std::string& frame_name) {
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(currentFramePose(frame_name));
    return out;
  }

  geometry_msgs::msg::PoseStamped transformPoseToPlanningFrame(
    const geometry_msgs::msg::PoseStamped& in) {
    if (in.header.frame_id.empty() || in.header.frame_id == planning_frame_) {
      geometry_msgs::msg::PoseStamped out = in;
      out.header.frame_id = planning_frame_;
      return out;
    }

    const pinocchio::SE3 ref_in_planning = referenceFramePoseInPlanningFrame(in.header.frame_id);
    const pinocchio::SE3 pose_in_ref = poseMsgToSE3(in.pose);
    geometry_msgs::msg::PoseStamped out;
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(ref_in_planning * pose_in_ref);
    return out;
  }

  pinocchio::SE3 referenceFramePoseInPlanningFrame(const std::string& frame_name) {
    if (apply_reference_transform_to_frozen_frame_ &&
        !frozen_camera_frame_id_.empty() &&
        frame_name == frozen_camera_frame_id_) {
      pinocchio::SE3 shadow_pose;
      if (shadowCameraFramePose(shadow_pose)) {
        return shadow_pose;
      }
    }
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (have_frozen_pose_estimator_reference_ &&
          !pose_estimator_reference_frame_.empty() &&
          frame_name == pose_estimator_reference_frame_) {
        return frozen_pose_estimator_reference_in_planning_;
      }
    }
    return currentFramePose(frame_name);
  }

  pinocchio::SE3 poseEstimatorReferenceTransformSE3() const {
    return pinocchio::SE3(
      pose_estimator_reference_transform_.block<3, 3>(0, 0),
      pose_estimator_reference_transform_.block<3, 1>(0, 3));
  }

  geometry_msgs::msg::TransformStamped transformMsgFromSE3(
    const pinocchio::SE3& tf,
    const std::string& parent_frame,
    const std::string& child_frame) const {
    geometry_msgs::msg::TransformStamped out;
    out.header.stamp = this->now();
    out.header.frame_id = parent_frame;
    out.child_frame_id = child_frame;
    out.transform.translation.x = tf.translation().x();
    out.transform.translation.y = tf.translation().y();
    out.transform.translation.z = tf.translation().z();
    const Eigen::Quaterniond q(tf.rotation());
    out.transform.rotation.x = q.x();
    out.transform.rotation.y = q.y();
    out.transform.rotation.z = q.z();
    out.transform.rotation.w = q.w();
    return out;
  }

  bool shadowCameraFramePose(pinocchio::SE3& out) {
    if (pose_estimator_reference_frame_.empty()) {
      return false;
    }
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (have_frozen_pose_estimator_reference_) {
        out = frozen_pose_estimator_reference_in_planning_;
        if (apply_reference_transform_to_frozen_frame_) {
          out = out * poseEstimatorReferenceTransformSE3();
        }
        return true;
      }
    }

    try {
      out = currentFramePose(pose_estimator_reference_frame_);
    } catch (const std::exception& e) {
      if (!shadow_camera_pose_error_logged_) {
        shadow_camera_pose_error_logged_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Shadow camera frame [%s] is not available before observe: %s",
          pose_estimator_reference_frame_.c_str(),
          e.what());
      }
      return false;
    }

    if (apply_reference_transform_to_frozen_frame_) {
      out = out * poseEstimatorReferenceTransformSE3();
    }
    return true;
  }

  bool shadowCameraFramePoseStamped(geometry_msgs::msg::PoseStamped& out) {
    pinocchio::SE3 pose;
    if (!shadowCameraFramePose(pose)) {
      return false;
    }
    out.header.frame_id = planning_frame_;
    out.pose = poseMsgFromSE3(pose);
    return true;
  }

  void publishFrozenCameraFrame() {
    if (!frozen_camera_tf_broadcaster_) {
      return;
    }
    pinocchio::SE3 shadow_pose;
    if (!shadowCameraFramePose(shadow_pose)) {
      return;
    }
    frozen_camera_tf_broadcaster_->sendTransform(
      transformMsgFromSE3(shadow_pose, planning_frame_, frozen_camera_frame_id_));
  }

  void transformPointCloudToReferenceFrame(sensor_msgs::msg::PointCloud2& cloud) {
    if (!transform_frozen_pointcloud_to_reference_frame_ ||
        apply_reference_transform_to_frozen_frame_) {
      return;
    }

    try {
      sensor_msgs::PointCloud2Iterator<float> iter_x(cloud, "x");
      sensor_msgs::PointCloud2Iterator<float> iter_y(cloud, "y");
      sensor_msgs::PointCloud2Iterator<float> iter_z(cloud, "z");
      for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        if (!std::isfinite(*iter_x) || !std::isfinite(*iter_y) || !std::isfinite(*iter_z)) {
          continue;
        }
        const Eigen::Vector4d p_src(*iter_x, *iter_y, *iter_z, 1.0);
        const Eigen::Vector4d p_ref = pose_estimator_reference_transform_ * p_src;
        *iter_x = static_cast<float>(p_ref.x());
        *iter_y = static_cast<float>(p_ref.y());
        *iter_z = static_cast<float>(p_ref.z());
      }
    } catch (const std::exception& e) {
      if (!pointcloud_transform_error_logged_) {
        pointcloud_transform_error_logged_ = true;
        RCLCPP_WARN(
          this->get_logger(),
          "Frozen point cloud transform skipped: %s",
          e.what());
      }
    }
  }

  void onCameraPointCloud(const sensor_msgs::msg::PointCloud2& msg) {
    if (!frozen_camera_pointcloud_pub_) {
      return;
    }
    pinocchio::SE3 shadow_pose;
    if (!shadowCameraFramePose(shadow_pose)) {
      return;
    }

    sensor_msgs::msg::PointCloud2 out = msg;
    transformPointCloudToReferenceFrame(out);
    out.header.frame_id = frozen_camera_frame_id_;
    out.header.stamp = this->now();
    frozen_camera_pointcloud_pub_->publish(out);
    publishFrozenCameraFrame();

    const auto now_mono = std::chrono::steady_clock::now();
    if (std::chrono::duration<double>(now_mono - last_frozen_pointcloud_log_time_).count() >
        frozen_camera_pointcloud_log_throttle_sec_) {
      last_frozen_pointcloud_log_time_ = now_mono;
      bool frozen = false;
      {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        frozen = have_frozen_pose_estimator_reference_;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Frozen camera cloud relay: source=%s source_frame=%s -> topic=%s frame=%s points=%u mode=%s",
        camera_pointcloud_topic_.c_str(),
        msg.header.frame_id.c_str(),
        frozen_camera_pointcloud_topic_.c_str(),
        frozen_camera_frame_id_.c_str(),
        msg.width * msg.height,
        frozen ? "frozen_at_observe" : "following_camera_link");
    }
  }

  geometry_msgs::msg::PoseStamped e3TargetPoseInPlanningFrame() {
    return transformPoseToPlanningFrame(
      makePoseStamped(e3_target_xyz_, e3_target_rpy_, e3_target_frame_));
  }

  void onArmState(const sensor_msgs::msg::JointState& msg) {
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    std::vector<double> positions;
    positions.reserve(arm_group.joint_names.size());
    for (const auto& joint_name : arm_group.joint_names) {
      const auto it = std::find(msg.name.begin(), msg.name.end(), joint_name);
      if (it == msg.name.end()) {
        return;
      }
      const auto index = static_cast<size_t>(std::distance(msg.name.begin(), it));
      if (index >= msg.position.size() || !std::isfinite(msg.position[index])) {
        return;
      }
      positions.push_back(msg.position[index]);
    }

    {
      std::lock_guard<std::mutex> lock(arm_state_mutex_);
      latest_measured_arm_positions_ = positions;
      latest_arm_state_time_ = std::chrono::steady_clock::now();
      have_complete_arm_state_ = true;
      ++arm_state_sequence_;
    }
    arm_state_cv_.notify_all();

    if (publish_tcp_trajectory_ && record_tcp_trajectory_.load()) {
      try {
        appendTcpTrajectoryPoint(measured_tcp_trajectory_, tcpPointForArmJoints(positions));
      } catch (const std::exception& e) {
        RCLCPP_WARN_ONCE(
          this->get_logger(),
          "Unable to record measured TCP trajectory: %s",
          e.what());
      }
    }
  }

  void onPoseEstimatorTarget(const geometry_msgs::msg::PoseStamped& msg) {
    const auto now = std::chrono::steady_clock::now();
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!pose_estimator_capture_enabled_ || have_pose_estimator_target_) {
        return;
      }
    }

    if (!isValidPoseEstimatorTarget(msg)) {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      resetPoseEstimatorContinuityLocked();
      return;
    }

    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!pose_estimator_capture_enabled_ || have_pose_estimator_target_) {
        return;
      }

      if (!pose_estimator_stream_active_ ||
          std::chrono::duration<double>(now - pose_estimator_last_msg_time_).count() >
            pose_estimator_continuous_max_gap_sec_) {
        pose_estimator_stream_active_ = true;
        pose_estimator_stream_start_time_ = now;
        RCLCPP_INFO(
          this->get_logger(),
          "Pose estimator target stream started; waiting %.2f s continuous publish before locking target.",
          pose_estimator_continuous_required_sec_);
      }
      pose_estimator_last_msg_time_ = now;

      const double continuous_sec =
        std::chrono::duration<double>(now - pose_estimator_stream_start_time_).count();
      if (continuous_sec < pose_estimator_continuous_required_sec_) {
        return;
      }

      latest_pose_estimator_target_ = msg;
      have_pose_estimator_target_ = true;
      RCLCPP_INFO(
        this->get_logger(),
        "Pose estimator target locked after %.3f s continuous publish; later poses will be ignored.",
        continuous_sec);
    }
    pose_estimator_cv_.notify_all();
  }

  void resetPoseEstimatorCaptureLocked() {
    have_pose_estimator_target_ = false;
    pose_estimator_stream_active_ = false;
    pose_estimator_capture_enabled_ = false;
    latest_pose_estimator_target_ = geometry_msgs::msg::PoseStamped();
  }

  bool enablePoseEstimatorCaptureAtCurrentReferencePose() {
    std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
    resetPoseEstimatorCaptureLocked();
    have_frozen_pose_estimator_reference_ = false;
    pose_estimator_capture_enabled_ = true;
    RCLCPP_INFO(
      this->get_logger(),
      "Pose estimator capture enabled; reference frame will be frozen after reading the physical arm state: %s",
      pose_estimator_reference_frame_.empty() ? "<message_header>" : pose_estimator_reference_frame_.c_str());
    return true;
  }

  bool freezePoseEstimatorReferenceAtCurrentArmState() {
    if (pose_estimator_reference_frame_.empty()) {
      return true;
    }
    try {
      const auto reference_pose = currentFramePose(pose_estimator_reference_frame_);
      {
        std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
        frozen_pose_estimator_reference_in_planning_ = reference_pose;
        have_frozen_pose_estimator_reference_ = true;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Pose estimator reference frame [%s] frozen from the measured arm state.",
        pose_estimator_reference_frame_.c_str());
      return true;
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to freeze pose estimator reference frame [%s] in [%s]: %s",
        pose_estimator_reference_frame_.c_str(),
        planning_frame_.c_str(),
        e.what());
      return false;
    }
  }

  bool isValidPoseEstimatorTarget(const geometry_msgs::msg::PoseStamped& msg) const {
    const auto& p = msg.pose.position;
    const auto& q = msg.pose.orientation;
    const double q_norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.z) &&
           std::isfinite(q.x) && std::isfinite(q.y) &&
           std::isfinite(q.z) && std::isfinite(q.w) &&
           q_norm_sq > 1e-12;
  }

  void resetPoseEstimatorContinuityLocked() {
    if (have_pose_estimator_target_) {
      return;
    }
    if (pose_estimator_stream_active_) {
      RCLCPP_WARN(
        this->get_logger(),
        "Pose estimator target stream continuity reset because an invalid pose was received.");
    }
    pose_estimator_stream_active_ = false;
  }

  geometry_msgs::msg::PoseStamped applyPoseEstimatorAxisConversion(
    const geometry_msgs::msg::PoseStamped& in) const {
    geometry_msgs::msg::PoseStamped out = in;
    const Eigen::Matrix3d converted_rotation =
      quaternionEigenFromMsg(in.pose.orientation).toRotationMatrix() *
      pose_estimator_axis_conversion_;
    Eigen::Quaterniond q(converted_rotation);
    q.normalize();
    out.pose.orientation.x = q.x();
    out.pose.orientation.y = q.y();
    out.pose.orientation.z = q.z();
    out.pose.orientation.w = q.w();
    return out;
  }

  geometry_msgs::msg::PoseStamped applyPoseEstimatorReferenceTransform(
    const geometry_msgs::msg::PoseStamped& in) const {
    if (pose_estimator_reference_frame_.empty()) {
      return in;
    }
    if (apply_reference_transform_to_frozen_frame_) {
      geometry_msgs::msg::PoseStamped out = in;
      out.header.frame_id = frozen_camera_frame_id_;
      return out;
    }

    const pinocchio::SE3 reference_from_estimator = poseEstimatorReferenceTransformSE3();
    geometry_msgs::msg::PoseStamped out;
    out.header = in.header;
    out.header.frame_id = pose_estimator_reference_frame_;
    out.pose = poseMsgFromSE3(reference_from_estimator * poseMsgToSE3(in.pose));
    return out;
  }

  geometry_msgs::msg::PoseStamped preparePoseEstimatorTarget(
    const geometry_msgs::msg::PoseStamped& in) const {
    return applyPoseEstimatorAxisConversion(applyPoseEstimatorReferenceTransform(in));
  }

  bool waitForPoseEstimatorTarget(geometry_msgs::msg::PoseStamped& out) {
    std::unique_lock<std::mutex> lock(pose_estimator_mutex_);
    const auto has_target = [this]() {
      return stop_requested_.load() || have_pose_estimator_target_;
    };
    if (!has_target()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting for pose estimator target on [%s] for up to %.2f s after %.2f s continuous publish.",
        pose_estimator_target_topic_.c_str(),
        pose_estimator_wait_timeout_sec_,
        pose_estimator_continuous_required_sec_);
      if (pose_estimator_wait_timeout_sec_ <= 0.0) {
        pose_estimator_cv_.wait(lock, has_target);
      } else if (!pose_estimator_cv_.wait_for(
          lock,
          std::chrono::duration<double>(pose_estimator_wait_timeout_sec_),
          has_target)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "No locked pose estimator target received on [%s] within %.2f s.",
          pose_estimator_target_topic_.c_str(),
          pose_estimator_wait_timeout_sec_);
        return false;
      }
    }
    if (stop_requested_.load() || !have_pose_estimator_target_) {
      return false;
    }
    out = latest_pose_estimator_target_;
    return true;
  }

  bool lockedPoseEstimatorTargetInPlanningFrame(geometry_msgs::msg::PoseStamped& out) {
    geometry_msgs::msg::PoseStamped sensed_pose;
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      if (!have_pose_estimator_target_) {
        return false;
      }
      sensed_pose = latest_pose_estimator_target_;
    }

    geometry_msgs::msg::PoseStamped prepared_pose;
    try {
      prepared_pose = preparePoseEstimatorTarget(sensed_pose);
      out = transformPoseToPlanningFrame(prepared_pose);
    } catch (const std::exception& e) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to transform locked pose estimator target from source_frame [%s] prepared_frame [%s] to [%s] for visualization: %s",
        sensed_pose.header.frame_id.c_str(),
        prepared_pose.header.frame_id.c_str(),
        planning_frame_.c_str(),
        e.what());
      return false;
    }
    return true;
  }

  bool resolveE3TargetPose(geometry_msgs::msg::PoseStamped& out) {
    if (use_yaml_e3_target_) {
      out = e3TargetPoseInPlanningFrame();
      RCLCPP_INFO(
        this->get_logger(),
        "Using YAML E3 target: source_frame=[%s] source_xyz=(%.4f, %.4f, %.4f) source_rpy=(%.4f, %.4f, %.4f) rad -> planning_frame=[%s] xyz=(%.4f, %.4f, %.4f).",
        e3_target_frame_.c_str(),
        e3_target_xyz_[0],
        e3_target_xyz_[1],
        e3_target_xyz_[2],
        e3_target_rpy_[0],
        e3_target_rpy_[1],
        e3_target_rpy_[2],
        planning_frame_.c_str(),
        out.pose.position.x,
        out.pose.position.y,
        out.pose.position.z);
    } else {
      geometry_msgs::msg::PoseStamped sensed_pose;
      if (!waitForPoseEstimatorTarget(sensed_pose)) {
        return false;
      }
      geometry_msgs::msg::PoseStamped prepared_pose;
      try {
        prepared_pose = preparePoseEstimatorTarget(sensed_pose);
        out = transformPoseToPlanningFrame(prepared_pose);
      } catch (const std::exception& e) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Failed to transform pose estimator target from source_frame [%s] prepared_frame [%s] to [%s]: %s",
          sensed_pose.header.frame_id.c_str(),
          prepared_pose.header.frame_id.c_str(),
          planning_frame_.c_str(),
          e.what());
        return false;
      }
      RCLCPP_INFO(
        this->get_logger(),
        "Pose estimator target transform: source_frame=[%s] source_xyz=(%.4f, %.4f, %.4f) -> prepared_frame=[%s] -> %s_xyz=(%.4f, %.4f, %.4f).",
        sensed_pose.header.frame_id.c_str(),
        sensed_pose.pose.position.x,
        sensed_pose.pose.position.y,
        sensed_pose.pose.position.z,
        prepared_pose.header.frame_id.c_str(),
        planning_frame_.c_str(),
        out.pose.position.x,
        out.pose.position.y,
        out.pose.position.z);
    }

    const bool any_forced_target_rpy =
      force_target_roll_ || force_target_pitch_ || force_target_yaw_;
    if (any_forced_target_rpy) {
      const Eigen::Vector3d visual_target_rpy =
        quaternionEigenFromMsg(out.pose.orientation).toRotationMatrix().eulerAngles(0, 1, 2);
      Eigen::Vector3d target_rpy = visual_target_rpy;
      if (force_target_roll_) {
        target_rpy.x() = forced_target_roll_;
      }
      if (force_target_pitch_) {
        target_rpy.y() = forced_target_pitch_;
      }
      if (force_target_yaw_) {
        target_rpy.z() = forced_target_yaw_;
      }
      out.pose.orientation = quaternionMsgFromRpy(
        target_rpy.x(), target_rpy.y(), target_rpy.z());
      RCLCPP_INFO(
        this->get_logger(),
        "Resolved target RPY in [%s]: visual=(%.4f, %.4f, %.4f) rad, final=(%.4f, %.4f, %.4f) rad, forced_axes=(roll:%s pitch:%s yaw:%s).",
        planning_frame_.c_str(),
        visual_target_rpy.x(),
        visual_target_rpy.y(),
        visual_target_rpy.z(),
        target_rpy.x(),
        target_rpy.y(),
        target_rpy.z(),
        force_target_roll_ ? "roll" : "visual",
        force_target_pitch_ ? "pitch" : "visual",
        force_target_yaw_ ? "yaw" : "visual");
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Resolved direct-grasp TCP target in [%s]: xyz=(%.4f, %.4f, %.4f).",
      planning_frame_.c_str(),
      out.pose.position.x,
      out.pose.position.y,
      out.pose.position.z);
    return true;
  }

  geometry_msgs::msg::Pose computeAttachRelativePose(
    const std::string& frame_name,
    const geometry_msgs::msg::PoseStamped& object_world_pose) {
    const pinocchio::SE3 frame_world = currentFramePose(frame_name);
    const pinocchio::SE3 object_world = poseMsgToSE3(object_world_pose.pose);
    return poseMsgFromSE3(frame_world.actInv(object_world));
  }

  sensor_msgs::msg::JointState makeJointStateMsg(
    const Eigen::VectorXd& q_full,
    const rclcpp::Time& stamp) const {
    sensor_msgs::msg::JointState msg;
    msg.header.stamp = stamp;
    msg.name = robot_model_->activeJointNames();
    msg.position.reserve(msg.name.size());
    for (const auto& joint_name : msg.name) {
      msg.position.push_back(q_full[robot_model_->qIndexOfJoint(joint_name)]);
    }
    return msg;
  }

  void publishCurrentTheoreticalJointState(const std::string& stage_name) {
    if (!joint_state_pub_ || !enable_visualization_) {
      return;
    }
    joint_state_pub_->publish(makeJointStateMsg(currentFullState(), this->now()));
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stage [%s] published current theoretical joint state.",
      stage_name.c_str());
  }

  bool publishGroupHoldTrajectory(
    const std::string& group_name,
    const std::vector<double>& target_joints,
    const std::string& stage_name,
    bool record_for_replay = false) {
    if (!trajectory_pub_) {
      return true;
    }

    const auto& group = robot_model_->getPlanningGroup(group_name);
    if (target_joints.size() != group.joint_names.size()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Stage [%s] hold trajectory size mismatch: target=%zu group=%zu",
        stage_name.c_str(),
        target_joints.size(),
        group.joint_names.size());
      return false;
    }

    trajectory_msgs::msg::JointTrajectory trajectory;
    trajectory.header.stamp = this->now();
    trajectory.joint_names = group.joint_names;

    trajectory_msgs::msg::JointTrajectoryPoint point;
    point.positions = target_joints;
    point.velocities.assign(target_joints.size(), 0.0);
    point.accelerations.assign(target_joints.size(), 0.0);
    point.time_from_start = durationFromSec(std::max(0.02, post_action_wait_sec_));
    trajectory.points.push_back(std::move(point));

    trajectory_pub_->publish(trajectory);
    if (record_for_replay) {
      recordReplaySegment(stage_name, trajectory);
    }
    return true;
  }

  void publishGripperCommand(
    const std::string& stage_name,
    int8_t serial_command) {
    std_msgs::msg::Int8 command_msg;
    if (serial_command != 1 && serial_command != -1) {
      throw std::invalid_argument("gripper command must be 1 (closed) or -1 (open)");
    }
    command_msg.data = serial_command;
    gripper_command_pub_->publish(command_msg);

    const bool closing = command_msg.data == 1;
    gripper_state_ = closing ? gripper_close_positions_ : gripper_open_positions_;
    RCLCPP_INFO(
      this->get_logger(),
      "Stage [%s] gripper command sent: serial_int8=%d (%s); no physical-arrival check.",
      stage_name.c_str(),
      static_cast<int>(command_msg.data),
      closing ? "closed" : "open");
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
  }

  template<typename ServiceT>
  bool waitForService(
    const typename rclcpp::Client<ServiceT>::SharedPtr& client,
    const std::string& service_name) {
    if (client->wait_for_service(std::chrono::duration<double>(service_wait_sec_))) {
      return true;
    }
    RCLCPP_ERROR(
      this->get_logger(),
      "Service [%s] not available within %.2f s",
      service_name.c_str(),
      service_wait_sec_);
    return false;
  }

  template<typename ServiceT>
  std::shared_ptr<typename ServiceT::Response> callService(
    const typename rclcpp::Client<ServiceT>::SharedPtr& client,
    const std::shared_ptr<typename ServiceT::Request>& request,
    const std::string& service_name) {
    auto future = client->async_send_request(request);
    if (future.wait_for(std::chrono::duration<double>(response_wait_sec_)) !=
        std::future_status::ready) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Timed out waiting for service [%s] response after %.2f s",
        service_name.c_str(),
        response_wait_sec_);
      return nullptr;
    }
    return future.get();
  }

  bool addMeshObject(
    const std::string& object_id,
    const std::string& mesh_resource,
    const geometry_msgs::msg::PoseStamped& object_pose) {
    auto req = std::make_shared<lite_motion_msgs::srv::AddSceneObject::Request>();
    req->object_id = object_id;
    req->mesh_resource = mesh_resource;
    req->pose = object_pose;
    req->collision_pose = object_pose.pose;
    req->mesh_scale = mesh_scale_;

    auto resp = callService<lite_motion_msgs::srv::AddSceneObject>(
      add_object_client_, req, "add_scene_object");
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Add scene object [%s] failed: %s",
        object_id.c_str(),
        resp ? resp->message.c_str() : "no response");
      return false;
    }
    RCLCPP_INFO(this->get_logger(), "Scene object [%s] loaded from [%s]", object_id.c_str(), mesh_resource.c_str());
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool removeObject(const std::string& object_id, bool warn_on_failure = true) {
    auto req = std::make_shared<lite_motion_msgs::srv::RemoveSceneObject::Request>();
    req->object_id = object_id;
    auto resp = callService<lite_motion_msgs::srv::RemoveSceneObject>(
      remove_object_client_, req, "remove_scene_object");
    if (!resp || !resp->success) {
      if (warn_on_failure) {
        RCLCPP_WARN(
          this->get_logger(),
          "Remove scene object [%s] failed or skipped: %s",
          object_id.c_str(),
          resp ? resp->message.c_str() : "no response");
      }
      return false;
    }
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setObjectAttached(
    const std::string& object_id,
    bool attach,
    const std::string& frame_id,
    const geometry_msgs::msg::Pose* relative_pose = nullptr,
    bool warn_on_failure = true) {
    auto req = std::make_shared<lite_motion_msgs::srv::AttachSceneObject::Request>();
    req->object_id = object_id;
    req->attach = attach;
    req->frame_id = frame_id;
    if (relative_pose != nullptr) {
      req->relative_pose = *relative_pose;
    } else {
      req->relative_pose.orientation.w = 1.0;
    }

    auto resp = callService<lite_motion_msgs::srv::AttachSceneObject>(
      attach_object_client_, req, "attach_scene_object");
    if (!resp || !resp->success) {
      if (warn_on_failure) {
        RCLCPP_ERROR(
          this->get_logger(),
          "%s object [%s] failed: %s",
          attach ? "Attach" : "Detach",
          object_id.c_str(),
          resp ? resp->message.c_str() : "no response");
      }
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "%s [%s] %s [%s]",
      attach ? "Attached" : "Detached",
      object_id.c_str(),
      attach ? "to" : "from",
      frame_id.c_str());
    if (object_id == e3_object_id_) {
      e3_attached_to_tcp_ = attach;
      if (attach && relative_pose != nullptr) {
        e3_relative_pose_in_tcp_ = *relative_pose;
      }
      publishReferenceMarkers(attach ? "after_attach_e3" : "after_detach_e3");
      publishE3Marker();
    }
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setRuntimeSceneCollisionAllowance(
    const std::vector<std::string>& object_ids,
    const std::vector<std::string>& link_names,
    double allowed_penetration_m,
    const std::string& stage_name) {
    auto req = std::make_shared<lite_motion_msgs::srv::SetSceneCollisionAllowance::Request>();
    req->object_ids = object_ids;
    req->link_names = link_names;
    req->allowed_penetration = allowed_penetration_m;

    auto resp = callService<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
      set_scene_collision_allowance_client_, req, "set_scene_collision_allowance");
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Set scene collision allowance failed at stage [%s]: %s",
        stage_name.c_str(),
        resp ? resp->message.c_str() : "no response");
      return false;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Stage [%s] applied collision allowance: objects=%zu links=%zu allowed_penetration=%.4f m",
      stage_name.c_str(),
      object_ids.size(),
      link_names.size(),
      allowed_penetration_m);
    sleepAfterAction(post_action_wait_sec_);
    return true;
  }

  bool setGripperE3CollisionAllowance(double allowed_penetration_m, const std::string& stage_name) {
    return setRuntimeSceneCollisionAllowance(
      std::vector<std::string>{e3_object_id_},
      grasp_collision_links_,
      allowed_penetration_m,
      stage_name);
  }

  double effectiveExecutionWaitSec(const trajectory_msgs::msg::JointTrajectory& traj) const {
    double wait_sec = trajectoryDurationSec(traj);
    if (visualization_playback_enabled_) {
      wait_sec /= visualization_playback_speed_scale_;
    }
    return wait_sec + post_action_wait_sec_ + 0.25;
  }

  double maxJointError(
    const std::vector<double>& lhs,
    const std::vector<double>& rhs) const {
    if (lhs.size() != rhs.size()) {
      return std::numeric_limits<double>::infinity();
    }
    double max_err = 0.0;
    for (size_t i = 0; i < lhs.size(); ++i) {
      // Joint feedback can use an equivalent representation one or more full
      // turns away from the planned angle. Arrival validation must not reject
      // a physically identical pose solely because of a 2*pi offset.
      const double error = std::abs(std::remainder(lhs[i] - rhs[i], 2.0 * kPi));
      max_err = std::max(max_err, error);
    }
    return max_err;
  }

  void logArmJointState(const std::string& stage_name, const std::vector<double>& joints) const {
    std::ostringstream oss;
    oss << "Stage [" << stage_name << "] arm joints:";
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    for (size_t i = 0; i < joints.size() && i < group.joint_names.size(); ++i) {
      oss << " " << group.joint_names[i] << "=" << joints[i];
    }
    RCLCPP_INFO(this->get_logger(), "%s", oss.str().c_str());
  }

  std::string formatArmJointErrors(
    const std::vector<double>& measured,
    const std::vector<double>& target) const {
    std::ostringstream oss;
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    const size_t n = std::min({measured.size(), target.size(), group.joint_names.size()});
    for (size_t i = 0; i < n; ++i) {
      const double raw_error = measured[i] - target[i];
      const double error = std::remainder(raw_error, 2.0 * kPi);
      const double abs_error = std::abs(error);
      oss << " " << group.joint_names[i]
          << "{target=" << target[i]
          << ", measured=" << measured[i]
          << ", error=" << error
          << ", raw_error=" << raw_error
          << ", abs=" << abs_error
          << (abs_error > arm_motion_goal_tolerance_rad_ ? ", BAD}" : ", ok}");
    }
    if (measured.size() != target.size()) {
      oss << " size_mismatch{target=" << target.size()
          << ", measured=" << measured.size() << "}";
    }
    return oss.str();
  }

  std::vector<double> clampArmJointsToLimits(
    const std::vector<double>& joints,
    const std::string& stage_name,
    const std::string& role) const {
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    if (joints.size() != group.joint_names.size()) {
      return joints;
    }

    const auto lower = robot_model_->lowerBoundsForGroup(arm_group_);
    const auto upper = robot_model_->upperBoundsForGroup(arm_group_);
    std::vector<double> clamped = joints;
    std::ostringstream oss;
    bool adjusted = false;
    for (size_t i = 0; i < clamped.size(); ++i) {
      const double before = clamped[i];
      const double bounded = std::clamp(
        before,
        lower[static_cast<Eigen::Index>(i)],
        upper[static_cast<Eigen::Index>(i)]);
      if (std::abs(bounded - before) > 1e-9) {
        if (!adjusted) {
          oss << "Stage [" << stage_name << "] clamped " << role
              << " joints to URDF limits:";
          adjusted = true;
        }
        oss << " " << group.joint_names[i] << "=" << before << "->" << bounded;
        clamped[i] = bounded;
      }
    }
    if (adjusted) {
      RCLCPP_WARN(this->get_logger(), "%s", oss.str().c_str());
    }
    return clamped;
  }

  geometry_msgs::msg::Point tcpPointForArmJoints(
    const std::vector<double>& joints) const {
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    if (joints.size() != arm_group.joint_names.size()) {
      throw std::runtime_error("TCP FK joint vector size does not match arm group DOF");
    }

    const Eigen::VectorXd q_group = Eigen::Map<const Eigen::VectorXd>(
      joints.data(), static_cast<Eigen::Index>(joints.size()));
    const Eigen::VectorXd q_full = robot_model_->groupToFull(
      arm_group_, q_group, robot_model_->neutralConfiguration());
    pinocchio::Data data(robot_model_->model());
    pinocchio::forwardKinematics(robot_model_->model(), data, q_full);
    pinocchio::updateFramePlacements(robot_model_->model(), data);
    const auto& tcp_pose = data.oMf[robot_model_->getFrameIdChecked(tcp_frame_)];

    geometry_msgs::msg::Point point;
    point.x = tcp_pose.translation().x();
    point.y = tcp_pose.translation().y();
    point.z = tcp_pose.translation().z();
    return point;
  }

  void appendTcpTrajectoryPoint(
    std::vector<geometry_msgs::msg::Point>& trajectory,
    const geometry_msgs::msg::Point& point) {
    std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
    if (!trajectory.empty()) {
      const auto& previous = trajectory.back();
      const double dx = point.x - previous.x;
      const double dy = point.y - previous.y;
      const double dz = point.z - previous.z;
      if (std::sqrt(dx * dx + dy * dy + dz * dz) < tcp_trajectory_min_distance_m_) {
        return;
      }
    }

    if (trajectory.size() >= tcp_trajectory_max_points_) {
      std::vector<geometry_msgs::msg::Point> reduced;
      reduced.reserve((trajectory.size() + 2) / 2);
      for (size_t i = 0; i < trajectory.size(); i += 2) {
        reduced.push_back(trajectory[i]);
      }
      if ((trajectory.size() - 1) % 2 != 0) {
        reduced.push_back(trajectory.back());
      }
      trajectory.swap(reduced);
    }
    trajectory.push_back(point);
  }

  void resetTcpTrajectories(const std::vector<double>& initial_joints) {
    {
      std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
      planned_tcp_trajectory_.clear();
      measured_tcp_trajectory_.clear();
    }
    if (!publish_tcp_trajectory_) {
      record_tcp_trajectory_.store(false);
      return;
    }

    const auto initial_point = tcpPointForArmJoints(initial_joints);
    appendTcpTrajectoryPoint(planned_tcp_trajectory_, initial_point);
    appendTcpTrajectoryPoint(measured_tcp_trajectory_, initial_point);
    record_tcp_trajectory_.store(true);
  }

  void recordPlannedTcpTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
    if (!publish_tcp_trajectory_ || !record_tcp_trajectory_.load() || trajectory.points.empty()) {
      return;
    }
    const auto& arm_joint_names = robot_model_->getPlanningGroup(arm_group_).joint_names;
    if (trajectory.joint_names != arm_joint_names) {
      return;
    }

    try {
      for (const auto& point : trajectory.points) {
        if (point.positions.size() != arm_joint_names.size()) {
          continue;
        }
        appendTcpTrajectoryPoint(
          planned_tcp_trajectory_, tcpPointForArmJoints(point.positions));
      }
    } catch (const std::exception& e) {
      RCLCPP_WARN(
        this->get_logger(),
        "Unable to record planned TCP trajectory: %s",
        e.what());
    }
  }

  void recordReplaySegment(
    const std::string& stage_name,
    const trajectory_msgs::msg::JointTrajectory& trajectory) {
    recordPlannedTcpTrajectory(trajectory);
    if (!replay_after_complete_ || trajectory.points.empty()) {
      return;
    }
    replay_segments_.push_back(ReplaySegment{stage_name, trajectory, e3_attached_to_tcp_});
  }

  trajectory_msgs::msg::JointTrajectory scaledTrajectoryForReplay(
    const trajectory_msgs::msg::JointTrajectory& trajectory) const {
    trajectory_msgs::msg::JointTrajectory out = trajectory;
    out.header.stamp = this->now();
    for (auto& point : out.points) {
      point.time_from_start =
        durationFromSec(pointTimeSec(point) / replay_speed_scale_);
    }
    return out;
  }

  double pointTimeSec(const trajectory_msgs::msg::JointTrajectoryPoint& point) const {
    return static_cast<double>(point.time_from_start.sec) +
           static_cast<double>(point.time_from_start.nanosec) * 1e-9;
  }

  std::vector<double> currentNamedJointPositions(
    const std::vector<std::string>& joint_names) const {
    std::vector<double> positions;
    positions.reserve(joint_names.size());
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    for (const auto& joint_name : joint_names) {
      const auto arm_it =
        std::find(arm_group.joint_names.begin(), arm_group.joint_names.end(), joint_name);
      if (arm_it != arm_group.joint_names.end()) {
        positions.push_back(
          arm_state_[static_cast<size_t>(std::distance(arm_group.joint_names.begin(), arm_it))]);
        continue;
      }

      const auto gripper_it =
        std::find(gripper_joint_names_.begin(), gripper_joint_names_.end(), joint_name);
      if (gripper_it != gripper_joint_names_.end()) {
        positions.push_back(
          gripper_state_[static_cast<size_t>(std::distance(gripper_joint_names_.begin(), gripper_it))]);
        continue;
      }

      positions.push_back(0.0);
    }
    return positions;
  }

  void applyNamedJointPositions(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions) {
    const size_t n = std::min(joint_names.size(), positions.size());
    const auto& arm_group = robot_model_->getPlanningGroup(arm_group_);
    for (size_t i = 0; i < n; ++i) {
      const auto arm_it =
        std::find(arm_group.joint_names.begin(), arm_group.joint_names.end(), joint_names[i]);
      if (arm_it != arm_group.joint_names.end()) {
        arm_state_[static_cast<size_t>(std::distance(arm_group.joint_names.begin(), arm_it))] =
          positions[i];
        continue;
      }

      const auto gripper_it =
        std::find(gripper_joint_names_.begin(), gripper_joint_names_.end(), joint_names[i]);
      if (gripper_it != gripper_joint_names_.end()) {
        gripper_state_[static_cast<size_t>(std::distance(gripper_joint_names_.begin(), gripper_it))] =
          positions[i];
      }
    }
  }

  std::vector<double> interpolatePositions(
    const std::vector<double>& from,
    const std::vector<double>& to,
    double ratio) const {
    const size_t n = std::min(from.size(), to.size());
    std::vector<double> out(n, 0.0);
    ratio = std::clamp(ratio, 0.0, 1.0);
    for (size_t i = 0; i < n; ++i) {
      out[i] = from[i] + (to[i] - from[i]) * ratio;
    }
    return out;
  }

  std::vector<double> sampleTrajectoryPositions(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::vector<double>& segment_start_positions,
    double elapsed_sec) const {
    if (trajectory.points.empty()) {
      return segment_start_positions;
    }

    if (trajectory.points.size() == 1) {
      const double duration = std::max(0.02, pointTimeSec(trajectory.points.front()));
      return interpolatePositions(
        segment_start_positions,
        trajectory.points.front().positions,
        elapsed_sec / duration);
    }

    if (elapsed_sec <= pointTimeSec(trajectory.points.front())) {
      return trajectory.points.front().positions;
    }

    for (size_t i = 1; i < trajectory.points.size(); ++i) {
      const double t0 = pointTimeSec(trajectory.points[i - 1]);
      const double t1 = pointTimeSec(trajectory.points[i]);
      if (elapsed_sec <= t1) {
        const double denom = std::max(1e-6, t1 - t0);
        return interpolatePositions(
          trajectory.points[i - 1].positions,
          trajectory.points[i].positions,
          (elapsed_sec - t0) / denom);
      }
    }

    return trajectory.points.back().positions;
  }

  std::vector<double> applyArmCommandVelocityLimit(
    const std::vector<double>& desired,
    const std::vector<double>& previous,
    double dt_sec) const {
    if (desired.size() != previous.size() ||
        desired.size() != max_arm_command_velocity_rad_s_.size()) {
      return desired;
    }

    std::vector<double> limited = desired;
    for (size_t i = 0; i < desired.size(); ++i) {
      const double vmax = max_arm_command_velocity_rad_s_[i];
      if (vmax <= 0.0) {
        continue;
      }
      const double max_step = vmax * std::max(1e-3, dt_sec);
      limited[i] = std::clamp(desired[i], previous[i] - max_step, previous[i] + max_step);
    }
    return limited;
  }

  void publishArmCommand(
    const std::vector<std::string>& joint_names,
    const std::vector<double>& positions) {
    if (!arm_command_pub_ || joint_names.size() != positions.size()) {
      return;
    }

    sensor_msgs::msg::JointState msg;
    msg.header.stamp = this->now();
    msg.name = joint_names;
    msg.position = positions;
    arm_command_pub_->publish(msg);
  }

  void publishArmCommandCurrentState(const std::string& stage_name) {
    if (!arm_command_pub_) {
      return;
    }
    const auto& group = robot_model_->getPlanningGroup(arm_group_);
    publishArmCommand(group.joint_names, arm_state_);
    RCLCPP_DEBUG(
      this->get_logger(),
      "Stage [%s] published current arm command.",
      stage_name.c_str());
  }

  bool publishArmCommandTrajectory(
    const trajectory_msgs::msg::JointTrajectory& trajectory,
    const std::string& stage_name) {
    if (!arm_command_pub_ || trajectory.points.empty()) {
      return true;
    }

    const auto segment_start_positions = currentNamedJointPositions(trajectory.joint_names);
    std::vector<double> previous_command = segment_start_positions;
    const auto final_positions = trajectory.points.back().positions;
    const double original_duration = std::max(0.02, trajectoryDurationSec(trajectory));
    const double period_sec = 1.0 / arm_command_publish_hz_;
    const auto started_at = std::chrono::steady_clock::now();
    auto previous_tick = started_at;

    RCLCPP_INFO(
      this->get_logger(),
      "Stage [%s] publishing arm command samples: hz=%.2f speed_scale=%.3f duration=%.3f s",
      stage_name.c_str(),
      arm_command_publish_hz_,
      arm_command_speed_scale_,
      original_duration);

    publishArmCommand(trajectory.joint_names, previous_command);

    while (!stop_requested_.load()) {
      const auto now_tp = std::chrono::steady_clock::now();
      const double wall_elapsed = std::chrono::duration<double>(now_tp - started_at).count();
      const double dt_sec = std::max(1e-3, std::chrono::duration<double>(now_tp - previous_tick).count());
      previous_tick = now_tp;

      const double trajectory_elapsed =
        std::min(original_duration, wall_elapsed * arm_command_speed_scale_);
      const auto desired = sampleTrajectoryPositions(
        trajectory,
        segment_start_positions,
        trajectory_elapsed);
      previous_command = applyArmCommandVelocityLimit(desired, previous_command, dt_sec);
      publishArmCommand(trajectory.joint_names, previous_command);

      if (trajectory_elapsed >= original_duration &&
          maxJointError(previous_command, final_positions) <= arm_command_goal_tolerance_rad_) {
        break;
      }

      std::this_thread::sleep_for(std::chrono::duration<double>(period_sec));
    }

    publishArmCommand(trajectory.joint_names, final_positions);
    return !stop_requested_.load();
  }

  bool executeArmJointPlan(
    const std::vector<double>& target_joints,
    const std::string& stage_name) {
    const auto planning_start_joints =
      clampArmJointsToLimits(arm_state_, stage_name, "start");
    const auto planning_target_joints =
      clampArmJointsToLimits(target_joints, stage_name, "goal");
    if (planning_start_joints != arm_state_) {
      arm_state_ = planning_start_joints;
      publishCurrentTheoreticalJointState(stage_name + "_clamped_start_sync");
    }

    if (planning_target_joints.size() == arm_state_.size()) {
      double max_err = 0.0;
      for (size_t i = 0; i < planning_target_joints.size(); ++i) {
        max_err = std::max(max_err, std::abs(planning_target_joints[i] - arm_state_[i]));
      }
      if (max_err < 1e-6) {
        RCLCPP_INFO(this->get_logger(), "Stage [%s] already at target arm joints.", stage_name.c_str());
        const auto command_started = std::chrono::steady_clock::now();
        publishArmCommandCurrentState(stage_name);
        if (!publishGroupHoldTrajectory(arm_group_, planning_target_joints, stage_name, true)) {
          return false;
        }
        if (!waitForArmAtTarget(planning_target_joints, stage_name, command_started)) {
          return false;
        }
        publishCurrentTheoreticalJointState(stage_name + "_skip_sync");
        sleepAfterAction(post_action_wait_sec_);
        return true;
      }
    }

    auto req = std::make_shared<lite_motion_msgs::srv::PlanArmMotion::Request>();
    req->group_name = arm_group_;
    req->ee_frame = tcp_frame_;
    req->use_pose_goal = false;
    req->start_joint_positions = planning_start_joints;
    req->goal_joint_positions = planning_target_joints;
    req->plan_only = false;
    req->use_cartesian_path = false;
    req->cartesian_max_step = 0.0;
    req->use_request_solver_settings = true;
    req->request_planning_timeout = joint_planning_timeout_;
    req->request_path_check_step = path_check_step_m_;

    auto resp = callService<lite_motion_msgs::srv::PlanArmMotion>(
      plan_client_, req, "plan_arm_motion");
    if (!resp || !resp->success) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Arm joint planning failed at stage [%s]: %s",
        stage_name.c_str(),
        resp ? resp->message.c_str() : "no response");
      return false;
    }
    if (resp->trajectory.points.empty()) {
      RCLCPP_ERROR(this->get_logger(), "Arm joint planning returned empty trajectory at stage [%s]", stage_name.c_str());
      return false;
    }
    recordReplaySegment(stage_name, resp->trajectory);
    const auto final_arm_state = resp->trajectory.points.back().positions;
    const double target_err = maxJointError(final_arm_state, planning_target_joints);
    logArmJointState(stage_name + "_planned_final", final_arm_state);
    if (target_err > joint_goal_tolerance_rad_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Stage [%s] final joint state is %.6f rad away from requested target, tolerance %.6f rad.",
        stage_name.c_str(),
        target_err,
        joint_goal_tolerance_rad_);
      return false;
    }
    const auto command_started = std::chrono::steady_clock::now();
    if (arm_command_pub_) {
      if (!publishArmCommandTrajectory(resp->trajectory, stage_name)) {
        return false;
      }
    } else {
      sleepAfterAction(effectiveExecutionWaitSec(resp->trajectory));
    }
    if (!waitForArmAtTarget(final_arm_state, stage_name, command_started)) {
      return false;
    }
    if (!publishGroupHoldTrajectory(
          arm_group_, final_arm_state, stage_name + "_endpoint_hold")) {
      return false;
    }
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
    return true;
  }

  std::vector<geometry_msgs::msg::PoseStamped> buildAdaptiveRpyTargets(
    const geometry_msgs::msg::PoseStamped& target_pose) const {
    std::vector<double> offset_levels{0.0};
    for (double offset = adaptive_rpy_step_rad_; offset <= adaptive_rpy_max_rad_ + 1e-9;
      offset += adaptive_rpy_step_rad_)
    {
      offset_levels.push_back(offset);
      offset_levels.push_back(-offset);
    }
    if (adaptive_rpy_max_rad_ > 1e-9 &&
      std::abs(offset_levels.back()) < adaptive_rpy_max_rad_ - 1e-9)
    {
      offset_levels.push_back(adaptive_rpy_max_rad_);
      offset_levels.push_back(-adaptive_rpy_max_rad_);
    }

    std::vector<std::array<double, 3>> offsets;
    offsets.reserve(offset_levels.size() * offset_levels.size() * offset_levels.size());
    for (const double roll : offset_levels) {
      for (const double pitch : offset_levels) {
        for (const double yaw : offset_levels) {
          if (std::abs(roll) < 1e-12 && std::abs(pitch) < 1e-12 &&
            std::abs(yaw) < 1e-12)
          {
            continue;
          }
          offsets.push_back({roll, pitch, yaw});
        }
      }
    }

    std::sort(
      offsets.begin(), offsets.end(), [](const std::array<double, 3>& lhs,
      const std::array<double, 3>& rhs) {
        const double lhs_max = std::max({std::abs(lhs[0]), std::abs(lhs[1]), std::abs(lhs[2])});
        const double rhs_max = std::max({std::abs(rhs[0]), std::abs(rhs[1]), std::abs(rhs[2])});
        if (std::abs(lhs_max - rhs_max) > 1e-12) {
          return lhs_max < rhs_max;
        }
        const int lhs_nonzero =
          static_cast<int>(std::abs(lhs[0]) > 1e-12) +
          static_cast<int>(std::abs(lhs[1]) > 1e-12) +
          static_cast<int>(std::abs(lhs[2]) > 1e-12);
        const int rhs_nonzero =
          static_cast<int>(std::abs(rhs[0]) > 1e-12) +
          static_cast<int>(std::abs(rhs[1]) > 1e-12) +
          static_cast<int>(std::abs(rhs[2]) > 1e-12);
        if (lhs_nonzero != rhs_nonzero) {
          return lhs_nonzero > rhs_nonzero;
        }
        const double lhs_sum = std::abs(lhs[0]) + std::abs(lhs[1]) + std::abs(lhs[2]);
        const double rhs_sum = std::abs(rhs[0]) + std::abs(rhs[1]) + std::abs(rhs[2]);
        if (std::abs(lhs_sum - rhs_sum) > 1e-12) {
          return lhs_sum < rhs_sum;
        }
        return lhs < rhs;
      });

    const size_t max_offset_count = static_cast<size_t>(std::max(0, adaptive_rpy_max_candidates_ - 1));
    if (offsets.size() > max_offset_count) {
      offsets.resize(max_offset_count);
    }

    const Eigen::Quaterniond original_q = quaternionEigenFromMsg(target_pose.pose.orientation);
    std::vector<geometry_msgs::msg::PoseStamped> targets;
    targets.reserve(offsets.size() + 1);
    targets.push_back(target_pose);
    for (const auto& offset : offsets) {
      const Eigen::Quaterniond delta =
        Eigen::AngleAxisd(offset[2], Eigen::Vector3d::UnitZ()) *
        Eigen::AngleAxisd(offset[1], Eigen::Vector3d::UnitY()) *
        Eigen::AngleAxisd(offset[0], Eigen::Vector3d::UnitX());
      const Eigen::Quaterniond candidate_q = (delta * original_q).normalized();

      geometry_msgs::msg::PoseStamped candidate = target_pose;
      candidate.pose.orientation.x = candidate_q.x();
      candidate.pose.orientation.y = candidate_q.y();
      candidate.pose.orientation.z = candidate_q.z();
      candidate.pose.orientation.w = candidate_q.w();
      targets.push_back(std::move(candidate));
    }
    return targets;
  }

  bool executeArmPosePlan(
    const geometry_msgs::msg::PoseStamped& target_pose,
    bool use_cartesian_path,
    const std::string& stage_name) {
    const auto candidate_targets = adaptive_rpy_ ?
      buildAdaptiveRpyTargets(target_pose) :
      std::vector<geometry_msgs::msg::PoseStamped>{target_pose};
    const Eigen::Vector3d original_rpy =
      quaternionEigenFromMsg(target_pose.pose.orientation).toRotationMatrix().eulerAngles(0, 1, 2);
    if (adaptive_rpy_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Adaptive RPY planning at fixed xyz=(%.4f, %.4f, %.4f) in [%s], centered on transformed TCP RPY=(%.4f, %.4f, %.4f) rad; candidates=%zu, step=%.2f deg, max=%.2f deg.",
        target_pose.pose.position.x,
        target_pose.pose.position.y,
        target_pose.pose.position.z,
        planning_frame_.c_str(),
        original_rpy.x(),
        original_rpy.y(),
        original_rpy.z(),
        candidate_targets.size(),
        adaptive_rpy_step_rad_ * 180.0 / kPi,
        adaptive_rpy_max_rad_ * 180.0 / kPi);
    }

    trajectory_msgs::msg::JointTrajectory selected_trajectory;
    size_t selected_candidate = 0;
    for (size_t candidate_index = 0; candidate_index < candidate_targets.size(); ++candidate_index) {
      const auto& candidate_target = candidate_targets[candidate_index];
      auto req = std::make_shared<lite_motion_msgs::srv::PlanArmMotion::Request>();
      req->group_name = arm_group_;
      req->ee_frame = tcp_frame_;
      req->use_pose_goal = true;
      req->target_pose = candidate_target;
      req->start_joint_positions = arm_state_;
      req->plan_only = adaptive_rpy_;
      req->use_cartesian_path = use_cartesian_path;
      req->cartesian_max_step = use_cartesian_path ? cartesian_max_step_m_ : 0.0;
      req->use_weighted_orientation_goal = position_only_target_;
      req->orientation_weight_x = 0.0;
      req->orientation_weight_y = 0.0;
      req->orientation_weight_z = 0.0;

      if (use_cartesian_path && cartesian_use_request_solver_settings_) {
        req->use_request_solver_settings = true;
        req->request_ik_max_iterations = cartesian_ik_max_iterations_;
        req->request_ik_pos_tolerance = cartesian_ik_pos_tolerance_;
        req->request_ik_rot_tolerance = cartesian_ik_rot_tolerance_;
        req->request_ik_damping = cartesian_ik_damping_;
        req->request_ik_alpha = cartesian_ik_alpha_;
        req->request_ik_max_step_norm = cartesian_ik_max_step_norm_;
        req->request_ik_max_seed_count = cartesian_ik_max_seed_count_;
        req->request_planning_timeout = cartesian_planning_timeout_;
        req->request_path_check_step = path_check_step_m_;
        req->request_use_sdls = cartesian_use_sdls_;
        req->request_cartesian_nullspace_gain = cartesian_nullspace_gain_;
        req->request_cartesian_singular_threshold = cartesian_singular_threshold_;
        req->request_cartesian_branch_jump_max_rad = cartesian_branch_jump_max_rad_;
        req->request_cartesian_branch_jump_norm_max = cartesian_branch_jump_norm_max_;
        req->request_cartesian_waypoint_pos_tolerance = cartesian_waypoint_pos_tolerance_;
        req->request_cartesian_waypoint_rot_tolerance = cartesian_waypoint_rot_tolerance_;
        req->request_cartesian_min_interpolation_steps = cartesian_min_interpolation_steps_;
        req->request_cartesian_refinement_max_steps = cartesian_refinement_max_steps_;
      } else {
        req->use_request_solver_settings = true;
        req->request_planning_timeout = joint_planning_timeout_;
        req->request_path_check_step = path_check_step_m_;
      }

      auto resp = callService<lite_motion_msgs::srv::PlanArmMotion>(
        plan_client_, req, "plan_arm_motion");
      if (!resp || !resp->success || resp->trajectory.points.empty()) {
        const std::string reason = !resp ? "no response" :
          (!resp->success ? resp->message : "empty trajectory");
        if (!adaptive_rpy_) {
          RCLCPP_ERROR(
            this->get_logger(),
            "Arm pose planning failed at stage [%s]: %s",
            stage_name.c_str(), reason.c_str());
          return false;
        }
        RCLCPP_WARN(
          this->get_logger(),
          "Adaptive RPY candidate %zu/%zu failed at stage [%s]: %s",
          candidate_index + 1,
          candidate_targets.size(),
          stage_name.c_str(),
          reason.c_str());
        continue;
      }

      selected_trajectory = resp->trajectory;
      selected_candidate = candidate_index;
      break;
    }

    if (selected_trajectory.points.empty()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Arm pose planning failed at stage [%s]: no adaptive RPY candidate produced a trajectory.",
        stage_name.c_str());
      return false;
    }
    if (adaptive_rpy_) {
      RCLCPP_INFO(
        this->get_logger(),
        "Adaptive RPY selected candidate %zu/%zu for stage [%s]; TCP xyz remains unchanged.",
        selected_candidate + 1,
        candidate_targets.size(),
        stage_name.c_str());
    }
    recordReplaySegment(stage_name, selected_trajectory);
    const auto final_arm_state = selected_trajectory.points.back().positions;
    logArmJointState(stage_name + "_planned_final", final_arm_state);
    const auto command_started = std::chrono::steady_clock::now();
    if (arm_command_pub_) {
      if (!publishArmCommandTrajectory(selected_trajectory, stage_name)) {
        return false;
      }
    } else {
      sleepAfterAction(effectiveExecutionWaitSec(selected_trajectory));
    }
    if (!waitForArmAtTarget(final_arm_state, stage_name, command_started)) {
      return false;
    }
    if (!publishGroupHoldTrajectory(
          arm_group_, final_arm_state, stage_name + "_endpoint_hold")) {
      return false;
    }
    publishCurrentTheoreticalJointState(stage_name + "_final_sync");
    return true;
  }

  bool readInitialArmJointPositions(std::vector<double>& positions) {
    if (!arm_feedback_enabled_) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Arm feedback is disabled; cannot determine a safe planning start state without serial feedback.");
      return false;
    }
    std::unique_lock<std::mutex> lock(arm_state_mutex_);
    const auto is_recent = [this]() {
      return have_complete_arm_state_ &&
             std::chrono::duration<double>(
               std::chrono::steady_clock::now() - latest_arm_state_time_).count() <=
               arm_state_max_age_sec_;
    };
    if (!is_recent()) {
      RCLCPP_INFO(
        this->get_logger(),
        "Waiting up to %.2f s for a complete arm state on [%s].",
        arm_state_wait_timeout_sec_,
        arm_state_topic_.c_str());
      const auto ready = [this, &is_recent]() {
        return stop_requested_.load() || is_recent();
      };
      if (arm_state_wait_timeout_sec_ <= 0.0) {
        arm_state_cv_.wait(lock, ready);
      } else {
        arm_state_cv_.wait_for(
          lock, std::chrono::duration<double>(arm_state_wait_timeout_sec_), ready);
      }
    }
    if (!stop_requested_.load() && is_recent()) {
      const auto measured = latest_measured_arm_positions_;
      lock.unlock();
      logArmJointState("serial_initial_state", measured);
      positions = measured;
      return true;
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "No complete recent arm state received within %.2f s on [%s]; refusing to plan from a fabricated arm state.",
      arm_state_wait_timeout_sec_,
      arm_state_topic_.c_str());
    return false;
  }

  bool waitForArmAtTarget(
    const std::vector<double>& target,
    const std::string& stage_name,
    const std::chrono::steady_clock::time_point& command_started) {
    if (target.empty()) {
      return false;
    }
    if (!arm_feedback_enabled_) {
      arm_state_ = target;
      return true;
    }

    // Command sampling is velocity-limited and can itself exceed the feedback timeout.
    // Start the arrival window after the final command has been sent, while still
    // accepting feedback received during command publication.
    const auto deadline =
      std::chrono::steady_clock::now() +
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(
        std::chrono::duration<double>(arm_motion_feedback_timeout_sec_));
    std::unique_lock<std::mutex> lock(arm_state_mutex_);
    uint64_t observed_sequence = arm_state_sequence_;
    bool stable = false;
    std::chrono::steady_clock::time_point stable_since{};
    double last_error = std::numeric_limits<double>::infinity();
    std::vector<double> last_measured_positions;

    const auto evaluate_latest_feedback = [this, &target, &command_started, &stable,
                                           &stable_since, &last_error,
                                           &last_measured_positions, &stage_name]() {
      if (!have_complete_arm_state_ || latest_arm_state_time_ < command_started ||
          latest_measured_arm_positions_.size() != target.size()) {
        return false;
      }
      last_measured_positions = latest_measured_arm_positions_;
      last_error = maxJointError(latest_measured_arm_positions_, target);
      const auto feedback_time = latest_arm_state_time_;
      if (last_error <= arm_motion_goal_tolerance_rad_) {
        if (!stable) {
          stable = true;
          stable_since = feedback_time;
        }
        if (std::chrono::duration<double>(feedback_time - stable_since).count() >=
            arm_motion_stable_duration_sec_) {
          arm_state_ = latest_measured_arm_positions_;
          RCLCPP_INFO(
            this->get_logger(),
            "Stage [%s] physical arm reached target: max_error=%.5f rad, stable=%.2f s.",
            stage_name.c_str(),
            last_error,
            arm_motion_stable_duration_sec_);
          return true;
        }
      } else {
        stable = false;
      }
      return false;
    };

    while (!stop_requested_.load()) {
      if (evaluate_latest_feedback()) {
        return true;
      }
      const auto now = std::chrono::steady_clock::now();
      if (now >= deadline) {
        break;
      }
      arm_state_cv_.wait_until(lock, deadline, [this, &observed_sequence]() {
        return stop_requested_.load() || arm_state_sequence_ != observed_sequence;
      });
      if (stop_requested_.load()) {
        return false;
      }
      if (arm_state_sequence_ == observed_sequence) {
        break;
      }
      observed_sequence = arm_state_sequence_;
    }

    if (require_arm_feedback_for_completion_) {
      if (!std::isfinite(last_error)) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Stage [%s] received no complete post-command arm feedback on [%s] within %.2f s; expected all joints in planning group [%s]. Aborting before the next subtask.",
          stage_name.c_str(),
          arm_state_topic_.c_str(),
          arm_motion_feedback_timeout_sec_,
          arm_group_.c_str());
      } else {
        RCLCPP_ERROR(
          this->get_logger(),
          "Stage [%s] did not receive physical-arrival confirmation within %.2f s (last max error %.5f rad); joint_errors:%s; aborting before the next subtask.",
          stage_name.c_str(),
          arm_motion_feedback_timeout_sec_,
          last_error,
          formatArmJointErrors(last_measured_positions, target).c_str());
      }
      return false;
    }
    RCLCPP_WARN(
      this->get_logger(),
      "Stage [%s] physical-arrival confirmation timed out; last max error %.5f rad; joint_errors:%s; continuing because tzb.require_arm_feedback_for_completion=false.",
      stage_name.c_str(),
      last_error,
      formatArmJointErrors(last_measured_positions, target).c_str());
    arm_state_ = target;
    return true;
  }

  bool runSequenceOnce() {
    replay_segments_.clear();
    if (e3_collision_object_enabled_) {
      removeObject(e3_object_id_, false);
    }
    e3_attached_to_tcp_ = false;
    e3_target_world_pose_ = geometry_msgs::msg::PoseStamped();
    {
      std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
      resetPoseEstimatorCaptureLocked();
      have_frozen_pose_estimator_reference_ = false;
    }

    if (!readInitialArmJointPositions(arm_state_)) {
      return false;
    }
    try {
      resetTcpTrajectories(arm_state_);
    } catch (const std::exception& e) {
      record_tcp_trajectory_.store(false);
      RCLCPP_ERROR(this->get_logger(), "Unable to initialize TCP trajectory recording: %s", e.what());
      return false;
    }
    publishCurrentTheoreticalJointState("physical_initial_arm_state");
    publishReferenceMarkers("physical_initial_arm_state");

    if (!executeArmJointPlan(initial_arm_positions_, "move_arm_to_ini_joint_positions")) {
      return false;
    }

    if (!executeArmJointPlan(
          observe_joint_positions_, "move_arm_to_observe_before_perception")) {
      return false;
    }
    publishGripperCommand("open_gripper_at_observe_joint_positions", -1);

    if (!use_yaml_e3_target_) {
      if (!freezePoseEstimatorReferenceAtCurrentArmState()) {
        return false;
      }

      // A pose can be geometrically valid while still being rejected by the
      // arm planner.  Keep acquiring fresh estimator poses until one can be
      // planned and physically reached.  The continuity timer is restarted
      // for every retry, so a candidate is only accepted after the estimator
      // has been publishing it continuously for the configured duration.
      bool grasp_target_reached = false;
      while (!stop_requested_.load()) {
        if (!enablePoseEstimatorCaptureAtCurrentReferencePose()) {
          return false;
        }

        geometry_msgs::msg::PoseStamped locked_pose;
        if (!waitForPoseEstimatorTarget(locked_pose)) {
          if (stop_requested_.load()) {
            return false;
          }
          RCLCPP_WARN(
            this->get_logger(),
            "Pose estimator did not provide a continuous target within %.2f s; retrying pose acquisition.",
            pose_estimator_wait_timeout_sec_);
          continue;
        }

        if (!resolveE3TargetPose(e3_target_world_pose_)) {
          RCLCPP_WARN(
            this->get_logger(),
            "Could not transform the pose estimator target; retrying pose acquisition.");
          continue;
        }
        e3_target_world_pose_.header.frame_id = planning_frame_;
        publishReferenceMarkers("resolved_tcp_target");
        publishE3Marker();

        if (e3_collision_object_enabled_) {
          // Remove any object left by a previous rejected candidate before
          // installing the next candidate in the planning scene.
          removeObject(e3_object_id_, false);
          if (!addMeshObject(e3_object_id_, e3_mesh_resource_, e3_target_world_pose_)) {
            return false;
          }
          if (!setGripperE3CollisionAllowance(
                grasp_collision_allowed_penetration_m_,
                "ignore_e3_collision_with_both_gripper_links")) {
            return false;
          }
        }

        if (executeArmPosePlan(e3_target_world_pose_, false, "move_tcp_to_target")) {
          grasp_target_reached = true;
          break;
        }

        RCLCPP_WARN(
          this->get_logger(),
          "Arm rejected the current pose estimator target; returning to pose acquisition and waiting for a new continuous target.");
        if (e3_collision_object_enabled_) {
          removeObject(e3_object_id_, false);
        }
        {
          std::lock_guard<std::mutex> lock(pose_estimator_mutex_);
          resetPoseEstimatorCaptureLocked();
        }
      }

      if (!grasp_target_reached) {
        return false;
      }
    } else {
      if (!resolveE3TargetPose(e3_target_world_pose_)) {
        return false;
      }
      e3_target_world_pose_.header.frame_id = planning_frame_;
      publishReferenceMarkers("resolved_tcp_target");
      publishE3Marker();

      if (e3_collision_object_enabled_) {
        if (!addMeshObject(e3_object_id_, e3_mesh_resource_, e3_target_world_pose_)) {
          return false;
        }
        if (!setGripperE3CollisionAllowance(
              grasp_collision_allowed_penetration_m_,
              "ignore_e3_collision_with_both_gripper_links")) {
          return false;
        }
      }

      if (!executeArmPosePlan(e3_target_world_pose_, false, "move_tcp_to_target")) {
        return false;
      }
    }
    publishGripperCommand("close_gripper_at_target", 1);
    if (!executeArmJointPlan(place_arm_positions_, "move_arm_to_place_joint_positions")) {
      RCLCPP_WARN(
        this->get_logger(),
        "Failed to reach place_joint_positions; returning to ini_joint_positions before retrying place.");
      if (!executeArmJointPlan(
            initial_arm_positions_, "recover_to_ini_before_place_retry")) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Could not recover to ini_joint_positions after place planning failed; keeping gripper closed.");
        return false;
      }
      if (!executeArmJointPlan(
            place_arm_positions_, "retry_move_arm_to_place_joint_positions")) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Place retry failed after recovery to ini_joint_positions; keeping gripper closed.");
        return false;
      }
    }
    publishGripperCommand("open_gripper_at_place_joint_positions", -1);

    if (!executeArmJointPlan(
          observe_joint_positions_, "move_arm_to_observe_after_place")) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to return to observe_joint_positions after opening the gripper.");
      return false;
    }

    RCLCPP_INFO(
      this->get_logger(),
      "tzb_catch sequence completed: target reached, gripper closed, arm moved to tzb.place_joint_positions, gripper opened, and arm returned to tzb.observe_joint_positions.");
    return true;
  }

  bool waitForTaskStart() {
    if (!task_start_gate_enabled_) {
      return true;
    }
    RCLCPP_INFO(
      this->get_logger(),
      "Autostart disabled; waiting for navigation goal status 1 on [%s] or a task trigger on [%s].",
      navigation_goal_status_topic_.c_str(),
      task_start_topic_.c_str());
    std::unique_lock<std::mutex> lock(task_trigger_mutex_);
    task_trigger_cv_.wait(lock, [this]() {
      return stop_requested_.load() || task_trigger_pending_;
    });
    if (stop_requested_.load()) {
      return false;
    }
    task_trigger_pending_ = false;
    return true;
  }

  void onArm2NavState(const arm2nav_msgs::msg::Arm2Nav& msg) {
    if (!sentry_mode_) {
      return;
    }
    bool should_notify = false;
    {
      std::lock_guard<std::mutex> lock(sentry_mutex_);
      latest_arm2nav_state_ = msg;
      if (msg.arm_trigger && !last_arm2nav_arm_trigger_ && !sentry_task_active_) {
        sentry_trigger_pending_ = true;
        should_notify = true;
      }
      last_arm2nav_arm_trigger_ = msg.arm_trigger;
    }
    if (should_notify) {
      RCLCPP_INFO(
        this->get_logger(),
        "Received arm_trigger=1 from arm2nav; starting the sentry arm task.");
      sentry_cv_.notify_all();
    }
  }

  void publishArm2NavState(
    bool target_need_stop,
    bool nav_trigger,
    bool return_home,
    bool arm_trigger = false) {
    if (!arm2nav_arm_state_pub_) {
      return;
    }
    arm2nav_msgs::msg::Arm2Nav latest_state;
    {
      std::lock_guard<std::mutex> lock(sentry_mutex_);
      latest_state = latest_arm2nav_state_;
    }
    arm2nav_msgs::msg::Arm2Nav msg;
    msg.target_need_stop = target_need_stop;
    msg.suggest_x = latest_state.suggest_x;
    msg.suggest_y = latest_state.suggest_y;
    msg.suggest_yaw = latest_state.suggest_yaw;
    msg.nav_trigger = nav_trigger;
    msg.arm_trigger = arm_trigger;
    msg.return_home = return_home;
    arm2nav_arm_state_pub_->publish(msg);
  }

  bool runSentrySequence() {
    if (!readInitialArmJointPositions(arm_state_)) {
      return false;
    }
    if (!executeArmJointPlan(sentry_joint_positions_, "move_arm_to_sentry_joint_positions")) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Failed to reach sentry_joint_positions; sentry mode cannot start.");
      return false;
    }
    publishGripperCommand("open_gripper_at_sentry_joint_positions", -1);
    publishArm2NavState(false, false, false);
    RCLCPP_INFO(
      this->get_logger(),
      "Sentry mode ready at sentry_joint_positions; waiting for arm2nav arm_trigger.");

    int completed_count = 0;
    while (!stop_requested_.load()) {
      {
        std::unique_lock<std::mutex> lock(sentry_mutex_);
        sentry_cv_.wait(lock, [this]() {
          return stop_requested_.load() || sentry_trigger_pending_;
        });
        if (stop_requested_.load()) {
          return false;
        }
        sentry_trigger_pending_ = false;
        sentry_task_active_ = true;
      }

      // Acknowledge the navigation command and clear target_need_stop while
      // the arm task is running. The arm2nav bridge clears its command latch
      // when it receives this arm-side state.
      publishArm2NavState(false, false, false);
      const bool success = runSequenceOnce();
      {
        std::lock_guard<std::mutex> lock(sentry_mutex_);
        sentry_task_active_ = false;
      }

      if (!success) {
        RCLCPP_ERROR(
          this->get_logger(),
          "Sentry arm task failed; returning to sentry_joint_positions and waiting for the next arm_trigger.");
        if (!executeArmJointPlan(
              sentry_joint_positions_, "recover_to_sentry_joint_positions")) {
          return false;
        }
        continue;
      }

      ++completed_count;
      const bool return_home = completed_count >= sentry_execution_limit_;
      RCLCPP_INFO(
        this->get_logger(),
        "Sentry arm task completed: %d/%d.",
        completed_count, sentry_execution_limit_);
      publishArm2NavState(false, true, return_home);
      std::this_thread::sleep_for(
        std::chrono::duration<double>(sentry_trigger_pulse_sec_));
      publishArm2NavState(false, false, return_home);

      if (return_home) {
        RCLCPP_INFO(
          this->get_logger(),
          "Sentry execution limit reached; published return_home=1.");
        return true;
      }
      if (!executeArmJointPlan(
            sentry_joint_positions_, "return_to_sentry_joint_positions")) {
        return false;
      }
    }
    return false;
  }

  void publishTaskResult(bool success) {
    if (!task_result_pub_) {
      return;
    }
    std_msgs::msg::Int8 msg;
    msg.data = success ? 1 : -1;
    task_result_pub_->publish(msg);
    RCLCPP_INFO(
      this->get_logger(),
      "Published task result %d on [%s].",
      static_cast<int>(msg.data),
      task_result_topic_.c_str());
  }

  void runSequence() {
    if (!waitForService<lite_motion_msgs::srv::PlanArmMotion>(
          plan_client_, "plan_arm_motion") ||
        (e3_collision_object_enabled_ &&
         (!waitForService<lite_motion_msgs::srv::AddSceneObject>(
            add_object_client_, "add_scene_object") ||
          !waitForService<lite_motion_msgs::srv::RemoveSceneObject>(
            remove_object_client_, "remove_scene_object") ||
          !waitForService<lite_motion_msgs::srv::SetSceneCollisionAllowance>(
            set_scene_collision_allowance_client_, "set_scene_collision_allowance")))) {
      return;
    }

    if (sentry_mode_) {
      const bool success = runSentrySequence();
      record_tcp_trajectory_.store(false);
      publishReferenceMarkers("sentry_sequence_complete");
      publishTaskResult(success);
      return;
    }

    do {
      if (!waitForTaskStart()) {
        return;
      }
      const bool success = runSequenceOnce();
      record_tcp_trajectory_.store(false);
      publishReferenceMarkers("tcp_trajectory_complete");
      publishTaskResult(success);
      if (!success) {
        if (task_start_gate_enabled_) {
          continue;
        }
        return;
      }
      if ((sequence_loop_ || task_start_gate_enabled_) && !stop_requested_.load()) {
        sleepAfterAction(loop_pause_sec_);
      }
    } while ((sequence_loop_ || task_start_gate_enabled_) && !stop_requested_.load());
  }

  void replayRecordedSequence() {
    RCLCPP_INFO(
      this->get_logger(),
      "Starting tzb_catch recorded trajectory replay: segments=%zu loop=%s speed_scale=%.3f",
      replay_segments_.size(),
      replay_loop_ ? "true" : "false",
      replay_speed_scale_);

    do {
      arm_state_ = initial_arm_positions_;
      gripper_state_ = gripper_open_positions_;
      e3_attached_to_tcp_ = false;
      publishReferenceMarkers("replay_reset");
      publishE3Marker();
      publishCurrentTheoreticalJointState("replay_reset");

      for (const auto& segment : replay_segments_) {
        if (stop_requested_.load()) {
          return;
        }

        e3_attached_to_tcp_ = segment.e3_attached_during_segment;
        publishReferenceMarkers("replay_" + segment.stage_name);
        publishE3Marker();

        const auto scaled_traj = scaledTrajectoryForReplay(segment.trajectory);
        if (trajectory_pub_) {
          trajectory_pub_->publish(scaled_traj);
        }
        replayTrajectorySegment(segment);
        sleepAfterAction(replay_segment_pause_sec_);
      }

      if (replay_loop_ && !stop_requested_.load()) {
        sleepAfterAction(loop_pause_sec_);
      }
    } while (replay_loop_ && !stop_requested_.load());
  }

  void replayTrajectorySegment(const ReplaySegment& segment) {
    if (segment.trajectory.points.empty()) {
      return;
    }

    const auto segment_start_positions =
      currentNamedJointPositions(segment.trajectory.joint_names);
    const double duration = std::max(0.02, trajectoryDurationSec(segment.trajectory));
    const auto started_at = std::chrono::steady_clock::now();

    RCLCPP_INFO(
      this->get_logger(),
      "Replay segment [%s]: joints=%zu duration=%.3f s replay_duration=%.3f s",
      segment.stage_name.c_str(),
      segment.trajectory.joint_names.size(),
      duration,
      duration / replay_speed_scale_);

    while (!stop_requested_.load()) {
      const double wall_elapsed =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - started_at).count();
      const double trajectory_elapsed = std::min(duration, wall_elapsed * replay_speed_scale_);
      const auto positions = sampleTrajectoryPositions(
        segment.trajectory,
        segment_start_positions,
        trajectory_elapsed);
      applyNamedJointPositions(segment.trajectory.joint_names, positions);
      if (arm_command_publish_during_replay_ &&
          segment.trajectory.joint_names == robot_model_->getPlanningGroup(arm_group_).joint_names) {
        publishArmCommand(segment.trajectory.joint_names, positions);
      }
      publishCurrentTheoreticalJointState("replay_" + segment.stage_name);
      publishReferenceMarkers();
      publishE3Marker();

      if (trajectory_elapsed >= duration) {
        break;
      }
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }

    const auto final_positions = segment.trajectory.points.back().positions;
    applyNamedJointPositions(segment.trajectory.joint_names, final_positions);
    if (arm_command_publish_during_replay_ &&
        segment.trajectory.joint_names == robot_model_->getPlanningGroup(arm_group_).joint_names) {
      publishArmCommand(segment.trajectory.joint_names, final_positions);
    }
    publishCurrentTheoreticalJointState("replay_" + segment.stage_name + "_final");
    publishReferenceMarkers();
    publishE3Marker();
  }

  void sleepAfterAction(double seconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration<double>(seconds);
    while (!stop_requested_.load() && std::chrono::steady_clock::now() < deadline) {
      std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
  }

  void publishReferenceMarkers(const std::string& stage_name = "") {
    if (!ref_marker_pub_) {
      return;
    }
    visualization_msgs::msg::MarkerArray arr;
    visualization_msgs::msg::Marker clear;
    clear.action = visualization_msgs::msg::Marker::DELETEALL;
    arr.markers.push_back(clear);
    int id = 0;
    geometry_msgs::msg::PoseStamped e3_visual_pose;
    if (e3VisualMarkerPose(e3_visual_pose)) {
      appendE3MeshMarker(arr, e3_visual_pose, id);
    }
    geometry_msgs::msg::PoseStamped e3_pose;
    if (currentE3TargetPoseForVisualization(e3_pose)) {
      appendFrameMarkers(arr, e3_pose, "E3_target", id);
    }
    appendFrameMarkers(arr, currentFramePoseStamped(tcp_frame_), tcp_frame_, id);
    if (!pose_estimator_reference_frame_.empty()) {
      try {
        appendFrameMarkers(
          arr,
          currentFramePoseStamped(pose_estimator_reference_frame_),
          pose_estimator_reference_frame_,
          id);
      } catch (const std::exception& e) {
        RCLCPP_DEBUG(
          this->get_logger(),
          "Skipping camera reference markers for [%s]: %s",
          pose_estimator_reference_frame_.c_str(),
          e.what());
      }
    }
    geometry_msgs::msg::PoseStamped shadow_camera_pose;
    if (shadowCameraFramePoseStamped(shadow_camera_pose)) {
      appendFrameMarkers(arr, shadow_camera_pose, frozen_camera_frame_id_, id);
    }
    appendTcpTrajectoryMarkers(arr, id);
    ref_marker_pub_->publish(arr);
    if (!stage_name.empty()) {
      RCLCPP_INFO(this->get_logger(), "Published tzb_catch reference markers at stage [%s].", stage_name.c_str());
    }
  }

  void appendTcpTrajectoryMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    int& id) {
    if (!publish_tcp_trajectory_) {
      return;
    }

    std::vector<geometry_msgs::msg::Point> planned_points;
    std::vector<geometry_msgs::msg::Point> measured_points;
    {
      std::lock_guard<std::mutex> lock(tcp_trajectory_mutex_);
      planned_points = planned_tcp_trajectory_;
      measured_points = measured_tcp_trajectory_;
    }

    const auto append_line = [this, &arr, &id](
      const std::string& marker_namespace,
      const std::vector<geometry_msgs::msg::Point>& points,
      float red,
      float green,
      float blue) {
      if (points.size() < 2) {
        return;
      }
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = planning_frame_;
      marker.header.stamp = this->now();
      marker.ns = marker_namespace;
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.pose.orientation.w = 1.0;
      marker.scale.x = tcp_trajectory_line_width_m_;
      marker.color.r = red;
      marker.color.g = green;
      marker.color.b = blue;
      marker.color.a = 1.0f;
      marker.points = points;
      arr.markers.push_back(std::move(marker));
    };

    append_line("tcp_planned_trajectory", planned_points, 0.0f, 0.8f, 1.0f);
    append_line("tcp_measured_trajectory", measured_points, 1.0f, 0.45f, 0.0f);
  }

  void appendE3MeshMarker(
    visualization_msgs::msg::MarkerArray& arr,
    const geometry_msgs::msg::PoseStamped& pose,
    int& id) const {
    visualization_msgs::msg::Marker marker;
    marker = makeE3MeshMarker(pose, id++);
    arr.markers.push_back(marker);
  }

  visualization_msgs::msg::Marker makeE3MeshMarker(
    const geometry_msgs::msg::PoseStamped& pose,
    int id = 0) const {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = pose.header.frame_id;
    marker.header.stamp = this->now();
    marker.ns = "E3_visual";
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::CUBE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose = pose.pose;
    marker.frame_locked = e3_attached_to_tcp_;
    marker.scale.x = 0.05;
    marker.scale.y = 0.05;
    marker.scale.z = 0.05;
    marker.color.r = 1.0f;
    marker.color.g = 1.0f;
    marker.color.b = 1.0f;
    marker.color.a = 1.0f;
    return marker;
  }

  void publishE3Marker() {
    if (!e3_marker_pub_) {
      return;
    }
    geometry_msgs::msg::PoseStamped pose;
    if (e3VisualMarkerPose(pose)) {
      e3_marker_pub_->publish(makeE3MeshMarker(pose));
      return;
    }

    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = planning_frame_;
    marker.header.stamp = this->now();
    marker.ns = "E3_visual";
    marker.id = 0;
    marker.action = visualization_msgs::msg::Marker::DELETE;
    e3_marker_pub_->publish(marker);
  }

  bool currentE3TargetPoseForVisualization(geometry_msgs::msg::PoseStamped& out) {
    if (!e3_target_world_pose_.header.frame_id.empty()) {
      out = e3_target_world_pose_;
      return true;
    }
    if (use_yaml_e3_target_) {
      out = e3TargetPoseInPlanningFrame();
      return true;
    }
    return lockedPoseEstimatorTargetInPlanningFrame(out);
  }

  bool e3VisualMarkerPose(geometry_msgs::msg::PoseStamped& out) {
    if (e3_attached_to_tcp_) {
      out.header.frame_id = tcp_frame_;
      out.pose = e3_relative_pose_in_tcp_;
      return true;
    }

    return currentE3TargetPoseForVisualization(out);
  }

  void appendFrameMarkers(
    visualization_msgs::msg::MarkerArray& arr,
    const geometry_msgs::msg::PoseStamped& pose,
    const std::string& ns,
    int& id) const {
    const Eigen::Quaterniond q = quaternionEigenFromMsg(pose.pose.orientation);
    const Eigen::Matrix3d rot = q.toRotationMatrix();
    const Eigen::Vector3d origin(pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    const std::array<Eigen::Vector3d, 3> axes = {
      rot * Eigen::Vector3d::UnitX(),
      rot * Eigen::Vector3d::UnitY(),
      rot * Eigen::Vector3d::UnitZ()};

    for (size_t i = 0; i < axes.size(); ++i) {
      visualization_msgs::msg::Marker marker;
      marker.header.frame_id = pose.header.frame_id;
      marker.header.stamp = this->now();
      marker.ns = ns + "_axis";
      marker.id = id++;
      marker.type = visualization_msgs::msg::Marker::ARROW;
      marker.action = visualization_msgs::msg::Marker::ADD;
      marker.scale.x = 0.008;
      marker.scale.y = 0.015;
      marker.scale.z = 0.015;
      marker.color.a = 1.0;
      marker.color.r = (i == 0) ? 1.0f : 0.0f;
      marker.color.g = (i == 1) ? 1.0f : 0.0f;
      marker.color.b = (i == 2) ? 1.0f : 0.0f;
      geometry_msgs::msg::Point p0;
      p0.x = origin.x();
      p0.y = origin.y();
      p0.z = origin.z();
      geometry_msgs::msg::Point p1;
      p1.x = origin.x() + 0.06 * axes[i].x();
      p1.y = origin.y() + 0.06 * axes[i].y();
      p1.z = origin.z() + 0.06 * axes[i].z();
      marker.points.push_back(p0);
      marker.points.push_back(p1);
      arr.markers.push_back(marker);
    }

    visualization_msgs::msg::Marker label;
    label.header.frame_id = pose.header.frame_id;
    label.header.stamp = this->now();
    label.ns = ns + "_label";
    label.id = id++;
    label.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    label.action = visualization_msgs::msg::Marker::ADD;
    label.pose = pose.pose;
    label.pose.position.z += 0.05;
    label.scale.z = 0.03;
    label.color.r = 1.0f;
    label.color.g = 1.0f;
    label.color.b = 1.0f;
    label.color.a = 1.0f;
    label.text = ns;
    arr.markers.push_back(label);
  }

  std::unique_ptr<RobotModel> robot_model_;
  rclcpp::Client<lite_motion_msgs::srv::PlanArmMotion>::SharedPtr plan_client_;
  rclcpp::Client<lite_motion_msgs::srv::AddSceneObject>::SharedPtr add_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::AttachSceneObject>::SharedPtr attach_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::RemoveSceneObject>::SharedPtr remove_object_client_;
  rclcpp::Client<lite_motion_msgs::srv::SetSceneCollisionAllowance>::SharedPtr
    set_scene_collision_allowance_client_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr ref_marker_pub_;
  rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr e3_marker_pub_;
  rclcpp::Publisher<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr arm_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr gripper_command_pub_;
  rclcpp::Publisher<std_msgs::msg::Int8>::SharedPtr task_result_pub_;
  rclcpp::Publisher<arm2nav_msgs::msg::Arm2Nav>::SharedPtr arm2nav_arm_state_pub_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr frozen_camera_pointcloud_pub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr arm_state_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr task_start_sub_;
  rclcpp::Subscription<std_msgs::msg::Int8>::SharedPtr navigation_goal_status_sub_;
  rclcpp::Subscription<arm2nav_msgs::msg::Arm2Nav>::SharedPtr arm2nav_state_sub_;
  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr pose_estimator_sub_;
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr camera_pointcloud_sub_;
  rclcpp::TimerBase::SharedPtr start_timer_;
  rclcpp::TimerBase::SharedPtr reference_marker_timer_;
  rclcpp::TimerBase::SharedPtr frozen_camera_tf_timer_;
  std::unique_ptr<tf2_ros::TransformBroadcaster> frozen_camera_tf_broadcaster_;

  std::thread worker_;
  std::atomic_bool stop_requested_{false};

  std::string planning_frame_;
  std::string arm_group_;
  std::string tcp_frame_;
  std::string joint_positions_unit_{"rad"};
  std::string e3_object_id_;
  std::string e3_mesh_resource_;
  std::vector<double> mesh_scale_;
  std::string e3_target_frame_;
  std::array<double, 3> e3_target_xyz_{};
  std::array<double, 3> e3_target_rpy_{};
  bool use_yaml_e3_target_{true};
  bool force_target_roll_{false};
  bool force_target_pitch_{false};
  bool force_target_yaw_{false};
  bool adaptive_rpy_{false};
  bool position_only_target_{false};
  double adaptive_rpy_step_rad_{15.0 * kPi / 180.0};
  double adaptive_rpy_max_rad_{45.0 * kPi / 180.0};
  int adaptive_rpy_max_candidates_{64};
  std::string target_rpy_unit_{"rad"};
  double forced_target_roll_{0.0};
  double forced_target_pitch_{0.0};
  double forced_target_yaw_{0.0};
  std::string pose_estimator_target_topic_{"/pose_estimator/object_pose"};
  double pose_estimator_wait_timeout_sec_{3.0};
  double pose_estimator_continuous_required_sec_{2.0};
  double pose_estimator_continuous_max_gap_sec_{0.75};
  std::string pose_estimator_reference_frame_;
  Eigen::Matrix4d pose_estimator_reference_transform_{Eigen::Matrix4d::Identity()};
  bool freeze_camera_pointcloud_enabled_{true};
  std::string camera_pointcloud_topic_{"/camera/depth/color/points"};
  std::string frozen_camera_pointcloud_topic_{"/tzb_catch/frozen_observe_camera_points"};
  std::string frozen_camera_frame_id_{"tzb_observe_camera_link"};
  bool apply_reference_transform_to_frozen_frame_{true};
  bool transform_frozen_pointcloud_to_reference_frame_{true};
  double frozen_camera_pointcloud_log_throttle_sec_{2.0};
  bool pointcloud_transform_error_logged_{false};
  bool shadow_camera_pose_error_logged_{false};
  std::chrono::steady_clock::time_point last_frozen_pointcloud_log_time_{};
  Eigen::Matrix3d pose_estimator_axis_conversion_{Eigen::Matrix3d::Identity()};
  std::vector<std::string> gripper_joint_names_;
  std::vector<double> gripper_home_positions_;
  std::vector<double> gripper_open_positions_;
  std::vector<double> gripper_close_positions_;
  std::vector<std::string> grasp_collision_links_;
  double grasp_collision_allowed_penetration_m_{-1.0};
  bool e3_collision_object_enabled_{true};
  double joint_planning_timeout_{2.0};
  double cartesian_planning_timeout_{2.0};
  double path_check_step_m_{0.05};
  double cartesian_max_step_m_{0.02};
  int cartesian_min_interpolation_steps_{2};
  int cartesian_refinement_max_steps_{16};
  bool cartesian_use_request_solver_settings_{true};
  bool cartesian_use_sdls_{true};
  int cartesian_ik_max_iterations_{400};
  double cartesian_ik_pos_tolerance_{0.008};
  double cartesian_ik_rot_tolerance_{0.08};
  double cartesian_ik_damping_{1.0e-6};
  double cartesian_ik_alpha_{0.6};
  double cartesian_ik_max_step_norm_{0.5};
  int cartesian_ik_max_seed_count_{32};
  double cartesian_nullspace_gain_{0.0};
  double cartesian_singular_threshold_{0.03};
  double cartesian_branch_jump_max_rad_{0.75};
  double cartesian_branch_jump_norm_max_{1.20};
  double cartesian_waypoint_pos_tolerance_{0.002};
  double cartesian_waypoint_rot_tolerance_{0.05};
  double post_action_wait_sec_{0.05};
  double service_wait_sec_{30.0};
  double response_wait_sec_{30.0};
  std::string e3_marker_topic_{"/tzb_catch/e3_marker"};
  bool publish_e3_marker_{true};
  double reference_marker_republish_hz_{5.0};
  bool publish_tcp_trajectory_{true};
  double tcp_trajectory_line_width_m_{0.006};
  double tcp_trajectory_min_distance_m_{0.002};
  size_t tcp_trajectory_max_points_{10000};
  std::atomic_bool record_tcp_trajectory_{false};
  std::mutex tcp_trajectory_mutex_;
  std::vector<geometry_msgs::msg::Point> planned_tcp_trajectory_;
  std::vector<geometry_msgs::msg::Point> measured_tcp_trajectory_;
  std::string gripper_command_topic_{"/tzb_catch/gripper_command"};
  bool enable_visualization_{true};
  bool visualization_playback_enabled_{true};
  double visualization_playback_speed_scale_{1.0};
  bool sequence_loop_{false};
  double loop_pause_sec_{0.5};
  bool replay_after_complete_{true};
  bool replay_loop_{true};
  double replay_speed_scale_{0.35};
  double replay_segment_pause_sec_{0.15};
  bool sentry_mode_{false};
  int sentry_execution_limit_{1};
  double sentry_trigger_pulse_sec_{0.2};
  std::string arm2nav_state_topic_{"/arm2nav"};
  std::string arm2nav_arm_state_topic_{"/arm2nav/from_arm"};
  double joint_goal_tolerance_rad_{0.01};
  bool arm_command_enabled_{true};
  double arm_command_publish_hz_{100.0};
  double arm_command_speed_scale_{1.0};
  double arm_command_goal_tolerance_rad_{0.005};
  bool arm_command_publish_during_replay_{false};
  std::vector<double> initial_arm_positions_;
  std::vector<double> place_arm_positions_;
  std::vector<double> observe_joint_positions_;
  std::vector<double> max_arm_command_velocity_rad_s_;
  std::string arm_state_topic_{"/serial/arm_joint_state"};
  bool arm_feedback_enabled_{true};
  double arm_state_wait_timeout_sec_{2.0};
  double arm_state_max_age_sec_{0.5};
  double arm_motion_feedback_timeout_sec_{30.0};
  double arm_motion_goal_tolerance_rad_{0.05};
  double arm_motion_stable_duration_sec_{0.25};
  bool require_arm_feedback_for_completion_{true};
  std::vector<double> arm_state_;
  std::vector<double> gripper_state_;
  std::mutex arm_state_mutex_;
  std::condition_variable arm_state_cv_;
  std::vector<double> latest_measured_arm_positions_;
  bool have_complete_arm_state_{false};
  std::chrono::steady_clock::time_point latest_arm_state_time_{};
  uint64_t arm_state_sequence_{0};
  bool task_autostart_{false};
  bool listen_navigation_goal_status_{true};
  bool task_start_gate_enabled_{false};
  std::string task_start_topic_{"/tzb_catch/start"};
  std::string navigation_goal_status_topic_{"/navigation_goal_status"};
  std::string task_result_topic_{"/tzb_catch/result"};
  std::vector<double> sentry_joint_positions_;
  std::mutex task_trigger_mutex_;
  std::condition_variable task_trigger_cv_;
  bool task_trigger_pending_{false};
  bool navigation_trigger_armed_{true};
  arm2nav_msgs::msg::Arm2Nav latest_arm2nav_state_;
  std::mutex sentry_mutex_;
  std::condition_variable sentry_cv_;
  bool sentry_trigger_pending_{false};
  bool sentry_task_active_{false};
  bool last_arm2nav_arm_trigger_{false};
  std::mutex pose_estimator_mutex_;
  std::condition_variable pose_estimator_cv_;
  geometry_msgs::msg::PoseStamped latest_pose_estimator_target_;
  bool have_pose_estimator_target_{false};
  bool pose_estimator_capture_enabled_{false};
  bool pose_estimator_stream_active_{false};
  bool have_frozen_pose_estimator_reference_{false};
  pinocchio::SE3 frozen_pose_estimator_reference_in_planning_{pinocchio::SE3::Identity()};
  std::chrono::steady_clock::time_point pose_estimator_stream_start_time_{};
  std::chrono::steady_clock::time_point pose_estimator_last_msg_time_{};
  geometry_msgs::msg::PoseStamped e3_target_world_pose_;
  geometry_msgs::msg::Pose e3_relative_pose_in_tcp_;
  bool e3_attached_to_tcp_{false};
  std::vector<ReplaySegment> replay_segments_;
};

}  // namespace lite_motion_planner

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<lite_motion_planner::TzbCatchNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
