#include "sorting_arm_skills/motion_commander.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

MotionCommander::MotionCommander(rclcpp::Node::SharedPtr node) : node_(std::move(node)), arm_(node_, "arm") {
  planning_time_s_ = node_->declare_parameter<double>("planning.planning_time_s", 5.0);
  planning_attempts_ = node_->declare_parameter<int>("planning.planning_attempts", 5);
  velocity_scaling_ = node_->declare_parameter<double>("planning.velocity_scaling", 0.15);
  acceleration_scaling_ = node_->declare_parameter<double>("planning.acceleration_scaling", 0.15);

  eef_step_m_ = node_->declare_parameter<double>("cartesian.eef_step_m", 0.005);
  min_fraction_ = node_->declare_parameter<double>("cartesian.min_fraction", 0.99);
  max_segment_duration_s_ = node_->declare_parameter<double>("cartesian.max_segment_duration_s", 15.0);

  require_positive_parameter("planning.planning_time_s", planning_time_s_);
  require_positive_parameter("planning.planning_attempts", planning_attempts_);
  require_unit_interval_parameter("planning.velocity_scaling", velocity_scaling_);
  require_unit_interval_parameter("planning.acceleration_scaling", acceleration_scaling_);
  require_positive_parameter("cartesian.eef_step_m", eef_step_m_);
  require_unit_interval_parameter("cartesian.min_fraction", min_fraction_);
  require_positive_parameter("cartesian.max_segment_duration_s", max_segment_duration_s_);

  arm_.setPoseReferenceFrame("world");
  arm_.setPlanningTime(planning_time_s_);
  arm_.setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
  arm_.setMaxVelocityScalingFactor(velocity_scaling_);
  arm_.setMaxAccelerationScalingFactor(acceleration_scaling_);
}

SkillResult MotionCommander::move_to_named(const std::string& target_name) {
  if (target_name.empty()) {
    return skill_error("named_motion", "empty named target");
  }
  if (!arm_.setNamedTarget(target_name)) {
    return skill_error("named_motion", "unknown named target '" + target_name + "'");
  }
  arm_.setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    return skill_error("named_motion", "named target planning failed", plan_result.val);
  }

  const auto exec_result = arm_.execute(plan);
  if (!exec_result) {
    return skill_error("named_motion", "named target execution failed", exec_result.val);
  }
  return skill_ok("named_motion");
}

SkillResult MotionCommander::move_to_joints(const std::array<double, 6>& joint_values) {
  for (const double value : joint_values) {
    if (!std::isfinite(value)) {
      return skill_error("joint_motion", "non-finite joint value");
    }
  }
  if (arm_.getActiveJoints().size() != joint_values.size()) {
    return skill_error("joint_motion", "configured arm group does not have exactly six active joints");
  }

  const std::vector<double> values(joint_values.begin(), joint_values.end());
  const auto current_state = arm_.getCurrentState();
  if (current_state == nullptr) {
    return skill_error("joint_motion", "no current robot state available");
  }
  current_state->setJointGroupPositions("arm", values);
  if (!current_state->satisfiesBounds(current_state->getJointModelGroup("arm"))) {
    return skill_error("joint_motion", "joint target violates configured joint bounds");
  }

  if (!arm_.setJointValueTarget(values)) {
    return skill_error("joint_motion", "setJointValueTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    return skill_error("joint_motion", "joint target planning failed", plan_result.val);
  }

  const auto exec_result = arm_.execute(plan);
  if (!exec_result) {
    return skill_error("joint_motion", "joint target execution failed", exec_result.val);
  }
  return skill_ok("joint_motion");
}

SkillResult MotionCommander::move_to_pose(const geometry_msgs::msg::PoseStamped& target) {
  if (!validate_pose(target, "world")) {
    return skill_error("pose_motion", "pose target failed frame/finite validation");
  }
  if (arm_.getEndEffectorLink() != "tcp") {
    return skill_error("pose_motion", "commanded link is not the configured tcp link");
  }

  if (!arm_.setPoseTarget(target)) {
    arm_.clearPoseTargets();
    return skill_error("pose_motion", "setPoseTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    arm_.clearPoseTargets();
    return skill_error("pose_motion", "pose planning failed", plan_result.val);
  }

  const auto exec_result = arm_.execute(plan);
  arm_.clearPoseTargets();
  if (!exec_result) {
    return skill_error("pose_motion", "pose execution failed", exec_result.val);
  }
  return skill_ok("pose_motion");
}

SkillResult MotionCommander::current_tcp_pose(geometry_msgs::msg::PoseStamped& pose) const {
  if (arm_.getEndEffectorLink() != "tcp") {
    return skill_error("tcp_pose", "commanded link is not the configured tcp link");
  }

  pose = arm_.getCurrentPose("tcp");
  if (!validate_pose(pose, "world")) {
    return skill_error("tcp_pose", "current tcp pose failed frame/finite validation");
  }
  return skill_ok("tcp_pose");
}

SkillResult MotionCommander::plan_pose_candidate(const geometry_msgs::msg::PoseStamped& target,
                                                 PreparedPoseMotion& prepared) {
  if (!validate_pose(target, "world")) {
    return skill_error("pre_grasp", "pose target failed frame/finite validation");
  }
  if (arm_.getEndEffectorLink() != "tcp") {
    return skill_error("pre_grasp", "commanded link is not the configured tcp link");
  }

  if (!arm_.setPoseTarget(target)) {
    arm_.clearPoseTargets();
    return skill_error("pre_grasp", "setPoseTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  // we ask for one plan here so Pick can inspect each candidate separately
  // and stop after its configured candidate bound
  arm_.setNumPlanningAttempts(1);
  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  arm_.setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
  arm_.clearPoseTargets();
  if (!plan_result) {
    return skill_error("pre_grasp", "pre-grasp candidate planning failed", plan_result.val);
  }

  moveit::core::RobotState start_state(arm_.getRobotModel());
  if (!moveit::core::robotStateMsgToRobotState(plan.start_state, start_state)) {
    return skill_error("pre_grasp", "pre-grasp candidate start state conversion failed");
  }
  robot_trajectory::RobotTrajectory robot_traj(arm_.getRobotModel(), "arm");
  robot_traj.setRobotTrajectoryMsg(start_state, plan.trajectory);
  if (robot_traj.getWayPointCount() == 0) {
    return skill_error("pre_grasp", "pre-grasp candidate contained no trajectory waypoints");
  }

  prepared.plan = std::move(plan);
  prepared.terminal_state = std::make_shared<moveit::core::RobotState>(robot_traj.getLastWayPoint());
  return skill_ok("pre_grasp");
}

SkillResult MotionCommander::compute_retimed_cartesian(const moveit::core::RobotState& start_state,
                                                       const geometry_msgs::msg::PoseStamped& target,
                                                       const std::string& phase,
                                                       moveit_msgs::msg::RobotTrajectory& trajectory_msg) {
  if (!validate_pose(target, "world")) {
    return skill_error(phase, "cartesian target failed frame/finite validation");
  }
  if (arm_.getEndEffectorLink() != "tcp") {
    return skill_error(phase, "commanded link is not the configured tcp link");
  }

  arm_.setStartState(start_state);
  const std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction =
      arm_.computeCartesianPath(waypoints, eef_step_m_, trajectory_msg, /*avoid_collisions=*/true, &error_code);
  arm_.setStartStateToCurrentState();

  if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return skill_error(phase, "cartesian path computation reported an error", error_code.val);
  }
  if (!accept_cartesian_fraction(fraction)) {
    return skill_error(phase, "cartesian coverage fraction below the configured minimum", error_code.val);
  }

  robot_trajectory::RobotTrajectory robot_traj(arm_.getRobotModel(), "arm");
  robot_traj.setRobotTrajectoryMsg(start_state, trajectory_msg);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  if (!totg.computeTimeStamps(robot_traj, velocity_scaling_, acceleration_scaling_)) {
    return skill_error(phase, "time-optimal retiming failed");
  }
  if (!accept_segment_duration(robot_traj.getDuration())) {
    return skill_error(phase, "retimed duration exceeds the configured ceiling");
  }

  robot_traj.getRobotTrajectoryMsg(trajectory_msg);
  return skill_ok(phase);
}

bool MotionCommander::accept_cartesian_fraction(double fraction) const {
  return std::isfinite(fraction) && fraction >= min_fraction_;
}

bool MotionCommander::accept_segment_duration(double duration_s) const {
  return std::isfinite(duration_s) && duration_s > 0.0 && duration_s <= max_segment_duration_s_;
}

SkillResult MotionCommander::plan_cartesian_from(const PreparedPoseMotion& start,
                                                 const geometry_msgs::msg::PoseStamped& target,
                                                 PreparedCartesianMotion& prepared) {
  if (start.terminal_state == nullptr) {
    return skill_error("descend_preflight", "pre-grasp candidate has no terminal robot state");
  }

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const auto compute_result = compute_retimed_cartesian(*start.terminal_state, target, "descend_preflight", trajectory_msg);
  if (!compute_result.ok) {
    return compute_result;
  }

  MoveGroupInterface::Plan plan;
  moveit::core::robotStateToRobotStateMsg(*start.terminal_state, plan.start_state);
  plan.trajectory = trajectory_msg;
  prepared.plan = std::move(plan);
  return skill_ok("descend_preflight");
}

SkillResult MotionCommander::execute_prepared_pose(const PreparedPoseMotion& prepared) {
  if (prepared.plan.trajectory.joint_trajectory.points.empty()) {
    return skill_error("pre_grasp", "prepared pre-grasp trajectory is empty");
  }
  const auto exec_result = arm_.execute(prepared.plan);
  if (!exec_result) {
    return skill_error("pre_grasp", "pre-grasp execution failed", exec_result.val);
  }
  return skill_ok("pre_grasp");
}

SkillResult MotionCommander::execute_prepared_cartesian(const PreparedCartesianMotion& prepared) {
  if (prepared.plan.trajectory.joint_trajectory.points.empty()) {
    return skill_error("descend", "prepared Cartesian trajectory is empty");
  }
  const auto exec_result = arm_.execute(prepared.plan);
  if (!exec_result) {
    return skill_error("descend", "cartesian execution failed", exec_result.val);
  }
  return skill_ok("descend");
}

SkillResult MotionCommander::move_cartesian_to(const geometry_msgs::msg::PoseStamped& target) {
  const auto current_state = arm_.getCurrentState();
  if (current_state == nullptr) {
    return skill_error("cartesian_motion", "no current robot state available");
  }

  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const auto compute_result = compute_retimed_cartesian(*current_state, target, "cartesian_motion", trajectory_msg);
  if (!compute_result.ok) {
    return compute_result;
  }

  MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory_msg;

  const auto exec_result = arm_.execute(plan);
  if (!exec_result) {
    return skill_error("cartesian_motion", "cartesian execution failed", exec_result.val);
  }
  return skill_ok("cartesian_motion");
}

}  // namespace sorting_arm
