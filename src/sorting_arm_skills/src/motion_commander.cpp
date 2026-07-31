#include "sorting_arm_skills/motion_commander.hpp"

#include <cmath>
#include <stdexcept>
#include <vector>

#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

namespace {
using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;
}  // namespace

MotionCommander::MotionCommander(rclcpp::Node::SharedPtr node, const std::string& planning_frame,
                                 const std::string& tcp_link) {
  node_ = node;
  planning_frame_ = planning_frame;
  tcp_link_ = tcp_link;

  arm_group_ = node_->declare_parameter<std::string>("planning.arm_group", "arm");
  planning_pipeline_ = node_->declare_parameter<std::string>("planning.planning_pipeline", "");
  planner_id_ = node_->declare_parameter<std::string>("planning.planner_id", "");
  planning_time_s_ = node_->declare_parameter<double>("planning.planning_time_s", 5.0);
  planning_attempts_ = node_->declare_parameter<int>("planning.planning_attempts", 5);
  velocity_scaling_ = node_->declare_parameter<double>("planning.velocity_scaling", 0.15);
  acceleration_scaling_ = node_->declare_parameter<double>("planning.acceleration_scaling", 0.15);

  eef_step_m_ = node_->declare_parameter<double>("cartesian.eef_step_m", 0.005);
  min_fraction_ = node_->declare_parameter<double>("cartesian.min_fraction", 0.99);
  max_segment_duration_s_ = node_->declare_parameter<double>("cartesian.max_segment_duration_s", 15.0);

  if (planning_time_s_ <= 0.0) {
    throw std::runtime_error("planning.planning_time_s must be positive");
  }
  if (velocity_scaling_ <= 0.0 || velocity_scaling_ > 1.0) {
    throw std::runtime_error("planning.velocity_scaling must be in (0, 1]");
  }
  if (acceleration_scaling_ <= 0.0 || acceleration_scaling_ > 1.0) {
    throw std::runtime_error("planning.acceleration_scaling must be in (0, 1]");
  }
  if (min_fraction_ <= 0.0 || min_fraction_ > 1.0) {
    throw std::runtime_error("cartesian.min_fraction must be in (0, 1]");
  }

  arm_ = std::make_shared<MoveGroupInterface>(node_, arm_group_);
  arm_->setPoseReferenceFrame(planning_frame_);
  arm_->setPlanningTime(planning_time_s_);
  arm_->setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts_));
  arm_->setMaxVelocityScalingFactor(velocity_scaling_);
  arm_->setMaxAccelerationScalingFactor(acceleration_scaling_);

  // empty pipeline/planner id means "keep move_group's own launch default"
  if (!planning_pipeline_.empty()) {
    arm_->setPlanningPipelineId(planning_pipeline_);
  }
  if (!planner_id_.empty()) {
    arm_->setPlannerId(planner_id_);
  }
}

SkillResult MotionCommander::move_to_named(const std::string& target_name) {
  if (target_name.empty()) {
    return skill_error("named_motion", "empty named target");
  }
  if (!arm_->setNamedTarget(target_name)) {
    return skill_error("named_motion", "unknown named target '" + target_name + "'");
  }
  arm_->setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_->plan(plan);
  if (!plan_result) {
    return skill_error("named_motion", "named target planning failed", plan_result.val);
  }

  const auto exec_result = arm_->execute(plan);
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
  if (arm_->getActiveJoints().size() != joint_values.size()) {
    return skill_error("joint_motion", "configured arm group does not have exactly six active joints");
  }

  const std::vector<double> values(joint_values.begin(), joint_values.end());
  const auto current_state = arm_->getCurrentState();
  if (current_state == nullptr) {
    return skill_error("joint_motion", "no current robot state available");
  }
  current_state->setJointGroupPositions(arm_group_, values);
  if (!current_state->satisfiesBounds(current_state->getJointModelGroup(arm_group_))) {
    return skill_error("joint_motion", "joint target violates configured joint bounds");
  }

  if (!arm_->setJointValueTarget(values)) {
    return skill_error("joint_motion", "setJointValueTarget rejected the target");
  }
  arm_->setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_->plan(plan);
  if (!plan_result) {
    return skill_error("joint_motion", "joint target planning failed", plan_result.val);
  }

  const auto exec_result = arm_->execute(plan);
  if (!exec_result) {
    return skill_error("joint_motion", "joint target execution failed", exec_result.val);
  }
  return skill_ok("joint_motion");
}

SkillResult MotionCommander::move_to_pose(const geometry_msgs::msg::PoseStamped& target) {
  if (!validate_pose(target, planning_frame_)) {
    return skill_error("pose_motion", "pose target failed frame/finite validation");
  }
  if (arm_->getEndEffectorLink() != tcp_link_) {
    return skill_error("pose_motion", "commanded link is not the configured tcp link");
  }

  if (!arm_->setPoseTarget(target)) {
    arm_->clearPoseTargets();
    return skill_error("pose_motion", "setPoseTarget rejected the target");
  }
  arm_->setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_->plan(plan);
  if (!plan_result) {
    arm_->clearPoseTargets();
    return skill_error("pose_motion", "pose planning failed", plan_result.val);
  }

  const auto exec_result = arm_->execute(plan);
  arm_->clearPoseTargets();
  if (!exec_result) {
    return skill_error("pose_motion", "pose execution failed", exec_result.val);
  }
  return skill_ok("pose_motion");
}

SkillResult MotionCommander::move_cartesian_to(const geometry_msgs::msg::PoseStamped& target) {
  if (!validate_pose(target, planning_frame_)) {
    return skill_error("cartesian_motion", "cartesian target failed frame/finite validation");
  }
  if (arm_->getEndEffectorLink() != tcp_link_) {
    return skill_error("cartesian_motion", "commanded link is not the configured tcp link");
  }

  const std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  moveit_msgs::msg::MoveItErrorCodes error_code;
  // installed-jazzy overload without the deprecated jump_threshold parameter
  const double fraction =
      arm_->computeCartesianPath(waypoints, eef_step_m_, trajectory_msg, /*avoid_collisions=*/true, &error_code);

  if (error_code.val != moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
    return skill_error("cartesian_motion", "cartesian path computation reported an error", error_code.val);
  }
  if (!accept_cartesian_fraction(fraction, min_fraction_)) {
    return skill_error("cartesian_motion", "cartesian coverage fraction below the configured minimum", error_code.val);
  }

  const auto current_state = arm_->getCurrentState();
  if (current_state == nullptr) {
    return skill_error("cartesian_motion", "no current robot state available for retiming");
  }
  robot_trajectory::RobotTrajectory robot_traj(arm_->getRobotModel(), arm_group_);
  robot_traj.setRobotTrajectoryMsg(*current_state, trajectory_msg);

  trajectory_processing::TimeOptimalTrajectoryGeneration totg;
  const bool retimed = totg.computeTimeStamps(robot_traj, velocity_scaling_, acceleration_scaling_);
  if (!retimed) {
    return skill_error("cartesian_motion", "time-optimal retiming failed");
  }

  const double duration_s = robot_traj.getDuration();
  if (!accept_segment_duration(duration_s, max_segment_duration_s_)) {
    return skill_error("cartesian_motion", "retimed duration exceeds the configured ceiling");
  }

  robot_traj.getRobotTrajectoryMsg(trajectory_msg);
  MoveGroupInterface::Plan plan;
  plan.trajectory = trajectory_msg;

  const auto exec_result = arm_->execute(plan);
  if (!exec_result) {
    return skill_error("cartesian_motion", "cartesian execution failed", exec_result.val);
  }
  return skill_ok("cartesian_motion");
}

}  // namespace sorting_arm
