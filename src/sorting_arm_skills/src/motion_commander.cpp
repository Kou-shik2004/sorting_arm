#include "sorting_arm_skills/motion_commander.hpp"

#include <cmath>
#include <string>
#include <utility>
#include <vector>

#include "moveit/robot_state/conversions.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/robot_trajectory/robot_trajectory.hpp"
#include "moveit_msgs/msg/move_it_error_codes.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"

namespace sorting_arm {

using MoveGroupInterface = moveit::planning_interface::MoveGroupInterface;

MotionCommander::MotionCommander(rclcpp::Node::SharedPtr node) : node_(std::move(node)), arm_(node_, "arm") {
  const auto planning_time_s = node_->declare_parameter<double>("planning.planning_time_s", 5.0);
  const auto planning_attempts = node_->declare_parameter<int>("planning.planning_attempts", 5);
  const auto velocity_scaling = node_->declare_parameter<double>("planning.velocity_scaling", 0.15);
  const auto acceleration_scaling = node_->declare_parameter<double>("planning.acceleration_scaling", 0.15);

  eef_step_m_ = node_->declare_parameter<double>("cartesian.eef_step_m", 0.005);
  min_fraction_ = node_->declare_parameter<double>("cartesian.min_fraction", 0.99);

  arm_.setPoseReferenceFrame("world");
  arm_.setPlanningTime(planning_time_s);
  arm_.setNumPlanningAttempts(static_cast<unsigned int>(planning_attempts));
  arm_.setMaxVelocityScalingFactor(velocity_scaling);
  arm_.setMaxAccelerationScalingFactor(acceleration_scaling);
}

SkillResult MotionCommander::plan_and_execute(const std::string& phase, const std::string& plan_fail_message,
                                              const std::string& exec_fail_message) {
  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    return skill_error(phase, plan_fail_message, plan_result.val);
  }

  const auto exec_result = arm_.execute(plan);
  if (!exec_result) {
    return skill_error(phase, exec_fail_message, exec_result.val);
  }
  return skill_ok(phase);
}

SkillResult MotionCommander::move_to_named(const std::string& target_name) {
  if (!arm_.setNamedTarget(target_name)) {
    return skill_error("named_motion", "unknown named target '" + target_name + "'");
  }
  arm_.setStartStateToCurrentState();

  return plan_and_execute("named_motion", "named target planning failed", "named target execution failed");
}

SkillResult MotionCommander::move_to_joints(const std::array<double, 6>& joint_values) {
  const std::vector<double> values(joint_values.begin(), joint_values.end());
  if (!arm_.setJointValueTarget(values)) {
    return skill_error("joint_motion", "setJointValueTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  return plan_and_execute("joint_motion", "joint target planning failed", "joint target execution failed");
}

SkillResult MotionCommander::move_to_pose(const geometry_msgs::msg::PoseStamped& target) {
  if (!arm_.setPoseTarget(target)) {
    return skill_error("pose_motion", "setPoseTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  return plan_and_execute("pose_motion", "pose planning failed", "pose execution failed");
}

geometry_msgs::msg::PoseStamped MotionCommander::current_tcp_pose() const { return arm_.getCurrentPose("tcp"); }

SkillResult MotionCommander::plan_pose_candidate(const geometry_msgs::msg::PoseStamped& target,
                                                 PreparedPoseMotion& prepared) {
  if (!arm_.setPoseTarget(target)) {
    return skill_error("pre_grasp", "setPoseTarget rejected the target");
  }
  arm_.setStartStateToCurrentState();

  MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    return skill_error("pre_grasp", "pre-grasp candidate planning failed", plan_result.val);
  }

  moveit::core::RobotState start_state(arm_.getRobotModel());
  moveit::core::robotStateMsgToRobotState(plan.start_state, start_state);
  robot_trajectory::RobotTrajectory robot_traj(arm_.getRobotModel(), "arm");
  robot_traj.setRobotTrajectoryMsg(start_state, plan.trajectory);

  prepared.plan = std::move(plan);
  prepared.terminal_state = std::make_shared<moveit::core::RobotState>(robot_traj.getLastWayPoint());
  return skill_ok("pre_grasp");
}

SkillResult MotionCommander::compute_cartesian_path(const moveit::core::RobotState& start_state,
                                                    const geometry_msgs::msg::PoseStamped& target, const std::string& phase,
                                                    moveit_msgs::msg::RobotTrajectory& trajectory_msg) {
  arm_.setStartState(start_state);
  const std::vector<geometry_msgs::msg::Pose> waypoints{target.pose};
  moveit_msgs::msg::MoveItErrorCodes error_code;
  const double fraction =
      arm_.computeCartesianPath(waypoints, eef_step_m_, trajectory_msg, /*avoid_collisions=*/true, &error_code);
  arm_.setStartStateToCurrentState();

  if (!accept_cartesian_fraction(fraction)) {
    return skill_error(phase, "cartesian coverage fraction below the configured minimum", error_code.val);
  }

  return skill_ok(phase);
}

bool MotionCommander::accept_cartesian_fraction(double fraction) const {
  return std::isfinite(fraction) && fraction >= min_fraction_;
}

SkillResult MotionCommander::plan_cartesian_from(const PreparedPoseMotion& start,
                                                 const geometry_msgs::msg::PoseStamped& target,
                                                 PreparedCartesianMotion& prepared) {
  moveit_msgs::msg::RobotTrajectory trajectory_msg;
  const auto compute_result = compute_cartesian_path(*start.terminal_state, target, "descend_preflight", trajectory_msg);
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
  const auto exec_result = arm_.execute(prepared.plan);
  if (!exec_result) {
    return skill_error("pre_grasp", "pre-grasp execution failed", exec_result.val);
  }
  return skill_ok("pre_grasp");
}

SkillResult MotionCommander::execute_prepared_cartesian(const PreparedCartesianMotion& prepared) {
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
  const auto compute_result = compute_cartesian_path(*current_state, target, "cartesian_motion", trajectory_msg);
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
