#pragma once

#include <array>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one arm MoveGroupInterface for the whole process (D6). Every motion
// request sets its target, plans once, and executes that same plan. Home uses
// move_to_named; the manipulation motion now lives in the MTC task (D31).
class MotionCommander {
 public:
  explicit MotionCommander(rclcpp::Node::SharedPtr node);

  SkillResult move_to_named(const std::string& target_name);
  SkillResult move_to_joints(const std::array<double, 6>& joint_values);
  SkillResult move_to_pose(const geometry_msgs::msg::PoseStamped& target);
  SkillResult move_cartesian_to(const geometry_msgs::msg::PoseStamped& target);

 private:
  // plan() then execute() on arm_'s current target, checking each result — the
  // one shape move_to_named, move_to_joints, and move_to_pose all share.
  SkillResult plan_and_execute(const std::string& phase, const std::string& plan_fail_message,
                               const std::string& exec_fail_message);

  // computeCartesianPath from start_state to target. move_group returns it
  // already time-parameterized with our scaling on Jazzy.
  SkillResult compute_cartesian_path(const moveit::core::RobotState& start_state,
                                     const geometry_msgs::msg::PoseStamped& target, const std::string& phase,
                                     moveit_msgs::msg::RobotTrajectory& trajectory_msg);
  [[nodiscard]] bool accept_cartesian_fraction(double fraction) const;

  rclcpp::Node::SharedPtr node_;
  double eef_step_m_ = 0.0;
  double min_fraction_ = 0.0;

  moveit::planning_interface::MoveGroupInterface arm_;
};

}  // namespace sorting_arm
