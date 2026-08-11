#pragma once

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit_msgs/msg/robot_trajectory.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one arm MoveGroupInterface for the whole process (D6) — every motion
// request goes validate -> set target -> plan once -> execute that same plan.
class MotionCommander {
 public:
  // we keep both pick plans so descent starts from the exact IK branch we
  // checked before moving
  struct PreparedPoseMotion {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
    moveit::core::RobotStatePtr terminal_state;
  };

  struct PreparedCartesianMotion {
    moveit::planning_interface::MoveGroupInterface::Plan plan;
  };

  explicit MotionCommander(rclcpp::Node::SharedPtr node);

  SkillResult move_to_named(const std::string& target_name);
  SkillResult move_to_joints(const std::array<double, 6>& joint_values);
  SkillResult move_to_pose(const geometry_msgs::msg::PoseStamped& target);
  SkillResult move_cartesian_to(const geometry_msgs::msg::PoseStamped& target);
  SkillResult current_tcp_pose(geometry_msgs::msg::PoseStamped& pose) const;

  // we split planning from execution here because Pick must reject a branch
  // whose vertical descent collides before the arm moves
  SkillResult plan_pose_candidate(const geometry_msgs::msg::PoseStamped& target, PreparedPoseMotion& prepared);
  SkillResult plan_cartesian_from(const PreparedPoseMotion& start, const geometry_msgs::msg::PoseStamped& target,
                                  PreparedCartesianMotion& prepared);
  SkillResult execute_prepared_pose(const PreparedPoseMotion& prepared);
  SkillResult execute_prepared_cartesian(const PreparedCartesianMotion& prepared);

 private:
  // computeCartesianPath from start_state to target, then TOTG retime — the one
  // path both move_cartesian_to and plan_cartesian_from need, so it lives once.
  SkillResult compute_retimed_cartesian(const moveit::core::RobotState& start_state,
                                        const geometry_msgs::msg::PoseStamped& target, const std::string& phase,
                                        moveit_msgs::msg::RobotTrajectory& trajectory_msg);
  [[nodiscard]] bool accept_cartesian_fraction(double fraction) const;
  [[nodiscard]] bool accept_segment_duration(double duration_s) const;

  rclcpp::Node::SharedPtr node_;
  double planning_time_s_ = 0.0;
  int planning_attempts_ = 0;
  double velocity_scaling_ = 0.0;
  double acceleration_scaling_ = 0.0;

  double eef_step_m_ = 0.0;
  double min_fraction_ = 0.0;
  double max_segment_duration_s_ = 0.0;

  moveit::planning_interface::MoveGroupInterface arm_;
};

}  // namespace sorting_arm
