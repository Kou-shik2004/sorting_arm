#pragma once

#include <array>
#include <memory>
#include <string>
#include <unordered_map>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "moveit/robot_state/robot_state.hpp"
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

  // planning_frame/tcp_link are declared once by whoever owns this node
  // (SkillServerNode or MotionDemo) and passed in, so two commanders on the
  // same node never both try to declare_parameter the same frames.* name.
  MotionCommander(rclcpp::Node::SharedPtr node, const std::string& planning_frame, const std::string& tcp_link);

  SkillResult move_to_named(const std::string& target_name);
  SkillResult move_to_joints(const std::array<double, 6>& joint_values);
  SkillResult move_to_pose(const geometry_msgs::msg::PoseStamped& target);
  SkillResult move_cartesian_to(const geometry_msgs::msg::PoseStamped& target);

  // we split planning from execution here because Pick must reject a branch
  // whose vertical descent collides before the arm moves
  SkillResult plan_pose_candidate(const geometry_msgs::msg::PoseStamped& target, PreparedPoseMotion& prepared);
  SkillResult plan_cartesian_from(const PreparedPoseMotion& start, const geometry_msgs::msg::PoseStamped& target,
                                  PreparedCartesianMotion& prepared);
  SkillResult execute_prepared_pose(const PreparedPoseMotion& prepared);
  SkillResult execute_prepared_cartesian(const PreparedCartesianMotion& prepared);

 private:
  void load_trajectory_limits();

  rclcpp::Node::SharedPtr node_;
  std::string planning_frame_;
  std::string tcp_link_;

  std::string arm_group_;
  double planning_time_s_ = 0.0;
  int planning_attempts_ = 0;
  double velocity_scaling_ = 0.0;
  double acceleration_scaling_ = 0.0;
  std::string planning_pipeline_;
  std::string planner_id_;
  std::string parameter_source_node_;
  double parameter_service_timeout_s_ = 0.0;
  std::unordered_map<std::string, double> velocity_limits_;
  std::unordered_map<std::string, double> acceleration_limits_;

  double eef_step_m_ = 0.0;
  double min_fraction_ = 0.0;
  double max_segment_duration_s_ = 0.0;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
};

}  // namespace sorting_arm
