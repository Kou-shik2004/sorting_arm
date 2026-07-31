#pragma once

#include <array>
#include <memory>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one arm MoveGroupInterface for the whole process (D6) — every motion
// request goes validate -> set target -> plan once -> execute that same plan.
class MotionCommander {
 public:
  // planning_frame/tcp_link are declared once by whoever owns this node
  // (SkillServerNode or MotionDemo) and passed in, so two commanders on the
  // same node never both try to declare_parameter the same frames.* name.
  MotionCommander(rclcpp::Node::SharedPtr node, const std::string& planning_frame, const std::string& tcp_link);

  SkillResult move_to_named(const std::string& target_name);
  SkillResult move_to_joints(const std::array<double, 6>& joint_values);
  SkillResult move_to_pose(const geometry_msgs::msg::PoseStamped& target);
  SkillResult move_cartesian_to(const geometry_msgs::msg::PoseStamped& target);

 private:
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

  double eef_step_m_ = 0.0;
  double min_fraction_ = 0.0;
  double max_segment_duration_s_ = 0.0;

  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> arm_;
};

}  // namespace sorting_arm
