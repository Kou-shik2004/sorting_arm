#pragma once

#include <string>
#include <string_view>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Small helpers shared by more than one skill owner live here.

// SkillResult -> the wire shape embedded in every Pick/Place/Home result.
sorting_arm_interfaces::msg::SkillResult to_msg(const SkillResult& result);

// Declares `name` with `fallback` the first time it's asked for on `node`,
// returns the already-declared value every time after. Lets two servers on
// the same node share targets.* without a second declare_parameter throwing
// ParameterAlreadyDeclaredException (rclcpp/node.hpp:434).
template <typename T>
T declare_or_get(rclcpp::Node& node, const std::string& name, const T& fallback) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

// True only if pose.header.frame_id equals expected_frame exactly and every
// position/orientation component is finite.
[[nodiscard]] bool validate_pose(const geometry_msgs::msg::PoseStamped& pose, std::string_view expected_frame);

// The only tool orientation grasp/pre-grasp/place/pre-place poses use — see
// docs/skills/pose_helpers.md for the closed-form derivation.
geometry_msgs::msg::Quaternion top_down_orientation(double yaw_rad);

// object_centre is the object's geometric centre, never a tcp pose.
// grasp_offset_m is signed: 0 grips at the centre, positive is higher.
geometry_msgs::msg::PoseStamped grasp_pose(const geometry_msgs::msg::PoseStamped& object_centre, double half_height_m, double grasp_offset_m,
                                           double yaw_rad = 0.0);

// approach_height_m above the grasp pose, same frame and orientation.
geometry_msgs::msg::PoseStamped pre_grasp_pose(const geometry_msgs::msg::PoseStamped& grasp, double approach_height_m);

// retreat_height_m above `from` (a grasp or place pose), same frame.
geometry_msgs::msg::PoseStamped retreat_pose(const geometry_msgs::msg::PoseStamped& from, double retreat_height_m);

// Same tcp-height derivation as grasp_pose, applied to destination_centre.
geometry_msgs::msg::PoseStamped place_pose(const geometry_msgs::msg::PoseStamped& destination_centre, double half_height_m, double grasp_offset_m,
                                           double yaw_rad = 0.0);

// approach_height_m above the place pose, clearing the destination tray wall.
geometry_msgs::msg::PoseStamped pre_place_pose(const geometry_msgs::msg::PoseStamped& place, double approach_height_m);

}  // namespace sorting_arm
