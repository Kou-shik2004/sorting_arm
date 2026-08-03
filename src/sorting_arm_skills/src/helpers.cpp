#include "sorting_arm_skills/helpers.hpp"

#include <cmath>

namespace sorting_arm {

sorting_arm_interfaces::msg::SkillResult to_msg(const SkillResult& result) {
  sorting_arm_interfaces::msg::SkillResult msg;
  msg.ok = result.ok;
  msg.native_code = result.native_code;
  msg.phase = result.phase;
  msg.message = result.message;
  return msg;
}

static bool finite_pose(const geometry_msgs::msg::Pose& pose) {
  return std::isfinite(pose.position.x) && std::isfinite(pose.position.y) && std::isfinite(pose.position.z) && std::isfinite(pose.orientation.x) &&
         std::isfinite(pose.orientation.y) && std::isfinite(pose.orientation.z) && std::isfinite(pose.orientation.w);
}

// `base` offset by delta_z along world Z — every grasp/pre-grasp/place/
// pre-place/retreat pose is one of these.
static geometry_msgs::msg::PoseStamped offset_z(const geometry_msgs::msg::PoseStamped& base, double delta_z) {
  geometry_msgs::msg::PoseStamped out = base;
  out.pose.position.z += delta_z;
  return out;
}

bool validate_pose(const geometry_msgs::msg::PoseStamped& pose, std::string_view expected_frame) {
  if (pose.header.frame_id != expected_frame) {
    return false;
  }
  return finite_pose(pose.pose);
}

geometry_msgs::msg::Quaternion top_down_orientation(double yaw_rad) {
  // Closed form of RPY(pi, 0, yaw) — see docs/skills/pose_helpers.md.
  geometry_msgs::msg::Quaternion q;
  q.x = std::cos(yaw_rad / 2.0);
  q.y = std::sin(yaw_rad / 2.0);
  q.z = 0.0;
  q.w = 0.0;
  return q;
}

geometry_msgs::msg::PoseStamped grasp_pose(const geometry_msgs::msg::PoseStamped& object_centre, double half_height_m, double grasp_offset_m,
                                           double yaw_rad) {
  geometry_msgs::msg::PoseStamped out = object_centre;
  out.pose.position.z = object_centre.pose.position.z + half_height_m + grasp_offset_m;
  out.pose.orientation = top_down_orientation(yaw_rad);
  return out;
}

geometry_msgs::msg::PoseStamped pre_grasp_pose(const geometry_msgs::msg::PoseStamped& grasp, double approach_height_m) {
  return offset_z(grasp, approach_height_m);
}

geometry_msgs::msg::PoseStamped retreat_pose(const geometry_msgs::msg::PoseStamped& from, double retreat_height_m) {
  return offset_z(from, retreat_height_m);
}

geometry_msgs::msg::PoseStamped place_pose(const geometry_msgs::msg::PoseStamped& destination_centre, double half_height_m, double grasp_offset_m,
                                           double yaw_rad) {
  // The gripper's own geometry doesn't change between pick and place.
  return grasp_pose(destination_centre, half_height_m, grasp_offset_m, yaw_rad);
}

geometry_msgs::msg::PoseStamped pre_place_pose(const geometry_msgs::msg::PoseStamped& place, double approach_height_m) {
  return offset_z(place, approach_height_m);
}

}  // namespace sorting_arm
