#pragma once

#include <optional>
#include <string>
#include <string_view>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Every helper function that isn't provided by MoveIt, rclcpp, or the
// message packages lives here — one file, not one file per topic.

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
geometry_msgs::msg::PoseStamped grasp_pose(const geometry_msgs::msg::PoseStamped& object_centre, double half_height_m,
                                           double grasp_offset_m, double yaw_rad = 0.0);

// approach_height_m above the grasp pose, same frame and orientation.
geometry_msgs::msg::PoseStamped pre_grasp_pose(const geometry_msgs::msg::PoseStamped& grasp, double approach_height_m);

// retreat_height_m above `from` (a grasp or place pose), same frame.
geometry_msgs::msg::PoseStamped retreat_pose(const geometry_msgs::msg::PoseStamped& from, double retreat_height_m);

// Same tcp-height derivation as grasp_pose, applied to destination_centre.
geometry_msgs::msg::PoseStamped place_pose(const geometry_msgs::msg::PoseStamped& destination_centre,
                                           double half_height_m, double grasp_offset_m, double yaw_rad = 0.0);

// approach_height_m above the place pose, clearing the destination tray wall.
geometry_msgs::msg::PoseStamped pre_place_pose(const geometry_msgs::msg::PoseStamped& place, double approach_height_m);

// One axis-aligned box collision object, centred at centre_pose, in metres.
moveit_msgs::msg::CollisionObject make_box_object(const std::string& id, const std::string& frame_id,
                                                  const geometry_msgs::msg::Pose& centre_pose, double size_x,
                                                  double size_y, double size_z);

// One box primitive inside a compound object, already in the compound
// object's own header frame.
struct BoxPrimitivePose {
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  geometry_msgs::msg::Pose pose;
};

// One CollisionObject built from several box primitives sharing an id and
// frame — see docs/skills/pose_helpers.md for why the table/trays need this.
moveit_msgs::msg::CollisionObject make_compound_box_object(const std::string& id, const std::string& frame_id,
                                                           const std::vector<BoxPrimitivePose>& boxes);

// Cartesian acceptance decisions, kept pure (no MoveIt link needed) even
// though motion_commander.cpp is the only caller. -1.0 is computeCartesianPath's error sentinel.
[[nodiscard]] bool accept_cartesian_fraction(double fraction, double min_fraction);
[[nodiscard]] bool accept_segment_duration(double duration_s, double max_duration_s);

// Robotiq 2f-85 jaw gap for a left-knuckle angle, closed form from the macro's
// own link geometry: gap(0)=85mm, gap(0.8)=0.15mm — docs/rca/gripper-grasp-instability.md.
[[nodiscard]] double jaw_gap_m(double knuckle_rad);

// Exact inverse of jaw_gap_m. nullopt if gap_m has no knuckle angle in [0, 0.8] — never clamped.
[[nodiscard]] std::optional<double> knuckle_angle_for_gap_m(double gap_m);

}  // namespace sorting_arm
