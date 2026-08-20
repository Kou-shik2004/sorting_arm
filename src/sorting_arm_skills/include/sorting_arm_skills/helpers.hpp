#pragma once

#include <string>

#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Small helpers shared by more than one skill owner live here.

// SkillResult -> the wire shape embedded in every Sort/Home result.
sorting_arm_interfaces::msg::SkillResult to_msg(const SkillResult& result);

// Declares `name` with `fallback` the first time it's asked for on `node`,
// returns the already-declared value every time after. Lets two owners on the
// same node share a parameter without a second declare_parameter throwing
// ParameterAlreadyDeclaredException (rclcpp/node.hpp:434).
template <typename T>
T declare_or_get(rclcpp::Node& node, const std::string& name, const T& fallback) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

}  // namespace sorting_arm
