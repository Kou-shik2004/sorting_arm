#pragma once

#include <string>
#include <utility>

#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"

namespace sorting_arm {

// One result for every skill op. native_code is MoveIt/gripper's own code (0 if
// none); phase names the step that produced it.
struct [[nodiscard("skill result must be checked")]] SkillResult {
  bool ok = false;
  int native_code = 0;
  std::string phase;
  std::string message;
};

inline SkillResult skill_ok(std::string phase = {}) { return SkillResult{true, 0, std::move(phase), {}}; }

inline SkillResult skill_error(std::string phase, std::string message, int native_code = 0) {
  return SkillResult{false, native_code, std::move(phase), std::move(message)};
}

// SkillResult -> the wire shape embedded in every Sort/Home result.
sorting_arm_interfaces::msg::SkillResult to_msg(const SkillResult& result);

// Declare `name` the first time, return the existing value after — lets two
// owners share a param without a second declare_parameter throwing.
template <typename T>
T declare_or_get(rclcpp::Node& node, const std::string& name, const T& fallback) {
  if (node.has_parameter(name)) {
    return node.get_parameter(name).get_value<T>();
  }
  return node.declare_parameter<T>(name, fallback);
}

}  // namespace sorting_arm
