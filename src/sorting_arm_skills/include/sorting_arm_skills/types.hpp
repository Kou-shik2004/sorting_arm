#pragma once

#include <string>
#include <utility>

namespace sorting_arm {

// One result for every skill-layer operation. native_code is whatever MoveIt or
// the gripper action gave back, 0 when there isn't one. phase names the step
// that was running when this result was produced ("descend", "close_gripper",
// ...), empty outside a multi-step sequence.
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

}  // namespace sorting_arm
