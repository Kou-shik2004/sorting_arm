#include "sorting_arm_skills/skill_state.hpp"

namespace sorting_arm {

bool SkillState::try_claim() {
  std::lock_guard<std::mutex> lock(mutex_);
  if (goal_active_) {
    return false;
  }
  goal_active_ = true;
  return true;
}

void SkillState::release() {
  std::lock_guard<std::mutex> lock(mutex_);
  goal_active_ = false;
}

std::optional<std::unique_lock<std::mutex>> SkillState::lock_if_idle() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (goal_active_) {
    return std::nullopt;
  }
  return lock;
}

}  // namespace sorting_arm
