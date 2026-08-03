#include "sorting_arm_skills/skill_state.hpp"

#include <utility>

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

std::optional<std::string> SkillState::attached_object() const {
  std::lock_guard<std::mutex> lock(mutex_);
  return attached_object_id_;
}

void SkillState::set_attached_object(std::optional<std::string> object_id) {
  std::lock_guard<std::mutex> lock(mutex_);
  attached_object_id_ = std::move(object_id);
}

std::optional<std::unique_lock<std::mutex>> SkillState::lock_if_idle() {
  std::unique_lock<std::mutex> lock(mutex_);
  if (goal_active_) {
    return std::nullopt;
  }
  return lock;
}

}  // namespace sorting_arm
