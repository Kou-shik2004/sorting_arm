#pragma once

#include <mutex>
#include <optional>

namespace sorting_arm {

// Shared guard: one manipulation goal at a time across Sort/Home (D6).
class SkillState {
 public:
  // false if a goal is already running — caller should REJECT the new goal.
  bool try_claim();
  void release();

  // Locks SceneManager against a running worker for the whole sync call; nullopt if a goal is active.
  std::optional<std::unique_lock<std::mutex>> lock_if_idle();

 private:
  mutable std::mutex mutex_;
  bool goal_active_ = false;
};

}  // namespace sorting_arm
