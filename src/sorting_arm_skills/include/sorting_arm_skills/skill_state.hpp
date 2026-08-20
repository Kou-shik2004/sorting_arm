#pragma once

#include <mutex>
#include <optional>

namespace sorting_arm {

// Shared state across Sort/Home: one manipulation goal at a time (D6). Each
// action server owns its own worker.
class SkillState {
 public:
  // false if a goal is already running — caller should REJECT the new goal.
  bool try_claim();
  void release();

  // Held for the whole sync_objects call, not just this check — SceneManager
  // has no locking of its own, so this lock is what stops a running worker
  // touching it at the same time. nullopt means a goal is active.
  std::optional<std::unique_lock<std::mutex>> lock_if_idle();

 private:
  mutable std::mutex mutex_;
  bool goal_active_ = false;
};

}  // namespace sorting_arm
