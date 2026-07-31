#pragma once

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

namespace sorting_arm {

// Shared exclusivity across the Pick/Place/Home servers (D6): one
// manipulation goal at a time, one worker thread, and the object id Pick
// hands to Place.
class SkillState {
 public:
  // false if a goal is already running — caller should REJECT the new goal.
  bool try_claim();
  void release();

  // Starts work on the one owned worker. Only ever called right after a
  // successful try_claim(), so there is never a previous worker still
  // running when this is called.
  void start_worker(std::function<void(std::stop_token)> work);
  void request_stop();

  std::optional<std::string> attached_object() const;
  void set_attached_object(std::optional<std::string> object_id);

  // Held for the whole sync_objects call, not just this check — SceneManager
  // has no locking of its own, so this lock is what stops a running worker
  // touching it at the same time. nullopt means a goal is active.
  std::optional<std::unique_lock<std::mutex>> lock_if_idle();

 private:
  mutable std::mutex mutex_;
  bool goal_active_ = false;
  std::optional<std::string> attached_object_id_;
  std::jthread worker_;
};

}  // namespace sorting_arm
