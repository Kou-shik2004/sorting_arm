#pragma once

#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_skills/gripper_commander.hpp"
#include "sorting_arm_skills/types.hpp"

// Task is forward-declared so MTC's headers stay in the .cpp (shielded by -isystem).
namespace moveit {
namespace task_constructor {
class Task;
}
}  // namespace moveit

namespace sorting_arm {

// Drives one pick-and-place. MTC does all arm motion; only the grasp close is
// ours (GripperCommander), to abort a missed grasp before the arm lifts.
class MtcPickPlace {
 public:
  // static scene ids (table, trays) the grasped object may rest against
  MtcPickPlace(rclcpp::Node::SharedPtr node, std::vector<std::string> support_surfaces);

  // Run the full cycle; report_phase names each step for action feedback
  // NOLINTNEXTLINE(build/include_what_you_use) — 'sort' is our method, not std::sort
  SkillResult sort(const std::string& object_id, double half_height_m, const geometry_msgs::msg::PoseStamped& destination,
                   const std::function<void(const std::string&)>& report_phase);

  // Preempt planning of the in-flight task; a solution already executing runs to completion
  void cancel();

 private:
  std::shared_ptr<moveit::task_constructor::Task> build_reach_task(const std::string& object_id, double half_height_m);
  std::shared_ptr<moveit::task_constructor::Task> build_place_task(const std::string& object_id,
                                                                   const geometry_msgs::msg::PoseStamped& destination);
  // init, store (for cancel), plan, and execute one task. phase names the step.
  SkillResult run_task(const std::shared_ptr<moveit::task_constructor::Task>& task, const std::string& phase);

  rclcpp::Node::SharedPtr node_;
  std::vector<std::string> support_surfaces_;
  GripperCommander gripper_;

  double approach_height_m_ = 0.0;
  double retreat_height_m_ = 0.0;
  double grasp_offset_m_ = 0.0;
  double grasp_angle_delta_rad_ = 0.0;
  double velocity_scaling_ = 0.0;
  double acceleration_scaling_ = 0.0;
  double eef_step_m_ = 0.0;
  int max_solutions_ = 0;

  // shared with cancel() on the action thread, so guard it
  std::mutex task_mutex_;
  std::shared_ptr<moveit::task_constructor::Task> task_;
};

}  // namespace sorting_arm
