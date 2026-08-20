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

// MTC's Task lives only in the .cpp so its headers never reach the rest of the
// package (they compile in as -isystem there, which is what shields -Werror).
namespace moveit {
namespace task_constructor {
class Task;
}
}  // namespace moveit

namespace sorting_arm {

// Drives one pick-and-place. MTC plans and executes all arm motion; the grasp
// close is our own GripperCommander so it can read the stall result and abort a
// missed grasp before the arm lifts. Because Task::execute() is atomic, the
// cycle is two MTC tasks with close() between them: open, reach+grasp (task A),
// close, carry+place (task B). Only the close is custom — the pre-grasp open and
// the place release stay native MTC. cancel() preempts the in-flight task from
// the action thread.
class MtcPickPlace {
 public:
  // support_surfaces are the static scene ids (table, trays) the grasped object
  // is allowed to rest against.
  MtcPickPlace(rclcpp::Node::SharedPtr node, std::vector<std::string> support_surfaces);

  // Open the gripper, plan+execute the reach-and-grasp task, run the stall-close,
  // then plan+execute the carry-and-place task. report_phase names each step for
  // action feedback. object_id must already be in the planning scene;
  // half_height_m sizes the tcp->grasp transform.
  // NOLINTNEXTLINE(build/include_what_you_use) — 'sort' is our method, not std::sort
  SkillResult sort(const std::string& object_id, double half_height_m,
                   const geometry_msgs::msg::PoseStamped& destination,
                   const std::function<void(const std::string&)>& report_phase);

  // Interrupt planning of the in-flight task. Execution of a planned solution
  // runs to completion — the installed Task::execute exposes no mid-execution
  // cancel.
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
