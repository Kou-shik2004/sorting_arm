#pragma once

#include <memory>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Grasp interpretation from a GripperCommand close — "the action completed" and
// "an object was captured" are two different questions.
struct GraspOutcome {
  bool ok = false;              // action completed: no timeout, rejection, or abort
  bool object_present = false;  // stalled==true before reaching the fully-closed target
  int native_code = 0;
  std::string detail;
};

// One control_msgs/action/GripperCommand client (D3). Blocks the calling
// thread — must only be called from the sequence worker, never the executor.
class GripperCommander {
 public:
  explicit GripperCommander(std::shared_ptr<rclcpp::Node> node);

  SkillResult open();
  GraspOutcome close();

 private:
  using GripperCommandAction = control_msgs::action::GripperCommand;

  // Sends one goal (open or close target) and blocks the calling thread until
  // a terminal result or a configured timeout elapses. `result` is only valid
  // when the returned SkillResult is ok. `phase` names the failure if it
  // doesn't — "open_gripper" or "close_gripper", not always the latter.
  SkillResult send_and_wait(double position, const std::string& phase, GripperCommandAction::Result& result);

  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  double open_position_ = 0.0;
  double close_position_ = 0.0;
  double max_effort_ = 0.0;
  double goal_timeout_s_ = 0.0;
  double result_timeout_s_ = 0.0;
  double close_target_tolerance_ = 0.0;

  rclcpp_action::Client<GripperCommandAction>::SharedPtr client_;
};

}  // namespace sorting_arm
