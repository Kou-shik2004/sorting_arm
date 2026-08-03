#pragma once

#include <memory>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// One control_msgs/action/GripperCommand client (D3). Open and close use the
// robot's fixed SRDF endpoints; the controller's native result decides success.
// Blocks the calling thread — must only be called from the sequence worker,
// never the executor.
class GripperCommander {
 public:
  explicit GripperCommander(rclcpp::Node::SharedPtr node);

  SkillResult open();
  SkillResult close();

 private:
  using GripperCommandAction = control_msgs::action::GripperCommand;

  // A result timeout requests cancellation and waits for its response so an
  // unobserved goal is not left active.
  SkillResult send_goal(double position, const std::string& phase, GripperCommandAction::Result& result, int& native_code);

  rclcpp::Node::SharedPtr node_;
  double goal_timeout_s_ = 0.0;
  double result_timeout_s_ = 0.0;

  rclcpp_action::Client<GripperCommandAction>::SharedPtr client_;
};

}  // namespace sorting_arm
