#pragma once

#include <memory>
#include <optional>
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
  SkillResult probe_grasp_retention();

 private:
  using GripperCommandAction = control_msgs::action::GripperCommand;

  struct CommandOutcome {
    rclcpp_action::ResultCode code = rclcpp_action::ResultCode::UNKNOWN;
    GripperCommandAction::Result result;
  };

  // A result timeout requests cancellation and waits for its response so an
  // unobserved goal is not left active.
  SkillResult send_goal(double position, const std::string& phase, CommandOutcome& outcome);
  SkillResult hold_position(double position, const std::string& phase, double& measured_position);

  rclcpp::Node::SharedPtr node_;
  double goal_timeout_s_ = 0.0;
  double result_timeout_s_ = 0.0;
  double close_step_rad_ = 0.0;

  std::optional<double> measured_position_;
  std::optional<double> held_position_;

  rclcpp_action::Client<GripperCommandAction>::SharedPtr client_;
};

}  // namespace sorting_arm
