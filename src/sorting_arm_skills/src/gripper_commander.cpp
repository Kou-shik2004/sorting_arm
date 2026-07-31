#include "sorting_arm_skills/gripper_commander.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <string>

#include "rclcpp_action/create_client.hpp"

namespace sorting_arm {

GripperCommander::GripperCommander(std::shared_ptr<rclcpp::Node> node) {
  node_ = node;

  action_name_ = node_->declare_parameter<std::string>("gripper.action_name", "/gripper_controller/gripper_cmd");
  open_position_ = node_->declare_parameter<double>("gripper.open_position", 0.0);
  close_position_ = node_->declare_parameter<double>("gripper.close_position", 0.8);
  max_effort_ = node_->declare_parameter<double>("gripper.max_effort", 40.0);
  goal_timeout_s_ = node_->declare_parameter<double>("gripper.goal_timeout_s", 5.0);
  result_timeout_s_ = node_->declare_parameter<double>("gripper.result_timeout_s", 10.0);

  if (close_position_ == open_position_) {
    throw std::runtime_error("gripper.close_position must differ from gripper.open_position");
  }
  if (max_effort_ <= 0.0) {
    throw std::runtime_error("gripper.max_effort must be positive");
  }

  client_ = rclcpp_action::create_client<GripperCommandAction>(node_, action_name_);
}

SkillResult GripperCommander::send_goal(double position, const std::string& phase,
                                        GripperCommandAction::Result& result) {
  const auto goal_timeout = std::chrono::duration<double>(goal_timeout_s_);
  const auto result_timeout = std::chrono::duration<double>(result_timeout_s_);

  if (!client_->wait_for_action_server(goal_timeout)) {
    return skill_error(phase, "gripper action server '" + action_name_ + "' unavailable");
  }

  GripperCommandAction::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = max_effort_;

  auto goal_handle_future = client_->async_send_goal(goal);

  if (goal_handle_future.wait_for(goal_timeout) != std::future_status::ready) {
    return skill_error(phase, "gripper goal response timed out");
  }
  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return skill_error(phase, "gripper goal was rejected");
  }

  auto result_future = client_->async_get_result(goal_handle);
  if (result_future.wait_for(result_timeout) != std::future_status::ready) {
    // give up on our end, but the controller keeps driving the joint unless we say otherwise
    client_->async_cancel_goal(goal_handle);
    return skill_error(phase, "gripper result timed out");
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error(phase, "gripper goal did not succeed (aborted or canceled)", static_cast<int>(wrapped.code));
  }

  result = *wrapped.result;
  return skill_ok();
}

SkillResult GripperCommander::open() {
  GripperCommandAction::Result result;
  return send_goal(open_position_, "open_gripper", result);
}

GraspOutcome GripperCommander::close() {
  GripperCommandAction::Result result;
  const auto sent = send_goal(close_position_, "close_gripper", result);
  if (!sent.ok) {
    return GraspOutcome{false, false, sent.native_code, sent.message};
  }
  // allow_stalling lets a real jam come back SUCCEEDED with stalled==true
  // instead of ABORTED, so this is the controller's own verdict, not ours
  if (result.stalled) {
    return GraspOutcome{true, true, 0, "stalled at position " + std::to_string(result.position)};
  }
  return GraspOutcome{true, false, 0, "reached close_position uncontested: no object captured"};
}

}  // namespace sorting_arm
