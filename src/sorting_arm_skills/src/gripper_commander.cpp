#include "sorting_arm_skills/gripper_commander.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <stdexcept>
#include <string>
#include <utility>

#include "rclcpp_action/create_client.hpp"

namespace sorting_arm {

// these values match sorting_arm.srdf and the driven joint's URDF limit;
// they describe our robot, not an object's width
static constexpr double kOpenPosition = 0.0;
static constexpr double kClosePosition = 0.8;
static constexpr double kMaxEffort = 50.0;

GripperCommander::GripperCommander(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  goal_timeout_s_ = node_->declare_parameter<double>("gripper.goal_timeout_s", 5.0);
  result_timeout_s_ = node_->declare_parameter<double>("gripper.result_timeout_s", 10.0);

  if (!std::isfinite(goal_timeout_s_) || goal_timeout_s_ <= 0.0) {
    throw std::runtime_error("gripper.goal_timeout_s must be positive");
  }
  if (!std::isfinite(result_timeout_s_) || result_timeout_s_ <= 0.0) {
    throw std::runtime_error("gripper.result_timeout_s must be positive");
  }

  client_ = rclcpp_action::create_client<GripperCommandAction>(node_, "/gripper_controller/gripper_cmd");
}

SkillResult GripperCommander::send_goal(double position, const std::string& phase, GripperCommandAction::Result& result, int& native_code) {
  native_code = 0;
  const auto goal_timeout = std::chrono::duration<double>(goal_timeout_s_);
  const auto result_timeout = std::chrono::duration<double>(result_timeout_s_);

  if (!client_->wait_for_action_server(goal_timeout)) {
    return skill_error(phase, "gripper action server '/gripper_controller/gripper_cmd' unavailable");
  }

  GripperCommandAction::Goal goal;
  goal.command.position = position;
  goal.command.max_effort = kMaxEffort;

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
    const auto cancel_future = client_->async_cancel_goal(goal_handle);
    if (cancel_future.wait_for(goal_timeout) != std::future_status::ready) {
      return skill_error(phase, "gripper result timed out and cancellation response timed out");
    }
    const auto cancel_response = cancel_future.get();
    if (cancel_response == nullptr || cancel_response->goals_canceling.empty()) {
      return skill_error(phase, "gripper result timed out and cancellation was not acknowledged");
    }
    return skill_error(phase, "gripper result timed out; goal canceled");
  }

  const auto wrapped = result_future.get();
  native_code = static_cast<int>(wrapped.code);
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error(phase, "gripper goal did not succeed (aborted or canceled)", native_code);
  }
  if (wrapped.result == nullptr) {
    return skill_error(phase, "gripper action returned no result message", native_code);
  }

  result = *wrapped.result;
  return skill_ok(phase);
}

SkillResult GripperCommander::open() {
  GripperCommandAction::Result result;
  int native_code = 0;
  const auto command_result = send_goal(kOpenPosition, "open_gripper", result, native_code);
  if (!command_result.ok) {
    return command_result;
  }
  if (!result.reached_goal || result.stalled) {
    return skill_error("open_gripper", "gripper did not reach the open position", native_code);
  }
  return skill_ok("open_gripper");
}

SkillResult GripperCommander::close() {
  GripperCommandAction::Result result;
  int native_code = 0;
  const auto command_result = send_goal(kClosePosition, "close_gripper", result, native_code);
  if (!command_result.ok) {
    return command_result;
  }
  RCLCPP_INFO(node_->get_logger(), "close_gripper result: position=%.6f effort=%.6f stalled=%d reached_goal=%d", result.position, result.effort,
              result.stalled ? 1 : 0, result.reached_goal ? 1 : 0);
  if (result.stalled && !result.reached_goal) {
    return skill_ok("close_gripper");
  }
  if (result.reached_goal && !result.stalled) {
    return skill_error("close_gripper", "gripper reached the fully closed position; no object was captured", native_code);
  }
  return skill_error("close_gripper", "gripper returned contradictory stalled/reached_goal flags", native_code);
}

}  // namespace sorting_arm
