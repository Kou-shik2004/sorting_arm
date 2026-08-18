#include "sorting_arm_skills/gripper_commander.hpp"

#include <algorithm>
#include <chrono>
#include <cstddef>
#include <future>
#include <string>
#include <utility>

#include "rclcpp_action/create_client.hpp"

namespace sorting_arm {

// these values match sorting_arm.srdf and the driven joint's URDF limit;
// they describe our robot, not an object's width

static constexpr double kOpenPosition = 0.0;
static constexpr double kClosePosition = 0.8;
static constexpr double kMaxEffort = 50.0;

static int native_code(rclcpp_action::ResultCode code) { return static_cast<int>(code); }

GripperCommander::GripperCommander(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  goal_timeout_s_ = node_->declare_parameter<double>("gripper.goal_timeout_s", 5.0);
  result_timeout_s_ = node_->declare_parameter<double>("gripper.result_timeout_s", 10.0);
  close_step_rad_ = node_->declare_parameter<double>("gripper.close_step_rad", 0.03);

  client_ = rclcpp_action::create_client<GripperCommandAction>(node_, "/gripper_controller/gripper_cmd");
}

SkillResult GripperCommander::send_goal(double position, const std::string& phase, CommandOutcome& outcome) {
  const auto goal_timeout = std::chrono::duration<double>(goal_timeout_s_);
  const auto result_timeout = std::chrono::duration<double>(result_timeout_s_);

  if (!client_->wait_for_action_server(goal_timeout)) {
    return skill_error(phase, "gripper action server '/gripper_controller/gripper_cmd' unavailable");
  }

  GripperCommandAction::Goal goal;
  goal.command.position = position;
  // our position-only adapter ignores effort, but GripperCommand still owns this field
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
    if (cancel_response->goals_canceling.empty()) {
      return skill_error(phase, "gripper result timed out and cancellation was not acknowledged");
    }
    return skill_error(phase, "gripper result timed out; goal canceled");
  }

  const auto wrapped = result_future.get();
  outcome.code = wrapped.code;
  outcome.result = *wrapped.result;
  return skill_ok(phase);
}

SkillResult GripperCommander::open() {
  measured_position_.reset();

  CommandOutcome outcome;
  const auto command_result = send_goal(kOpenPosition, "open_gripper", outcome);
  if (!command_result.ok) {
    return command_result;
  }
  if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error("open_gripper", "gripper open goal finished with a non-success action result",
                       native_code(outcome.code));
  }
  if (!outcome.result.reached_goal) {
    return skill_error("open_gripper", "gripper did not reach the open position", native_code(outcome.code));
  }

  measured_position_ = outcome.result.position;
  return skill_ok("open_gripper");
}

SkillResult GripperCommander::hold_position(double position, const std::string& phase, double& measured_position) {
  CommandOutcome outcome;
  const auto command_result = send_goal(position, phase, outcome);
  if (!command_result.ok) {
    return command_result;
  }
  if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error(phase, "measured-position hold finished with a non-success action result", native_code(outcome.code));
  }
  if (!outcome.result.reached_goal) {
    return skill_error(phase, "gripper did not reach the measured-position hold target", native_code(outcome.code));
  }

  measured_position = outcome.result.position;
  return skill_ok(phase);
}

SkillResult GripperCommander::close() {
  // steps monotonically toward kClosePosition, so the loop always exits: it either
  // stalls on the object or reaches full close (handled below), never runs forever.
  std::size_t step_index = 0;
  while (true) {
    ++step_index;
    const double previous_position = *measured_position_;
    const double target = std::min(previous_position + close_step_rad_, kClosePosition);

    CommandOutcome outcome;
    const auto command_result = send_goal(target, "close_gripper", outcome);
    if (!command_result.ok) {
      return command_result;
    }
    if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
      return skill_error("close_gripper", "gripper close step finished with a non-success action result",
                         native_code(outcome.code));
    }

    if (outcome.result.reached_goal) {
      measured_position_ = outcome.result.position;
      if (target == kClosePosition) {
        return skill_error("close_gripper", "gripper reached the fully closed position; no obstruction was detected",
                           native_code(outcome.code));
      }
      continue;
    }

    // not reached_goal -> the gripper stalled against an obstruction; hold there
    double held_position = 0.0;
    const auto hold_result = hold_position(outcome.result.position, "close_gripper_hold", held_position);
    if (!hold_result.ok) {
      return hold_result;
    }
    measured_position_ = held_position;
    RCLCPP_INFO(node_->get_logger(),
                "close obstruction candidate: step=%zu target=%.6f stalled_position=%.6f hold_position=%.6f", step_index,
                target, outcome.result.position, held_position);
    return skill_ok("close_gripper");
  }
}

}  // namespace sorting_arm
