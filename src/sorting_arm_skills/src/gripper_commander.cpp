#include "sorting_arm_skills/gripper_commander.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
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
static constexpr double kGoalTolerance = 0.01;

static bool valid_measured_position(double position) {
  return std::isfinite(position) && position >= kOpenPosition - kGoalTolerance &&
         position <= kClosePosition + kGoalTolerance;
}

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
    if (cancel_response == nullptr || cancel_response->goals_canceling.empty()) {
      return skill_error(phase, "gripper result timed out and cancellation was not acknowledged");
    }
    return skill_error(phase, "gripper result timed out; goal canceled");
  }

  const auto wrapped = result_future.get();
  outcome.code = wrapped.code;
  if (wrapped.result == nullptr) {
    return skill_error(phase, "gripper action returned no result message", native_code(outcome.code));
  }

  outcome.result = *wrapped.result;
  return skill_ok(phase);
}

SkillResult GripperCommander::open() {
  measured_position_.reset();
  held_position_.reset();

  CommandOutcome outcome;
  const auto command_result = send_goal(kOpenPosition, "open_gripper", outcome);
  if (!command_result.ok) {
    return command_result;
  }
  if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error("open_gripper", "gripper open goal finished with a non-success action result",
                       native_code(outcome.code));
  }
  if (!outcome.result.reached_goal || outcome.result.stalled) {
    return skill_error("open_gripper", "gripper did not reach the open position", native_code(outcome.code));
  }
  if (!valid_measured_position(outcome.result.position)) {
    return skill_error("open_gripper", "gripper returned an invalid measured position", native_code(outcome.code));
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
  if (!outcome.result.reached_goal || outcome.result.stalled) {
    return skill_error(phase, "gripper did not reach the measured-position hold target", native_code(outcome.code));
  }
  if (!valid_measured_position(outcome.result.position)) {
    return skill_error(phase, "gripper hold returned an invalid measured position", native_code(outcome.code));
  }

  measured_position = outcome.result.position;
  return skill_ok(phase);
}

SkillResult GripperCommander::close() {
  held_position_.reset();

  const double minimum_progress = close_step_rad_ - kGoalTolerance;
  const auto max_steps = static_cast<std::size_t>(std::ceil((kClosePosition - kOpenPosition) / minimum_progress)) + 1;

  for (std::size_t step_index = 1; step_index <= max_steps; ++step_index) {
    const double previous_position = *measured_position_;
    const double target = std::min(previous_position + close_step_rad_, kClosePosition);
    if (target <= previous_position) {
      return skill_error("close_gripper", "fresh measured position left no forward close step");
    }

    CommandOutcome outcome;
    const auto command_result = send_goal(target, "close_gripper", outcome);
    if (!command_result.ok) {
      return command_result;
    }
    if (!valid_measured_position(outcome.result.position)) {
      return skill_error("close_gripper", "gripper returned an invalid measured position", native_code(outcome.code));
    }
    if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
      if (outcome.result.stalled) {
        return skill_error("close_gripper",
                           "controller returned a stalled goal as non-success; allow_stalling=true contract is not active",
                           native_code(outcome.code));
      }
      return skill_error("close_gripper", "gripper close step finished with a non-success action result",
                         native_code(outcome.code));
    }

    if (outcome.result.reached_goal && !outcome.result.stalled) {
      if (outcome.result.position <= previous_position) {
        return skill_error("close_gripper", "gripper reported reached_goal without forward measured motion",
                           native_code(outcome.code));
      }
      measured_position_ = outcome.result.position;
      if (target == kClosePosition) {
        return skill_error("close_gripper", "gripper reached the fully closed position; no obstruction was detected",
                           native_code(outcome.code));
      }
      continue;
    }

    if (outcome.result.stalled && !outcome.result.reached_goal) {
      if (outcome.result.position < previous_position) {
        return skill_error("close_gripper", "gripper stalled after moving opposite the close direction",
                           native_code(outcome.code));
      }

      double held_position = 0.0;
      const auto hold_result = hold_position(outcome.result.position, "close_gripper_hold", held_position);
      if (!hold_result.ok) {
        return hold_result;
      }
      measured_position_ = held_position;
      held_position_ = held_position;
      RCLCPP_INFO(node_->get_logger(),
                  "close obstruction candidate: step=%zu target=%.6f stalled_position=%.6f hold_position=%.6f", step_index,
                  target, outcome.result.position, held_position);
      return skill_ok("close_gripper");
    }

    return skill_error("close_gripper", "gripper returned contradictory stalled/reached_goal flags",
                       native_code(outcome.code));
  }

  return skill_error("close_gripper", "bounded close exhausted its step limit without a terminal result");
}

SkillResult GripperCommander::probe_grasp_retention() {
  const double probe_target = std::min(*held_position_ + close_step_rad_, kClosePosition);
  if (probe_target - *held_position_ <= kGoalTolerance) {
    return skill_error("probe_grasp_retention", "held position is too close to the joint limit for a distinguishable probe");
  }

  CommandOutcome outcome;
  const auto command_result = send_goal(probe_target, "probe_grasp_retention", outcome);
  if (!command_result.ok) {
    return command_result;
  }
  if (!valid_measured_position(outcome.result.position)) {
    return skill_error("probe_grasp_retention", "gripper probe returned an invalid measured position",
                       native_code(outcome.code));
  }
  if (outcome.code != rclcpp_action::ResultCode::SUCCEEDED) {
    if (outcome.result.stalled) {
      return skill_error("probe_grasp_retention",
                         "controller returned a stalled probe as non-success; allow_stalling=true contract is not active",
                         native_code(outcome.code));
    }
    return skill_error("probe_grasp_retention", "gripper probe finished with a non-success action result",
                       native_code(outcome.code));
  }
  if (outcome.result.reached_goal && !outcome.result.stalled) {
    return skill_error("probe_grasp_retention",
                       "gripper reached the retention probe target; grasp is likely empty or dropped",
                       native_code(outcome.code));
  }
  if (!outcome.result.stalled || outcome.result.reached_goal) {
    return skill_error("probe_grasp_retention", "gripper probe returned contradictory stalled/reached_goal flags",
                       native_code(outcome.code));
  }

  double held_position = 0.0;
  const auto hold_result = hold_position(outcome.result.position, "probe_grasp_hold", held_position);
  if (!hold_result.ok) {
    return hold_result;
  }
  measured_position_ = held_position;
  held_position_ = held_position;
  RCLCPP_INFO(node_->get_logger(), "retention obstruction candidate: target=%.6f stalled_position=%.6f hold_position=%.6f",
              probe_target, outcome.result.position, held_position);
  return skill_ok("probe_grasp_retention");
}

}  // namespace sorting_arm
