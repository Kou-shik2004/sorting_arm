#include "sorting_arm_skills/gripper_commander.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/exceptions.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

namespace {
<<<<<<< Updated upstream
constexpr char kLeftKnuckleJoint[] = "robotiq_85_left_knuckle_joint";
constexpr char kLeftFingerTipJoint[] = "robotiq_85_left_finger_tip_joint";
constexpr char kRightFingerTipJoint[] = "robotiq_85_right_finger_tip_joint";
=======

bool recoverable_knuckle_position(double position_rad, double open_position_rad, double close_position_rad,
                                  double step_rad) {
  return std::isfinite(position_rad) && position_rad >= open_position_rad - step_rad &&
         position_rad <= close_position_rad;
}

>>>>>>> Stashed changes
}  // namespace

GripperCommander::GripperCommander(std::shared_ptr<rclcpp::Node> node) {
  node_ = node;

  action_name_ = node_->declare_parameter<std::string>("gripper.action_name", "/gripper_controller/gripper_cmd");
  open_position_ = node_->declare_parameter<double>("gripper.open_position", 0.0);
  squeeze_depth_m_ = node_->declare_parameter<double>("gripper.squeeze_depth_m", 0.003);
  capture_tolerance_m_ = node_->declare_parameter<double>("gripper.capture_tolerance_m", 0.005);
  symmetry_tolerance_rad_ = node_->declare_parameter<double>("gripper.symmetry_tolerance_rad", 0.005);
  max_effort_ = node_->declare_parameter<double>("gripper.max_effort", 40.0);
  goal_timeout_s_ = node_->declare_parameter<double>("gripper.goal_timeout_s", 5.0);
  result_timeout_s_ = node_->declare_parameter<double>("gripper.result_timeout_s", 10.0);

  if (squeeze_depth_m_ <= 0.0) {
    throw std::runtime_error("gripper.squeeze_depth_m must be positive");
  }
  if (capture_tolerance_m_ <= 0.0) {
    throw std::runtime_error("gripper.capture_tolerance_m must be positive");
  }
  if (symmetry_tolerance_rad_ <= 0.0) {
    throw std::runtime_error("gripper.symmetry_tolerance_rad must be positive");
  }
  if (max_effort_ <= 0.0) {
    throw std::runtime_error("gripper.max_effort must be positive");
  }

  client_ = rclcpp_action::create_client<GripperCommandAction>(node_, action_name_);

  // followers are state-only, gz_ros2_control's own mimic drive is what moves them
  // now (docs/rca/gripper-grasp-instability.md) — this just watches all three
  joint_state_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
      "/joint_states", 10, [this](const sensor_msgs::msg::JointState::SharedPtr msg) { joint_state_callback(msg); });
}

SkillResult GripperCommander::send_goal(double position, const std::string& phase,
                                        GripperCommandAction::Result& result,
                                        bool& timed_out_waiting_for_result) {
  timed_out_waiting_for_result = false;
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
<<<<<<< Updated upstream
  if (result_future.wait_for(result_timeout) != std::future_status::ready) {
    // give up on our end, but the controller keeps driving the joint unless we say otherwise
=======
  const auto result_deadline = node_->now() + rclcpp::Duration::from_seconds(result_timeout_s_);
  RCLCPP_INFO(node_->get_logger(), "%s: gripper goal accepted; waiting up to %.3f ROS seconds", phase.c_str(),
              result_timeout_s_);
  while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (node_->now() < result_deadline) {
      continue;
    }

    // only open uses this longer deadline now; close steps use send_step.
    const auto elapsed_wall_s =
        std::chrono::duration<double>(std::chrono::steady_clock::now() - goal_sent_wall).count();
    RCLCPP_WARN(node_->get_logger(),
                "%s: no gripper result within %.3f ROS seconds (%.2fs wall elapsed); canceling goal", phase.c_str(),
                result_timeout_s_, elapsed_wall_s);
>>>>>>> Stashed changes
    client_->async_cancel_goal(goal_handle);
    timed_out_waiting_for_result = true;
    return skill_error(phase, "gripper result timed out");
  }
  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return skill_error(phase, "gripper goal did not succeed (aborted or canceled)", static_cast<int>(wrapped.code));
  }

  result = *wrapped.result;
  return skill_ok();
}

<<<<<<< Updated upstream
=======
GripperCommander::StepOutcome GripperCommander::send_step(double measured_start_rad, double target_rad, StepMode mode) {
  const auto mode_name = mode == StepMode::must_reach ? "must_reach" : "contact_allowed";
  GripperCommandAction::Goal goal;
  goal.command.position = target_rad;
  goal.command.max_effort = max_effort_;

  RCLCPP_INFO(node_->get_logger(),
              "close_gripper: %s step measured_start=%.6f target=%.6f delta=%.6f target_gap=%.1fmm", mode_name,
              measured_start_rad, target_rad, target_rad - measured_start_rad, jaw_gap_m(target_rad) * 1000.0);

  auto goal_handle_future = client_->async_send_goal(goal);
  if (goal_handle_future.wait_for(std::chrono::duration<double>(goal_timeout_s_)) != std::future_status::ready) {
    return {StepState::failed, 0, "close ladder goal response timed out", std::nullopt};
  }
  const auto goal_handle = goal_handle_future.get();
  if (!goal_handle) {
    return {StepState::failed, 0, "close ladder goal was rejected", std::nullopt};
  }

  auto result_future = client_->async_get_result(goal_handle);
  const auto result_deadline = node_->now() + rclcpp::Duration::from_seconds(step_timeout_s_);
  while (result_future.wait_for(std::chrono::milliseconds(100)) != std::future_status::ready) {
    if (node_->now() < result_deadline) {
      continue;
    }

    if (mode == StepMode::contact_allowed) {
      // next goal preempts this goal; cancel would release the object.
      RCLCPP_INFO(node_->get_logger(),
                  "close_gripper: contact_allowed step reached its %.3f ROS-second deadline; treating as contact "
                  "and leaving goal active",
                  step_timeout_s_);
      return {StepState::contact, 0, "no step result before the ROS-time deadline", std::nullopt};
    }

    std::shared_future<GripperCommandAction::Impl::CancelGoalService::Response::SharedPtr> cancel_future;
    try {
      cancel_future = client_->async_cancel_goal(goal_handle);
    } catch (const rclcpp_action::exceptions::UnknownGoalHandleError&) {
      return {StepState::failed, 0, "free-space approach timed out after its goal became terminal", std::nullopt};
    }
    if (cancel_future.wait_for(std::chrono::duration<double>(goal_timeout_s_)) != std::future_status::ready) {
      return {StepState::failed, 0, "free-space approach timed out and cancellation response timed out", std::nullopt};
    }
    const auto cancel_response = cancel_future.get();
    using CancelResponse = GripperCommandAction::Impl::CancelGoalService::Response;
    if (cancel_response->return_code != CancelResponse::ERROR_NONE || cancel_response->goals_canceling.empty()) {
      return {StepState::failed, cancel_response->return_code,
              "free-space approach timed out and cancellation was rejected", std::nullopt};
    }
    return {StepState::failed, 0, "free-space approach step timed out before reaching its target", std::nullopt};
  }

  const auto wrapped = result_future.get();
  if (wrapped.code != rclcpp_action::ResultCode::SUCCEEDED) {
    return {StepState::failed, static_cast<int>(wrapped.code), "close ladder goal did not succeed", std::nullopt};
  }

  RCLCPP_INFO(node_->get_logger(), "close_gripper: %s step result position=%.6f gap=%.1fmm stalled=%s reached_goal=%s",
              mode_name, wrapped.result->position, jaw_gap_m(wrapped.result->position) * 1000.0,
              wrapped.result->stalled ? "true" : "false", wrapped.result->reached_goal ? "true" : "false");

  if (mode == StepMode::must_reach) {
    if (wrapped.result->stalled) {
      return {StepState::failed, 0, "free-space approach step stalled before reaching its target",
              wrapped.result->position};
    }
    if (!wrapped.result->reached_goal) {
      return {StepState::failed, 0, "free-space approach step ended without reaching its target",
              wrapped.result->position};
    }
    return {StepState::reached, 0, "free-space approach step reached its target", wrapped.result->position};
  }

  if (wrapped.result->stalled && !wrapped.result->reached_goal) {
    return {StepState::contact, 0, "close ladder step reported contact stall", wrapped.result->position};
  }
  if (!wrapped.result->reached_goal || wrapped.result->stalled) {
    return {StepState::failed, 0, "close ladder step returned inconsistent completion flags", wrapped.result->position};
  }
  return {StepState::reached, 0, "step completed", wrapped.result->position};
}

std::optional<double> GripperCommander::current_knuckle_rad() const {
  const std::lock_guard<std::mutex> lock(knuckle_mutex_);
  if (!last_knuckle_rad_) {
    return std::nullopt;
  }
  constexpr double kStaleAfterS = 1.0;
  const auto age_s = (node_->now() - last_knuckle_stamp_).seconds();
  if (age_s < 0.0 || age_s > kStaleAfterS) {
    return std::nullopt;
  }
  return last_knuckle_rad_;
}

void GripperCommander::wait_for_ros_duration(double duration_s) const {
  const auto deadline = node_->now() + rclcpp::Duration::from_seconds(duration_s);
  while (node_->now() < deadline) {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

>>>>>>> Stashed changes
SkillResult GripperCommander::open() {
  GripperCommandAction::Result result;
  bool timed_out_waiting_for_result = false;
  return send_goal(open_position_, "open_gripper", result, timed_out_waiting_for_result);
}

void GripperCommander::joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg) {
  JawSample sample;
  bool found_left_knuckle = false;
  bool found_left_tip = false;
  bool found_right_tip = false;

  for (size_t i = 0; i < msg->name.size(); ++i) {
    if (msg->name[i] == kLeftKnuckleJoint) {
      sample.left_knuckle_rad = msg->position[i];
      found_left_knuckle = true;
    } else if (msg->name[i] == kLeftFingerTipJoint) {
      sample.left_finger_tip_rad = msg->position[i];
      found_left_tip = true;
    } else if (msg->name[i] == kRightFingerTipJoint) {
      sample.right_finger_tip_rad = msg->position[i];
      found_right_tip = true;
    }
  }
  sample.complete = found_left_knuckle && found_left_tip && found_right_tip;

  const std::lock_guard<std::mutex> lock(jaw_sample_mutex_);
  latest_jaw_sample_ = sample;
}

GraspOutcome GripperCommander::evaluate_capture(double object_width_m) const {
  JawSample sample;
  {
    const std::lock_guard<std::mutex> lock(jaw_sample_mutex_);
    sample = latest_jaw_sample_;
  }
  if (!sample.complete) {
    return GraspOutcome{true, false, 0, "no complete /joint_states sample for the gripper joints yet"};
  }

<<<<<<< Updated upstream
  const double measured_gap_m = jaw_gap_m(sample.left_knuckle_rad);
  const double gap_error_m = std::abs(measured_gap_m - object_width_m);
  const double symmetry_rad = std::abs(std::abs(sample.left_finger_tip_rad) - std::abs(sample.right_finger_tip_rad));
  const bool captured = gap_error_m <= capture_tolerance_m_ && symmetry_rad <= symmetry_tolerance_rad_;

  const std::string detail = "measured gap " + std::to_string(measured_gap_m * 1000.0) + " mm vs object " +
                             std::to_string(object_width_m * 1000.0) + " mm, symmetry " + std::to_string(symmetry_rad) +
                             " rad";
  return GraspOutcome{true, captured, 0, detail};
}

GraspOutcome GripperCommander::close(double object_width_m) {
  const double target_gap_m = object_width_m - squeeze_depth_m_;
  const auto target_rad = knuckle_angle_for_gap_m(target_gap_m);
  if (!target_rad) {
    return GraspOutcome{false, false, 0,
                        "object_width_m " + std::to_string(object_width_m) +
                            " has no valid knuckle angle at squeeze_depth_m " + std::to_string(squeeze_depth_m_)};
  }

  RCLCPP_INFO(node_->get_logger(), "close_gripper: object width %.4f m, target %.4f rad (%.1f mm gap)", object_width_m,
              *target_rad, target_gap_m * 1000.0);
=======
  RCLCPP_INFO(node_->get_logger(), "close_gripper: approach target gap=%.1fmm angle=%.3f", approach_gap_m * 1000.0,
              *approach_rad);

  if (!client_->wait_for_action_server(std::chrono::duration<double>(goal_timeout_s_))) {
    return {false, false, 0, "gripper action server '" + action_name_ + "' unavailable"};
  }

  const auto measured_start_rad = current_knuckle_rad();
  if (!measured_start_rad) {
    return {false, false, 0, "no fresh /joint_states sample before the free-space approach"};
  }
  RCLCPP_INFO(node_->get_logger(), "close_gripper: measured approach start=%.9f", *measured_start_rad);
  if (!recoverable_knuckle_position(*measured_start_rad, open_position_, close_position_, close_step_rad_)) {
    return {false, false, 0,
            "free-space approach started from invalid joint position " + std::to_string(*measured_start_rad)};
  }
  if (*approach_rad > close_position_) {
    return {false, false, 0, "free-space approach target exceeds close_position"};
  }
  if (*measured_start_rad > *approach_rad) {
    return {false, false, 0, "measured knuckle position is already past the free-space approach target"};
  }

  double position_rad = *measured_start_rad;
  bool approach_complete = false;
  while (!approach_complete) {
    const auto target_rad = std::min(position_rad + close_step_rad_, *approach_rad);
    const auto final_approach_step = target_rad >= *approach_rad;
    const auto outcome = send_step(position_rad, target_rad, StepMode::must_reach);
    if (outcome.state == StepState::failed) {
      return {false, false, outcome.native_code, outcome.detail};
    }
    if (!outcome.result_position_rad ||
        !recoverable_knuckle_position(*outcome.result_position_rad, open_position_, close_position_, close_step_rad_)) {
      return {false, false, 0,
              "free-space approach returned invalid joint position " +
                  (outcome.result_position_rad ? std::to_string(*outcome.result_position_rad) : "missing")};
    }
    position_rad = *outcome.result_position_rad;
    approach_complete = final_approach_step;
  }

  while (position_rad < close_position_) {
    const auto target_rad = std::min(position_rad + close_step_rad_, close_position_);
    const auto outcome = send_step(position_rad, target_rad, StepMode::contact_allowed);
    if (outcome.state == StepState::failed) {
      return {false, false, outcome.native_code, outcome.detail};
    }
    if (outcome.state == StepState::reached && target_rad >= close_position_) {
      RCLCPP_INFO(node_->get_logger(),
                  "close_gripper: reached close_position with no contact; nothing between the pads");
      return {true, false, 0, "reached close_position uncontested: no object captured"};
    }
    if (outcome.state == StepState::contact) {
      const auto contact_rad = current_knuckle_rad();
      if (!contact_rad) {
        return {false, false, 0, "no fresh /joint_states sample at close contact"};
      }
      if (!recoverable_knuckle_position(*contact_rad, open_position_, close_position_, close_step_rad_)) {
        return {false, false, 0, "close contact returned invalid joint position " + std::to_string(*contact_rad)};
      }
>>>>>>> Stashed changes

  GripperCommandAction::Result result;
  bool timed_out_waiting_for_result = false;
  const auto sent = send_goal(*target_rad, "close_gripper", result, timed_out_waiting_for_result);
  if (!sent.ok) {
    if (timed_out_waiting_for_result) {
      // send_goal already canceled this goal, which only freezes the target at
      // wherever the joint physically was — our timeout isn't proof nothing was
      // caught, so check the jaw directly before believing the controller's silence
      const auto physical_outcome = evaluate_capture(object_width_m);
      RCLCPP_INFO(node_->get_logger(), "close_gripper: no terminal controller result, checked jaw directly — %s",
                  physical_outcome.detail.c_str());
      if (physical_outcome.object_present) {
        return physical_outcome;
      }
<<<<<<< Updated upstream
    }
    return GraspOutcome{false, false, sent.native_code, sent.message};
=======
      if (!squeeze_handle_future.get()) {
        return {false, false, 0, "squeeze goal was rejected"};
      }

      RCLCPP_INFO(node_->get_logger(),
                  "close_gripper: squeeze measured_start=%.6f target=%.6f delta=%.6f target_gap=%.1fmm left running",
                  *contact_rad, squeeze_rad, squeeze_rad - *contact_rad, jaw_gap_m(squeeze_rad) * 1000.0);
      wait_for_ros_duration(settle_s_);

      const auto settled_knuckle_rad = current_knuckle_rad();
      if (!settled_knuckle_rad) {
        return {false, false, 0, "no fresh /joint_states sample to verify the grasp"};
      }
      const auto measured_gap_m = jaw_gap_m(*settled_knuckle_rad);
      const auto captured = std::fabs(measured_gap_m - expected_width_m) <= grip_tolerance_m_;
      RCLCPP_INFO(node_->get_logger(), "close_gripper: settled gap=%.1fmm expected=%.1fmm -> %s",
                  measured_gap_m * 1000.0, expected_width_m * 1000.0, captured ? "CAPTURED" : "MISS");
      return {
          true, captured, 0,
          captured ? "captured at gap " + std::to_string(measured_gap_m) : "settled gap did not match expected width"};
    }
    if (!outcome.result_position_rad ||
        !recoverable_knuckle_position(*outcome.result_position_rad, open_position_, close_position_, close_step_rad_)) {
      return {false, false, 0,
              "close ladder returned invalid joint position " +
                  (outcome.result_position_rad ? std::to_string(*outcome.result_position_rad) : "missing")};
    }
    position_rad = *outcome.result_position_rad;
>>>>>>> Stashed changes
  }

  // stalled/reached_goal only decided whether the action ended — the jaw gap
  // and fingertip symmetry below are what decide if we actually caught the box
  const auto outcome = evaluate_capture(object_width_m);
  RCLCPP_INFO(node_->get_logger(), "close_gripper result: stalled=%d reached_goal=%d position=%.4f — %s",
              result.stalled, result.reached_goal, result.position, outcome.detail.c_str());
  return outcome;
}

GraspOutcome GripperCommander::verify_hold(double object_width_m) const { return evaluate_capture(object_width_m); }

}  // namespace sorting_arm
