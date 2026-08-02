#include "sorting_arm_skills/gripper_commander.hpp"

#include <chrono>
#include <cmath>
#include <future>
#include <mutex>
#include <stdexcept>
#include <string>

#include "rclcpp_action/create_client.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

namespace {
constexpr char kLeftKnuckleJoint[] = "robotiq_85_left_knuckle_joint";
constexpr char kLeftFingerTipJoint[] = "robotiq_85_left_finger_tip_joint";
constexpr char kRightFingerTipJoint[] = "robotiq_85_right_finger_tip_joint";
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
  if (result_future.wait_for(result_timeout) != std::future_status::ready) {
    // give up on our end, but the controller keeps driving the joint unless we say otherwise
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
    }
    return GraspOutcome{false, false, sent.native_code, sent.message};
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
