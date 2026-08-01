#pragma once

#include <memory>
#include <mutex>
#include <string>

#include "control_msgs/action/gripper_command.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp_action/client.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Grasp interpretation from a GripperCommand close — "we have a usable verdict"
// and "an object was captured" are two different questions.
struct GraspOutcome {
  bool ok = false;              // a verdict exists: the controller's own result, or a direct jaw measurement
                                // after it never produced one — server-unavailable/rejected/aborted are the
                                // only cases left false, since nothing ran long enough to measure
  bool object_present = false;  // measured jaw gap and fingertip symmetry both agree with the object
  int native_code = 0;
  std::string detail;
};

// One control_msgs/action/GripperCommand client (D3), plus a /joint_states
// subscription that turns the driven knuckle angle into a physical jaw gap —
// controller stalled/reached_goal only says the action ended, never that we
// caught anything: docs/rca/gripper-grasp-instability.md.
// Blocks the calling thread — must only be called from the sequence worker,
// never the executor.
class GripperCommander {
 public:
  explicit GripperCommander(std::shared_ptr<rclcpp::Node> node);

  SkillResult open();

  // Closes to the knuckle angle for (object_width_m - squeeze_depth_m_), then
  // measures the settled jaw against object_width_m instead of trusting the
  // controller's own stalled/reached_goal verdict.
  GraspOutcome close(double object_width_m);

  // Re-measures the current jaw against object_width_m without commanding
  // anything — call after retreat to confirm the object is still held.
  GraspOutcome verify_hold(double object_width_m) const;

 private:
  using GripperCommandAction = control_msgs::action::GripperCommand;

  // one /joint_states sample, joints found by name — never by assumed index
  struct JawSample {
    double left_knuckle_rad = 0.0;
    double left_finger_tip_rad = 0.0;
    double right_finger_tip_rad = 0.0;
    bool complete = false;
  };

  // Sends one goal and blocks up to result_timeout_s_ for a terminal result.
  // Cancels the goal itself on any timeout — nothing is left driving the joint.
  SkillResult send_goal(double position, const std::string& phase, GripperCommandAction::Result& result);

  void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
  GraspOutcome evaluate_capture(double object_width_m) const;

  std::shared_ptr<rclcpp::Node> node_;
  std::string action_name_;
  double open_position_ = 0.0;
  double squeeze_depth_m_ = 0.0;
  double capture_tolerance_m_ = 0.0;
  double symmetry_tolerance_rad_ = 0.0;
  double max_effort_ = 0.0;
  double goal_timeout_s_ = 0.0;
  double result_timeout_s_ = 0.0;

  rclcpp_action::Client<GripperCommandAction>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;

  mutable std::mutex jaw_sample_mutex_;
  JawSample latest_jaw_sample_;
};

}  // namespace sorting_arm
