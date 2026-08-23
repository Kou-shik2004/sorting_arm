#include "sorting_arm_skills/home_server.hpp"

#include <exception>
#include <functional>
#include <string>
#include <utility>

namespace sorting_arm {

HomeServerNode::HomeServerNode(rclcpp::Node::SharedPtr node, SkillState& state)
    : node_(std::move(node)), state_(state), arm_(node_, "arm") {
  arm_.setPoseReferenceFrame("world");
  arm_.setPlanningTime(declare_or_get<double>(*node_, "planning.planning_time_s", 5.0));
  arm_.setNumPlanningAttempts(static_cast<unsigned int>(declare_or_get<int>(*node_, "planning.planning_attempts", 5)));
  arm_.setMaxVelocityScalingFactor(declare_or_get<double>(*node_, "planning.velocity_scaling", 0.15));
  arm_.setMaxAccelerationScalingFactor(declare_or_get<double>(*node_, "planning.acceleration_scaling", 0.15));

  server_ = rclcpp_action::create_server<Home>(node_, "home", std::bind_front(&HomeServerNode::handle_goal, this),
                                               std::bind_front(&HomeServerNode::handle_cancel, this),
                                               std::bind_front(&HomeServerNode::handle_accepted, this));
}

rclcpp_action::GoalResponse HomeServerNode::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Home::Goal>) {
  if (!state_.try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Home goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HomeServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Home goal");
  worker_.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HomeServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult HomeServerNode::home(std::stop_token stop_token, std::shared_ptr<Home::Feedback> feedback,
                                 std::shared_ptr<GoalHandle> goal_handle) {
  feedback->phase = "named_motion";
  goal_handle->publish_feedback(feedback);
  if (stop_token.stop_requested()) {
    return skill_error("named_motion", "cancellation requested");
  }

  if (!arm_.setNamedTarget("home")) {
    return skill_error("named_motion", "unknown named target 'home'");
  }
  arm_.setStartStateToCurrentState();

  moveit::planning_interface::MoveGroupInterface::Plan plan;
  const auto plan_result = arm_.plan(plan);
  if (!plan_result) {
    return skill_error("named_motion", "named target planning failed", plan_result.val);
  }

  const auto exec_result = arm_.execute(plan);
  if (!exec_result) {
    return skill_error("named_motion", "named target execution failed", exec_result.val);
  }
  return skill_ok("named_motion");
}

void HomeServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  auto feedback = std::make_shared<Home::Feedback>();
  auto result = std::make_shared<Home::Result>();
  try {
    const auto outcome = home(stop_token, feedback, goal_handle);
    result->result = to_msg(outcome);

    if (goal_handle->is_canceling()) {
      goal_handle->canceled(result);
    } else if (outcome.ok) {
      goal_handle->succeed(result);
    } else {
      goal_handle->abort(result);
    }
  } catch (const std::exception& error) {
    result->result = to_msg(skill_error("internal", "Home worker exception: " + std::string(error.what())));
    goal_handle->abort(result);
  }
  state_.release();
}

}  // namespace sorting_arm
