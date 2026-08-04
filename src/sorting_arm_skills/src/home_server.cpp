#include "sorting_arm_skills/home_server.hpp"

#include <exception>
#include <functional>
#include <string>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

HomeServerNode::HomeServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SkillState& state)
    : node_(std::move(node)), motion_(motion), state_(state) {
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

SkillResult HomeServerNode::home(std::stop_token stop_token, std::shared_ptr<Home::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle) {
  feedback->phase = "named_motion";
  goal_handle->publish_feedback(feedback);
  if (stop_token.stop_requested()) {
    return skill_error("named_motion", "cancellation requested");
  }

  return motion_.move_to_named("home");
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
  } catch (...) {
    result->result = to_msg(skill_error("internal", "Home worker caught an unknown exception"));
    goal_handle->abort(result);
  }
  state_.release();
}

}  // namespace sorting_arm
