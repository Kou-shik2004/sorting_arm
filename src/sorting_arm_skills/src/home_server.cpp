#include "sorting_arm_skills/home_server.hpp"

#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

using namespace std::placeholders;

HomeServerNode::HomeServerNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MotionCommander> motion,
                               std::shared_ptr<SkillState> state)
    : node_(std::move(node)), motion_(std::move(motion)), state_(std::move(state)) {
  home_named_target_ = declare_or_get<std::string>(*node_, "targets.home_named_target", "home");

  server_ =
      rclcpp_action::create_server<Home>(node_, "home", std::bind(&HomeServerNode::handle_goal, this, _1, _2),
                                         std::bind(&HomeServerNode::handle_cancel, this, _1),
                                         std::bind(&HomeServerNode::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse HomeServerNode::handle_goal(const rclcpp_action::GoalUUID&,
                                                        std::shared_ptr<const Home::Goal>) {
  if (!state_->try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Home goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse HomeServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Home goal");
  state_->request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void HomeServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  state_->start_worker([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult HomeServerNode::home(std::stop_token stop_token, std::shared_ptr<Home::Feedback> feedback,
                                 std::shared_ptr<GoalHandle> goal_handle) {
  feedback->phase = "named_motion";
  goal_handle->publish_feedback(feedback);
  if (stop_token.stop_requested()) return skill_error("named_motion", "cancellation requested");

  return motion_->move_to_named(home_named_target_);
}

void HomeServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  auto feedback = std::make_shared<Home::Feedback>();
  auto result = std::make_shared<Home::Result>();

  const auto outcome = home(stop_token, feedback, goal_handle);
  result->result = to_msg(outcome);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (outcome.ok) {
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  state_->release();
}

}  // namespace sorting_arm
