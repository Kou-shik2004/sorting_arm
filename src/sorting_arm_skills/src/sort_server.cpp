#include "sorting_arm_skills/sort_server.hpp"

#include <exception>
#include <functional>
#include <string>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

SortServerNode::SortServerNode(rclcpp::Node::SharedPtr node, MtcPickPlace& planner, SceneManager& scene,
                               SkillState& state)
    : node_(std::move(node)), planner_(planner), scene_(scene), state_(state) {
  server_ = rclcpp_action::create_server<Sort>(node_, "sort", std::bind_front(&SortServerNode::handle_goal, this),
                                               std::bind_front(&SortServerNode::handle_cancel, this),
                                               std::bind_front(&SortServerNode::handle_accepted, this));
}

rclcpp_action::GoalResponse SortServerNode::handle_goal(const rclcpp_action::GoalUUID&,
                                                        std::shared_ptr<const Sort::Goal>) {
  if (!state_.try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Sort goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse SortServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Sort goal");
  planner_.cancel();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void SortServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  worker_ = std::jthread([this, goal_handle] { run(goal_handle); });
}

SkillResult SortServerNode::sort_object(const std::string& object_id, double half_height_m,
                                        const geometry_msgs::msg::PoseStamped& destination,
                                        std::shared_ptr<Sort::Feedback> feedback,
                                        std::shared_ptr<GoalHandle> goal_handle) {
  const auto report_phase = [&feedback, &goal_handle](const std::string& phase) {
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
  };
  return planner_.sort(object_id, half_height_m, destination, report_phase);
}

void SortServerNode::run(std::shared_ptr<GoalHandle> goal_handle) {
  auto result = std::make_shared<Sort::Result>();
  try {
    const auto goal = goal_handle->get_goal();
    const auto geometry = scene_.known_object_geometry(goal->object_id);
    if (!geometry) {
      result->result = to_msg(skill_error("validate", "object '" + goal->object_id + "' was never synced into the scene"));
      goal_handle->abort(result);
    } else {
      auto feedback = std::make_shared<Sort::Feedback>();
      const auto outcome =
          sort_object(goal->object_id, geometry->half_height_m, goal->destination, feedback, goal_handle);
      result->result = to_msg(outcome);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else if (outcome.ok) {
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    }
  } catch (const std::exception& error) {
    result->result = to_msg(skill_error("internal", "Sort worker exception: " + std::string(error.what())));
    goal_handle->abort(result);
  }
  state_.release();
}

}  // namespace sorting_arm
