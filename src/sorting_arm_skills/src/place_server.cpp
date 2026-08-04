#include "sorting_arm_skills/place_server.hpp"

#include <functional>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

PlaceServerNode::PlaceServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SceneManager& scene, GripperCommander& gripper,
                                 SkillState& state)
    : node_(std::move(node)), motion_(motion), scene_(scene), gripper_(gripper), state_(state) {
  // Same three targets.* Pick already declared on this node — declare_or_get
  // returns the existing value instead of throwing a second declare.
  approach_height_m_ = declare_or_get<double>(*node_, "targets.approach_height_m", 0.12);
  retreat_height_m_ = declare_or_get<double>(*node_, "targets.retreat_height_m", 0.12);
  grasp_offset_m_ = declare_or_get<double>(*node_, "targets.grasp_offset_m", -0.036);

  require_positive_parameter("targets.approach_height_m", approach_height_m_);
  require_positive_parameter("targets.retreat_height_m", retreat_height_m_);
  require_finite_parameter("targets.grasp_offset_m", grasp_offset_m_);

  server_ = rclcpp_action::create_server<Place>(node_, "place", std::bind_front(&PlaceServerNode::handle_goal, this),
                                                std::bind_front(&PlaceServerNode::handle_cancel, this),
                                                std::bind_front(&PlaceServerNode::handle_accepted, this));
}

rclcpp_action::GoalResponse PlaceServerNode::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Place::Goal>) {
  if (!state_.try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Place goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlaceServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Place goal");
  worker_.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlaceServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult PlaceServerNode::place(const std::string& object_id, const geometry_msgs::msg::PoseStamped& destination_centre, double half_height_m,
                                   std::stop_token stop_token, std::shared_ptr<Place::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle) {
  auto enter_phase = [&](const std::string& phase) {
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
    return stop_token.stop_requested();
  };

  if (enter_phase("pre_place")) {
    return skill_error("pre_place", "cancellation requested");
  }
  const auto place = place_pose(destination_centre, half_height_m, grasp_offset_m_);
  const auto pre_place = pre_place_pose(place, approach_height_m_);
  const auto pre_place_result = motion_.move_to_pose(pre_place);
  if (!pre_place_result.ok) {
    return pre_place_result;
  }

  if (enter_phase("descend")) {
    return skill_error("descend", "cancellation requested");
  }
  const auto descend_result = motion_.move_cartesian_to(place);
  if (!descend_result.ok) {
    return descend_result;
  }

  if (enter_phase("open_for_place")) {
    return skill_error("open_for_place", "cancellation requested");
  }
  const auto open_result = gripper_.open();
  if (!open_result.ok) {
    return open_result;
  }

  // no cancellation check between open and detach — releasing the scene
  // attachment before the fingers open lets MoveIt plan through a held object
  feedback->phase = "detach_reinsert";
  goal_handle->publish_feedback(feedback);
  const auto detach_result = scene_.detach_and_place(object_id, destination_centre);
  if (!detach_result.ok) {
    return detach_result;
  }

  if (enter_phase("retreat")) {
    return skill_error("retreat", "cancellation requested");
  }
  const auto retreat = retreat_pose(place, retreat_height_m_);
  const auto retreat_result = motion_.move_cartesian_to(retreat);
  if (!retreat_result.ok) {
    return retreat_result;
  }

  return skill_ok("retreat");
}

void PlaceServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  auto result = std::make_shared<Place::Result>();
  try {
    const auto goal = goal_handle->get_goal();
    if (goal == nullptr) {
      result->result = to_msg(skill_error("internal", "Place goal handle returned no goal"));
      goal_handle->abort(result);
    } else {
      const auto attached = state_.attached_object();
      if (!attached || *attached != goal->object_id) {
        result->result = to_msg(skill_error("validate", "object '" + goal->object_id + "' is not the currently attached object"));
        goal_handle->abort(result);
      } else {
        const auto geometry = scene_.known_object_geometry(goal->object_id);
        if (!geometry) {
          result->result = to_msg(skill_error("validate", "object '" + goal->object_id + "' has no synced geometry"));
          goal_handle->abort(result);
        } else {
          auto feedback = std::make_shared<Place::Feedback>();
          const auto outcome = place(goal->object_id, goal->destination, geometry->half_height_m, stop_token, feedback, goal_handle);
          result->result = to_msg(outcome);

          if (goal_handle->is_canceling()) {
            goal_handle->canceled(result);
          } else if (outcome.ok) {
            state_.set_attached_object(std::nullopt);
            goal_handle->succeed(result);
          } else {
            goal_handle->abort(result);
          }
        }
      }
    }
  } catch (const std::exception& error) {
    result->result = to_msg(skill_error("internal", "Place worker exception: " + std::string(error.what())));
    goal_handle->abort(result);
  } catch (...) {
    result->result = to_msg(skill_error("internal", "Place worker caught an unknown exception"));
    goal_handle->abort(result);
  }
  state_.release();
}

}  // namespace sorting_arm
