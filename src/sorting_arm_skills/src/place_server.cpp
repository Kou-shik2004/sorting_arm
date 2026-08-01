#include "sorting_arm_skills/place_server.hpp"

#include <mutex>
#include <stdexcept>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

using namespace std::placeholders;

PlaceServerNode::PlaceServerNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MotionCommander> motion,
                                 std::shared_ptr<SceneManager> scene, std::shared_ptr<GripperCommander> gripper,
                                 std::shared_ptr<SkillState> state)
    : node_(std::move(node)), motion_(std::move(motion)), scene_(std::move(scene)), gripper_(std::move(gripper)),
      state_(std::move(state)) {
  // Same three targets.* Pick already declared on this node — declare_or_get
  // returns the existing value instead of throwing a second declare.
  approach_height_m_ = declare_or_get<double>(*node_, "targets.approach_height_m", 0.12);
  retreat_height_m_ = declare_or_get<double>(*node_, "targets.retreat_height_m", 0.12);
  grasp_offset_m_ = declare_or_get<double>(*node_, "targets.grasp_offset_m", -0.036);

  if (approach_height_m_ <= 0.0) {
    throw std::runtime_error("targets.approach_height_m must be positive");
  }
  if (retreat_height_m_ <= 0.0) {
    throw std::runtime_error("targets.retreat_height_m must be positive");
  }

  server_ =
      rclcpp_action::create_server<Place>(node_, "place", std::bind(&PlaceServerNode::handle_goal, this, _1, _2),
                                          std::bind(&PlaceServerNode::handle_cancel, this, _1),
                                          std::bind(&PlaceServerNode::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse PlaceServerNode::handle_goal(const rclcpp_action::GoalUUID&,
                                                         std::shared_ptr<const Place::Goal>) {
  if (!state_->try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Place goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PlaceServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Place goal");
  state_->request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PlaceServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  state_->start_worker([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult PlaceServerNode::place(const std::string& object_id,
                                   const geometry_msgs::msg::PoseStamped& destination_centre, double half_height_m,
                                   std::stop_token stop_token, std::shared_ptr<Place::Feedback> feedback,
                                   std::shared_ptr<GoalHandle> goal_handle) {
  auto enter_phase = [&](const std::string& phase) {
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
    return stop_token.stop_requested();
  };

  if (enter_phase("pre_place")) return skill_error("pre_place", "cancellation requested");
  const auto place = place_pose(destination_centre, half_height_m, grasp_offset_m_);
  const auto pre_place = pre_place_pose(place, approach_height_m_);
  const auto pre_place_result = motion_->move_to_pose(pre_place);
  if (!pre_place_result.ok) return pre_place_result;

  if (enter_phase("descend")) return skill_error("descend", "cancellation requested");
  const auto descend_result = motion_->move_cartesian_to(place);
  if (!descend_result.ok) return descend_result;

  if (enter_phase("open_for_place")) return skill_error("open_for_place", "cancellation requested");
  const auto open_result = gripper_->open();
  if (!open_result.ok) return open_result;

  // no cancellation check between open and detach — releasing the scene
  // attachment before the fingers open lets MoveIt plan through a held object
  feedback->phase = "detach_reinsert";
  goal_handle->publish_feedback(feedback);
  const auto detach_result = scene_->detach_and_place(object_id, destination_centre);
  if (!detach_result.ok) return detach_result;

  if (enter_phase("retreat")) return skill_error("retreat", "cancellation requested");
  const auto retreat = retreat_pose(place, retreat_height_m_);
  const auto retreat_result = motion_->move_cartesian_to(retreat);
  if (!retreat_result.ok) return retreat_result;

  return skill_ok("retreat");
}

void PlaceServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Place::Feedback>();
  auto result = std::make_shared<Place::Result>();

  // object_id must match what Pick actually attached — server-owned state,
  // checked here rather than inside the sequence itself
  const auto attached = state_->attached_object();
  if (!attached || *attached != goal->object_id) {
    result->result =
        to_msg(skill_error("validate", "object '" + goal->object_id + "' is not the currently attached object"));
    goal_handle->abort(result);
    state_->release();
    return;
  }

  const auto geometry = scene_->known_object_geometry(goal->object_id);
  const double half_height_m = geometry ? geometry->half_height_m : 0.0;

  const auto outcome = place(goal->object_id, goal->destination, half_height_m, stop_token, feedback, goal_handle);
  result->result = to_msg(outcome);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (outcome.ok) {
    state_->set_attached_object(std::nullopt);
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  state_->release();
}

}  // namespace sorting_arm
