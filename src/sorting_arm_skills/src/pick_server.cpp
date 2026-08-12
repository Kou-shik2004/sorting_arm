#include "sorting_arm_skills/pick_server.hpp"

#include <Eigen/Geometry>
#include <exception>
#include <functional>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"
#include "tf2_eigen/tf2_eigen.hpp"

namespace sorting_arm {

PickServerNode::PickServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SceneManager& scene,
                               GripperCommander& gripper, SkillState& state)
    : node_(std::move(node)), motion_(motion), scene_(scene), gripper_(gripper), state_(state) {
  approach_height_m_ = declare_or_get<double>(*node_, "targets.approach_height_m", 0.12);
  retreat_height_m_ = declare_or_get<double>(*node_, "targets.retreat_height_m", 0.12);
  grasp_validation_lift_m_ = declare_or_get<double>(*node_, "targets.grasp_validation_lift_m", 0.02);
  grasp_offset_m_ = declare_or_get<double>(*node_, "targets.grasp_offset_m", -0.036);
  max_pre_grasp_candidates_ = declare_or_get<int>(*node_, "targets.max_pre_grasp_candidates", 5);

  server_ = rclcpp_action::create_server<Pick>(node_, "pick", std::bind_front(&PickServerNode::handle_goal, this),
                                               std::bind_front(&PickServerNode::handle_cancel, this),
                                               std::bind_front(&PickServerNode::handle_accepted, this));
}

rclcpp_action::GoalResponse PickServerNode::handle_goal(const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal>) {
  if (!state_.try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Pick goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Pick goal");
  worker_.request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  worker_ = std::jthread([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult PickServerNode::with_grasp_contacts_disabled(const std::string& object_id,
                                                         const std::function<SkillResult()>& body) {
  const auto allow_result = scene_.begin_grasp_contacts(object_id);
  if (!allow_result.ok) {
    return allow_result;
  }

  const auto body_result = body();

  const auto restore_result = scene_.end_grasp_contacts();
  if (restore_result.ok) {
    return body_result;
  }
  if (body_result.ok) {
    return restore_result;
  }
  return skill_error("grasp_contacts", body_result.message + "; " + restore_result.message, restore_result.native_code);
}

SkillResult PickServerNode::pick(const std::string& object_id, const geometry_msgs::msg::PoseStamped& object_centre,
                                 double half_height_m, std::stop_token stop_token, std::shared_ptr<Pick::Feedback> feedback,
                                 std::shared_ptr<GoalHandle> goal_handle) {
  auto enter_phase = [&](const std::string& phase) {
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
    return stop_token.stop_requested();
  };

  if (enter_phase("open_gripper")) {
    return skill_error("open_gripper", "cancellation requested");
  }
  const auto open_result = gripper_.open();
  if (!open_result.ok) {
    return open_result;
  }

  if (enter_phase("pre_grasp")) {
    return skill_error("pre_grasp", "cancellation requested");
  }
  const auto grasp = grasp_pose(object_centre, half_height_m, grasp_offset_m_);
  const auto pre_grasp = pre_grasp_pose(grasp, approach_height_m_);
  MotionCommander::PreparedPoseMotion prepared_pre_grasp;
  MotionCommander::PreparedCartesianMotion prepared_descent;
  SkillResult last_candidate_result = skill_error("descend_preflight", "no pre-grasp candidate was checked");
  bool approach_prepared = false;

  for (int candidate_index = 1; candidate_index <= max_pre_grasp_candidates_; ++candidate_index) {
    if (stop_token.stop_requested()) {
      return skill_error("pre_grasp", "cancellation requested during pre-grasp candidate search");
    }

    MotionCommander::PreparedPoseMotion candidate_pre_grasp;
    const auto pose_result = motion_.plan_pose_candidate(pre_grasp, candidate_pre_grasp);
    if (!pose_result.ok) {
      last_candidate_result = pose_result;
      RCLCPP_WARN(node_->get_logger(), "rejecting pre-grasp candidate %d/%d: %s", candidate_index, max_pre_grasp_candidates_,
                  pose_result.message.c_str());
      continue;
    }

    MotionCommander::PreparedCartesianMotion candidate_descent;
    const auto descent_result = with_grasp_contacts_disabled(
        object_id, [&] { return motion_.plan_cartesian_from(candidate_pre_grasp, grasp, candidate_descent); });

    if (!descent_result.ok) {
      last_candidate_result = descent_result;
      RCLCPP_WARN(node_->get_logger(), "rejecting pre-grasp candidate %d/%d: %s", candidate_index, max_pre_grasp_candidates_,
                  descent_result.message.c_str());
      continue;
    }

    prepared_pre_grasp = std::move(candidate_pre_grasp);
    prepared_descent = std::move(candidate_descent);
    approach_prepared = true;
    RCLCPP_INFO(node_->get_logger(), "selected pre-grasp candidate %d/%d after full Cartesian descent preflight",
                candidate_index, max_pre_grasp_candidates_);
    break;
  }

  if (!approach_prepared) {
    return skill_error("descend_preflight",
                       "no pre-grasp candidate supported the complete collision-checked Cartesian descent; last "
                       "failure: " +
                           last_candidate_result.message,
                       last_candidate_result.native_code);
  }

  const auto pre_grasp_result = motion_.execute_prepared_pose(prepared_pre_grasp);
  if (!pre_grasp_result.ok) {
    return pre_grasp_result;
  }

  if (enter_phase("descend")) {
    return skill_error("descend", "cancellation requested");
  }

  const auto sequence_result = with_grasp_contacts_disabled(object_id, [&]() -> SkillResult {
    const auto descend_result = motion_.execute_prepared_cartesian(prepared_descent);
    if (!descend_result.ok) {
      return descend_result;
    }

    if (enter_phase("close_gripper")) {
      return skill_error("close_gripper", "cancellation requested");
    }
    const auto close_result = gripper_.close();
    if (!close_result.ok) {
      return close_result;
    }

    const auto tcp_before_lift = motion_.current_tcp_pose();

    if (enter_phase("validation_lift")) {
      return skill_error("validation_lift", "cancellation requested");
    }
    const auto validation_target = retreat_pose(tcp_before_lift, grasp_validation_lift_m_);
    const auto lift_result = motion_.move_cartesian_to(validation_target);
    if (!lift_result.ok) {
      return skill_error("validation_lift", lift_result.message, lift_result.native_code);
    }

    if (enter_phase("probe_grasp_retention")) {
      return skill_error("probe_grasp_retention", "cancellation requested");
    }
    const auto probe_result = gripper_.probe_grasp_retention();
    if (!probe_result.ok) {
      return probe_result;
    }

    const auto tcp_after_lift = motion_.current_tcp_pose();

    Eigen::Isometry3d world_tcp_before;
    Eigen::Isometry3d world_tcp_after;
    Eigen::Isometry3d world_object_before;
    tf2::fromMsg(tcp_before_lift.pose, world_tcp_before);
    tf2::fromMsg(tcp_after_lift.pose, world_tcp_after);
    tf2::fromMsg(object_centre.pose, world_object_before);
    const Eigen::Isometry3d tcp_object_before = world_tcp_before.inverse() * world_object_before;
    const Eigen::Isometry3d world_object_after = world_tcp_after * tcp_object_before;

    geometry_msgs::msg::PoseStamped lifted_object_centre = object_centre;
    lifted_object_centre.pose = tf2::toMsg(world_object_after);

    // once our probe passes we attach without a cancellation gap, otherwise a
    // held object can be left outside the planning scene
    feedback->phase = "attach";
    goal_handle->publish_feedback(feedback);
    return scene_.attach_at_pose(object_id, lifted_object_centre);
  });
  if (!sequence_result.ok) {
    return sequence_result;
  }

  if (enter_phase("retreat")) {
    return skill_error("retreat", "cancellation requested");
  }
  const auto retreat = retreat_pose(grasp, retreat_height_m_);
  const auto retreat_result = motion_.move_cartesian_to(retreat);
  if (!retreat_result.ok) {
    return retreat_result;
  }

  return skill_ok("retreat");
}

void PickServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  auto result = std::make_shared<Pick::Result>();
  try {
    const auto goal = goal_handle->get_goal();
    const auto geometry = scene_.known_object_geometry(goal->object_id);
    if (!geometry) {
      result->result = to_msg(skill_error("validate", "object '" + goal->object_id + "' was never synced into the scene"));
      goal_handle->abort(result);
    } else {
      auto feedback = std::make_shared<Pick::Feedback>();
      const auto outcome =
          pick(goal->object_id, geometry->centre, geometry->half_height_m, stop_token, feedback, goal_handle);
      result->result = to_msg(outcome);

      if (goal_handle->is_canceling()) {
        goal_handle->canceled(result);
      } else if (outcome.ok) {
        state_.set_attached_object(goal->object_id);
        goal_handle->succeed(result);
      } else {
        goal_handle->abort(result);
      }
    }
  } catch (const std::exception& error) {
    result->result = to_msg(skill_error("internal", "Pick worker exception: " + std::string(error.what())));
    goal_handle->abort(result);
  } catch (...) {
    result->result = to_msg(skill_error("internal", "Pick worker caught an unknown exception"));
    goal_handle->abort(result);
  }
  state_.release();
}

}  // namespace sorting_arm
