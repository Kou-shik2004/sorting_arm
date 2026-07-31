#include "sorting_arm_skills/pick_server.hpp"

#include <stdexcept>
#include <utility>

#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

using namespace std::placeholders;

PickServerNode::PickServerNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MotionCommander> motion,
                               std::shared_ptr<SceneManager> scene, std::shared_ptr<GripperCommander> gripper,
                               std::shared_ptr<SkillState> state)
    : node_(std::move(node)),
      motion_(std::move(motion)),
      scene_(std::move(scene)),
      gripper_(std::move(gripper)),
      state_(std::move(state)) {
  approach_height_m_ = declare_or_get<double>(*node_, "targets.approach_height_m", 0.12);
  retreat_height_m_ = declare_or_get<double>(*node_, "targets.retreat_height_m", 0.12);
  grasp_offset_m_ = declare_or_get<double>(*node_, "targets.grasp_offset_m", -0.036);
  max_pre_grasp_candidates_ = declare_or_get<int>(*node_, "targets.max_pre_grasp_candidates", 5);

  if (approach_height_m_ <= 0.0) {
    throw std::runtime_error("targets.approach_height_m must be positive");
  }
  if (retreat_height_m_ <= 0.0) {
    throw std::runtime_error("targets.retreat_height_m must be positive");
  }
  if (max_pre_grasp_candidates_ <= 0) {
    throw std::runtime_error("targets.max_pre_grasp_candidates must be positive");
  }

  server_ = rclcpp_action::create_server<Pick>(node_, "pick", std::bind(&PickServerNode::handle_goal, this, _1, _2),
                                               std::bind(&PickServerNode::handle_cancel, this, _1),
                                               std::bind(&PickServerNode::handle_accepted, this, _1));
}

rclcpp_action::GoalResponse PickServerNode::handle_goal(const rclcpp_action::GoalUUID&,
                                                        std::shared_ptr<const Pick::Goal>) {
  if (!state_->try_claim()) {
    RCLCPP_WARN(node_->get_logger(), "rejecting Pick goal: a manipulation goal is already active");
    return rclcpp_action::GoalResponse::REJECT;
  }
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse PickServerNode::handle_cancel(std::shared_ptr<GoalHandle>) {
  RCLCPP_INFO(node_->get_logger(), "cancel requested for the active Pick goal");
  state_->request_stop();
  return rclcpp_action::CancelResponse::ACCEPT;
}

void PickServerNode::handle_accepted(std::shared_ptr<GoalHandle> goal_handle) {
  state_->start_worker([this, goal_handle](std::stop_token stop_token) { run(stop_token, goal_handle); });
}

SkillResult PickServerNode::pick(const std::string& object_id, const geometry_msgs::msg::PoseStamped& object_centre,
                                 double half_height_m, std::stop_token stop_token,
                                 std::shared_ptr<Pick::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle) {
  auto enter_phase = [&](const std::string& phase) {
    feedback->phase = phase;
    goal_handle->publish_feedback(feedback);
    return stop_token.stop_requested();
  };

  if (enter_phase("open_gripper")) return skill_error("open_gripper", "cancellation requested");
  const auto open_result = gripper_->open();
  if (!open_result.ok) return open_result;

  if (enter_phase("pre_grasp")) return skill_error("pre_grasp", "cancellation requested");
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
    const auto pose_result = motion_->plan_pose_candidate(pre_grasp, candidate_pre_grasp);
    if (!pose_result.ok) {
      last_candidate_result = pose_result;
      RCLCPP_WARN(node_->get_logger(), "rejecting pre-grasp candidate %d/%d: %s", candidate_index,
                  max_pre_grasp_candidates_, pose_result.message.c_str());
      continue;
    }

    const auto allow_result = scene_->begin_grasp_contacts(object_id);
    if (!allow_result.ok) {
      return allow_result;
    }

    MotionCommander::PreparedCartesianMotion candidate_descent;
    const auto descent_result = motion_->plan_cartesian_from(candidate_pre_grasp, grasp, candidate_descent);
    const auto restore_result = scene_->end_grasp_contacts();
    if (!restore_result.ok) {
      if (!descent_result.ok) {
        return skill_error("grasp_contacts", descent_result.message + "; " + restore_result.message,
                           restore_result.native_code);
      }
      return restore_result;
    }

    if (!descent_result.ok) {
      last_candidate_result = descent_result;
      RCLCPP_WARN(node_->get_logger(), "rejecting pre-grasp candidate %d/%d: %s", candidate_index,
                  max_pre_grasp_candidates_, descent_result.message.c_str());
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

  const auto pre_grasp_result = motion_->execute_prepared_pose(prepared_pre_grasp);
  if (!pre_grasp_result.ok) return pre_grasp_result;

  if (enter_phase("descend")) return skill_error("descend", "cancellation requested");
  const auto allow_result = scene_->begin_grasp_contacts(object_id);
  if (!allow_result.ok) return allow_result;

  auto restore_contacts = [&](const SkillResult& primary_result) {
    const auto restore_result = scene_->end_grasp_contacts();
    if (restore_result.ok) {
      return primary_result;
    }
    if (primary_result.ok) {
      return restore_result;
    }
    return skill_error("grasp_contacts", primary_result.message + "; " + restore_result.message,
                       restore_result.native_code);
  };

  const auto descend_result = motion_->execute_prepared_cartesian(prepared_descent);
  if (!descend_result.ok) return restore_contacts(descend_result);

  if (enter_phase("close_gripper")) {
    return restore_contacts(skill_error("close_gripper", "cancellation requested"));
  }
  const auto grasp_outcome = gripper_->close();
  if (!grasp_outcome.ok) {
    return restore_contacts(skill_error("close_gripper", grasp_outcome.detail, grasp_outcome.native_code));
  }

  feedback->phase = "verify_grasp";
  goal_handle->publish_feedback(feedback);
  if (!grasp_outcome.object_present) {
    return restore_contacts(skill_error("verify_grasp", grasp_outcome.detail));
  }

  // no cancellation check between verify_grasp and attach — an unverified
  // grasp must never reach the scene
  feedback->phase = "attach";
  goal_handle->publish_feedback(feedback);
  const auto attach_result = scene_->attach(object_id);
  if (!attach_result.ok) return restore_contacts(attach_result);

  const auto restore_result = restore_contacts(skill_ok("attach"));
  if (!restore_result.ok) return restore_result;

  if (enter_phase("retreat")) return skill_error("retreat", "cancellation requested");
  const auto retreat = retreat_pose(grasp, retreat_height_m_);
  const auto retreat_result = motion_->move_cartesian_to(retreat);
  if (!retreat_result.ok) return retreat_result;

  return skill_ok("retreat");
}

void PickServerNode::run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle) {
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<Pick::Feedback>();
  auto result = std::make_shared<Pick::Result>();

  const auto geometry = scene_->known_object_geometry(goal->object_id);
  if (!geometry) {
    result->result =
        to_msg(skill_error("validate", "object '" + goal->object_id + "' was never synced into the scene"));
    goal_handle->abort(result);
    state_->release();
    return;
  }

  const auto outcome =
      pick(goal->object_id, geometry->centre, geometry->half_height_m, stop_token, feedback, goal_handle);
  result->result = to_msg(outcome);

  if (goal_handle->is_canceling()) {
    goal_handle->canceled(result);
  } else if (outcome.ok) {
    state_->set_attached_object(goal->object_id);
    goal_handle->succeed(result);
  } else {
    goal_handle->abort(result);
  }
  state_->release();
}

}  // namespace sorting_arm
