#pragma once

#include <functional>
#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_arm_interfaces/action/pick.hpp"
#include "sorting_arm_skills/gripper_commander.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// The Pick action server: open, pre-grasp, descend, close, attach, retreat.
// Shares `state` with Place/Home so only one of the three ever runs.
class PickServerNode {
 public:
  PickServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SceneManager& scene, GripperCommander& gripper, SkillState& state);

 private:
  using Pick = sorting_arm_interfaces::action::Pick;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Pick>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Pick::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle);

  // The pick sequence itself. Returns as soon as a step fails or reports
  // cancellation.
  SkillResult pick(const std::string& object_id, const geometry_msgs::msg::PoseStamped& object_centre, double half_height_m,
                   std::stop_token stop_token, std::shared_ptr<Pick::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle);

  // Runs body() with object_id's grasp contacts allowed, then always restores
  // the collision matrix — a restore failure is folded into body's result
  // instead of silently dropped.
  SkillResult with_grasp_contacts_disabled(const std::string& object_id, const std::function<SkillResult()>& body);

  rclcpp::Node::SharedPtr node_;
  MotionCommander& motion_;
  SceneManager& scene_;
  GripperCommander& gripper_;
  SkillState& state_;

  double approach_height_m_ = 0.0;
  double retreat_height_m_ = 0.0;
  double grasp_offset_m_ = 0.0;
  int max_pre_grasp_candidates_ = 0;

  rclcpp_action::Server<Pick>::SharedPtr server_;

  // Declared last so destruction requests stop and joins before any borrowed
  // dependency can be destroyed by skill_server_main.cpp.
  std::jthread worker_;
};

}  // namespace sorting_arm
