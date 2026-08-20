#pragma once

#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_arm_interfaces/action/sort.hpp"
#include "sorting_arm_skills/mtc_pick_place.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// The Sort action server: one native MTC task grasps a synced object and places
// it at a destination (D31). Shares `state` with Home so only one goal runs at a
// time; cancellation preempts the in-flight MTC plan.
class SortServerNode {
 public:
  SortServerNode(rclcpp::Node::SharedPtr node, MtcPickPlace& planner, SceneManager& scene, SkillState& state);

 private:
  using Sort = sorting_arm_interfaces::action::Sort;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Sort>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Sort::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::shared_ptr<GoalHandle> goal_handle);

  // Plan then execute the MTC task, publishing a coarse phase around each.
  SkillResult sort_object(const std::string& object_id, double half_height_m,
                          const geometry_msgs::msg::PoseStamped& destination, std::shared_ptr<Sort::Feedback> feedback,
                          std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_;
  MtcPickPlace& planner_;
  SceneManager& scene_;
  SkillState& state_;

  rclcpp_action::Server<Sort>::SharedPtr server_;

  // Declared last so destruction joins the worker before any borrowed
  // dependency is destroyed by skill_server_main.cpp.
  std::jthread worker_;
};

}  // namespace sorting_arm
