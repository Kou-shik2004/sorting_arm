#pragma once

#include <memory>
#include <thread>

#include "moveit/move_group_interface/move_group_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Home action server: one named move to the SRDF `home` pose on its own
// MoveGroupInterface (D6). Shares `state` with Sort so only one goal runs.
class HomeServerNode {
 public:
  HomeServerNode(rclcpp::Node::SharedPtr node, SkillState& state);

 private:
  using Home = sorting_arm_interfaces::action::Home;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Home>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Home::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle);

  SkillResult home(std::stop_token stop_token, std::shared_ptr<Home::Feedback> feedback,
                   std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_;
  SkillState& state_;
  moveit::planning_interface::MoveGroupInterface arm_;

  rclcpp_action::Server<Home>::SharedPtr server_;
  std::jthread worker_;
};

}  // namespace sorting_arm
