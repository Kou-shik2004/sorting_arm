#pragma once

#include <memory>
#include <string>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// The Home action server: one named motion to the measured SRDF `home`
// state. Shares `state` with Pick/Place so only one of the three ever runs.
class HomeServerNode {
 public:
  HomeServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SkillState& state);

 private:
  using Home = sorting_arm_interfaces::action::Home;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Home>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Home::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle);

  SkillResult home(std::stop_token stop_token, std::shared_ptr<Home::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_;
  MotionCommander& motion_;
  SkillState& state_;

  rclcpp_action::Server<Home>::SharedPtr server_;
  std::jthread worker_;
};

}  // namespace sorting_arm
