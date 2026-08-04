#pragma once

#include <memory>
#include <string>
#include <thread>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "sorting_arm_interfaces/action/place.hpp"
#include "sorting_arm_skills/gripper_commander.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// The Place action server: pre-place, descend, open, detach+reinsert,
// retreat. Shares `state` with Pick/Home so only one of the three ever runs.
class PlaceServerNode {
 public:
  PlaceServerNode(rclcpp::Node::SharedPtr node, MotionCommander& motion, SceneManager& scene, GripperCommander& gripper,
                  SkillState& state);

 private:
  using Place = sorting_arm_interfaces::action::Place;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Place>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Place::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle);

  SkillResult place(const std::string& object_id, const geometry_msgs::msg::PoseStamped& destination_centre,
                    double half_height_m, std::stop_token stop_token, std::shared_ptr<Place::Feedback> feedback,
                    std::shared_ptr<GoalHandle> goal_handle);

  rclcpp::Node::SharedPtr node_;
  MotionCommander& motion_;
  SceneManager& scene_;
  GripperCommander& gripper_;
  SkillState& state_;

  double approach_height_m_ = 0.0;
  double retreat_height_m_ = 0.0;
  double grasp_offset_m_ = 0.0;

  rclcpp_action::Server<Place>::SharedPtr server_;
  std::jthread worker_;
};

}  // namespace sorting_arm
