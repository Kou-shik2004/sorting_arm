#pragma once

#include <memory>
#include <string>

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

// The Pick action server: open, pre-grasp, descend, close, verify, attach,
// retreat. Shares `state` with Place/Home so only one of the three ever runs.
class PickServerNode {
 public:
  PickServerNode(std::shared_ptr<rclcpp::Node> node, std::shared_ptr<MotionCommander> motion,
                 std::shared_ptr<SceneManager> scene, std::shared_ptr<GripperCommander> gripper,
                 std::shared_ptr<SkillState> state);

 private:
  using Pick = sorting_arm_interfaces::action::Pick;
  using GoalHandle = rclcpp_action::ServerGoalHandle<Pick>;

  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID& uuid, std::shared_ptr<const Pick::Goal> goal);
  rclcpp_action::CancelResponse handle_cancel(std::shared_ptr<GoalHandle> goal_handle);
  void handle_accepted(std::shared_ptr<GoalHandle> goal_handle);
  void run(std::stop_token stop_token, std::shared_ptr<GoalHandle> goal_handle);

  // The pick sequence itself. Returns as soon as a step fails or reports
  // cancellation.
  SkillResult pick(const std::string& object_id, const geometry_msgs::msg::PoseStamped& object_centre,
                   double half_height_m, double width_m, std::stop_token stop_token,
                   std::shared_ptr<Pick::Feedback> feedback, std::shared_ptr<GoalHandle> goal_handle);

  std::shared_ptr<rclcpp::Node> node_;
  std::shared_ptr<MotionCommander> motion_;
  std::shared_ptr<SceneManager> scene_;
  std::shared_ptr<GripperCommander> gripper_;
  std::shared_ptr<SkillState> state_;

  double approach_height_m_ = 0.0;
  double retreat_height_m_ = 0.0;
  double grasp_offset_m_ = 0.0;
  int max_pre_grasp_candidates_ = 0;

  rclcpp_action::Server<Pick>::SharedPtr server_;
};

}  // namespace sorting_arm
