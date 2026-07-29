#include <array>
#include <cmath>
#include <exception>
#include <memory>
#include <string>
#include <string_view>
#include <thread>

#include <moveit/move_group_interface/move_group_interface.hpp>
#include <moveit/robot_state/robot_state.hpp>
#include <rclcpp/rclcpp.hpp>

namespace {

constexpr std::string_view kPlanningGroup{"arm"};
constexpr std::string_view kExpectedPlanningFrame{"world"};
constexpr std::string_view kExpectedEndEffectorLink{"tcp"};
constexpr double kMoveGroupWaitSeconds{10.0};
constexpr double kRobotStateWaitSeconds{5.0};

constexpr std::array<std::string_view, 6> kExpectedJointNames{
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint",      "wrist_2_joint",       "wrist_3_joint",
};

int checkMoveItReadiness(const rclcpp::Node::SharedPtr &node) {
  const auto logger = node->get_logger();

  RCLCPP_INFO(logger, "Connecting to MoveIt for planning group '%s'...",
              kPlanningGroup.data());

  moveit::planning_interface::MoveGroupInterface move_group{
      node, std::string{kPlanningGroup}, nullptr,
      rclcpp::Duration::from_seconds(kMoveGroupWaitSeconds)};

  const auto &planning_frame = move_group.getPlanningFrame();
  const auto &end_effector_link = move_group.getEndEffectorLink();

  RCLCPP_INFO(logger, "Planning frame: '%s'", planning_frame.c_str());
  RCLCPP_INFO(logger, "End-effector link: '%s'", end_effector_link.c_str());

  if (planning_frame != kExpectedPlanningFrame) {
    RCLCPP_ERROR(logger,
                 "Expected planning frame '%s', but MoveIt reported '%s'",
                 kExpectedPlanningFrame.data(), planning_frame.c_str());
    return 2;
  }

  if (end_effector_link != kExpectedEndEffectorLink) {
    RCLCPP_ERROR(logger,
                 "Expected end-effector link '%s', but MoveIt reported '%s'",
                 kExpectedEndEffectorLink.data(), end_effector_link.c_str());
    return 2;
  }

  const auto &joint_names = move_group.getJointNames();
  if (joint_names.size() != kExpectedJointNames.size()) {
    RCLCPP_ERROR(logger, "Expected %zu arm joints, but MoveIt reported %zu",
                 kExpectedJointNames.size(), joint_names.size());
    return 2;
  }

  for (std::size_t index = 0; index < kExpectedJointNames.size(); ++index) {
    if (joint_names[index] != kExpectedJointNames[index]) {
      RCLCPP_ERROR(logger, "Joint %zu should be '%s', but MoveIt reported '%s'",
                   index, kExpectedJointNames[index].data(),
                   joint_names[index].c_str());
      return 2;
    }
  }

  RCLCPP_INFO(logger,
              "Waiting up to %.1f seconds for the current robot state...",
              kRobotStateWaitSeconds);

  const auto current_state = move_group.getCurrentState(kRobotStateWaitSeconds);
  if (!current_state) {
    RCLCPP_ERROR(logger, "No current robot state was received. Check the "
                         "joint-state broadcaster, "
                         "robot_description, /clock, and use_sim_time.");
    return 3;
  }

  for (const auto joint_name : kExpectedJointNames) {
    const double position =
        current_state->getVariablePosition(std::string{joint_name});
    if (!std::isfinite(position)) {
      RCLCPP_ERROR(logger, "Joint '%s' has a non-finite position",
                   joint_name.data());
      return 3;
    }

    RCLCPP_INFO(logger, "Current joint position: %-20s % .6f rad",
                joint_name.data(), position);
  }

  const auto &named_targets = move_group.getNamedTargets();
  RCLCPP_INFO(logger, "Named arm targets available in the SRDF: %zu",
              named_targets.size());
  for (const auto &target : named_targets) {
    RCLCPP_INFO(logger, "Named arm target: '%s'", target.c_str());
  }

  RCLCPP_INFO(logger, "MoveIt readiness check passed: model, frames, arm "
                      "joints, and live state "
                      "match this project.");
  return 0;
}

} // namespace

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  const auto node = std::make_shared<rclcpp::Node>(
      "moveit_ready_check",
      rclcpp::NodeOptions{}.automatically_declare_parameters_from_overrides(
          true));

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::jthread spin_thread{[&executor]() { executor.spin(); }};

  int exit_code = 4;
  try {
    exit_code = checkMoveItReadiness(node);
  } catch (const std::exception &error) {
    RCLCPP_FATAL(node->get_logger(),
                 "MoveIt readiness check threw an exception: %s", error.what());
  }

  executor.cancel();
  spin_thread.join();
  rclcpp::shutdown();
  return exit_code;
}
