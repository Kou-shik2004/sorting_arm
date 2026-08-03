#include <algorithm>
#include <array>
#include <cmath>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/types.hpp"

static void print_result(const rclcpp::Logger& logger, const sorting_arm::SkillResult& result) {
  RCLCPP_INFO(logger, "result: ok=%s phase=%s native_code=%d message=%s", result.ok ? "true" : "false", result.phase.c_str(), result.native_code,
              result.message.c_str());
}

// One-shot, parameter-driven proof of each reusable API: pick one operation by
// parameter, run it once, print the result, exit.
class MotionDemo {
 public:
  explicit MotionDemo(rclcpp::Node::SharedPtr node) {
    node_ = node;

    operation_ = node_->declare_parameter<std::string>("operation", "");
    if (operation_ != "named" && operation_ != "joint" && operation_ != "pose" && operation_ != "cartesian" && operation_ != "apply_scene") {
      throw std::runtime_error("set -p operation:=<named|joint|pose|cartesian|apply_scene>, plus that operation's targets");
    }

    target_name_ = node_->declare_parameter<std::string>("target_name", "");
    joint_values_ = node_->declare_parameter<std::vector<double>>("joint_values", std::vector<double>{});

    pose_target_.header.frame_id = node_->declare_parameter<std::string>("frame_id", "world");
    pose_target_.pose.position.x = node_->declare_parameter<double>("x", std::nan(""));
    pose_target_.pose.position.y = node_->declare_parameter<double>("y", std::nan(""));
    pose_target_.pose.position.z = node_->declare_parameter<double>("z", std::nan(""));
    pose_target_.pose.orientation.x = node_->declare_parameter<double>("qx", std::nan(""));
    pose_target_.pose.orientation.y = node_->declare_parameter<double>("qy", std::nan(""));
    pose_target_.pose.orientation.z = node_->declare_parameter<double>("qz", std::nan(""));
    pose_target_.pose.orientation.w = node_->declare_parameter<double>("qw", std::nan(""));

    RCLCPP_INFO(node_->get_logger(), "operation=%s frame_id=%s", operation_.c_str(), pose_target_.header.frame_id.c_str());
  }

  // Builds only the commander the chosen operation needs — SceneManager's
  // PlanningSceneInterface blocks on MoveIt's scene services, so a named/
  // joint/pose/cartesian run must never pay that cost for nothing.
  sorting_arm::SkillResult run() {
    if (operation_ == "apply_scene") {
      sorting_arm::SceneManager scene(node_);
      return scene.apply_static_scene();
    }

    if (operation_ == "named") {
      sorting_arm::MotionCommander motion(node_);
      return motion.move_to_named(target_name_);
    }

    if (operation_ == "joint") {
      if (joint_values_.size() != 6) {
        return sorting_arm::skill_error("joint_motion", "joint_values must have exactly six entries");
      }
      std::array<double, 6> joints{};
      std::copy(joint_values_.begin(), joint_values_.end(), joints.begin());
      sorting_arm::MotionCommander motion(node_);
      return motion.move_to_joints(joints);
    }

    sorting_arm::MotionCommander motion(node_);
    return (operation_ == "pose") ? motion.move_to_pose(pose_target_) : motion.move_cartesian_to(pose_target_);
  }

 private:
  rclcpp::Node::SharedPtr node_;
  std::string operation_;
  std::string target_name_;
  std::vector<double> joint_values_;
  geometry_msgs::msg::PoseStamped pose_target_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("motion_demo");

  std::unique_ptr<MotionDemo> demo;
  try {
    demo = std::make_unique<MotionDemo>(node);
  } catch (const std::exception& e) {
    RCLCPP_ERROR(node->get_logger(), "%s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  // MotionCommander/SceneManager need the executor already spinning; spinning
  // unconditionally keeps every operation's setup identical.
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::jthread spin_thread([&executor](std::stop_token stop_token) {
    std::stop_callback cancel_on_stop(stop_token, [&executor]() { executor.cancel(); });
    executor.spin();
  });

  const auto result = demo->run();
  print_result(node->get_logger(), result);

  const int exit_code = result.ok ? 0 : 1;
  spin_thread.request_stop();
  rclcpp::shutdown();
  return exit_code;
}
