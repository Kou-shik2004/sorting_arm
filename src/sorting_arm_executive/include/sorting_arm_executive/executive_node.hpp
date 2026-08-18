#pragma once

#include <chrono>
#include <memory>
#include <optional>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"
#include "sorting_arm_executive/config.hpp"
#include "sorting_arm_interfaces/srv/detect_objects.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"

namespace sorting_arm_executive {

enum class CycleState { waiting_for_readiness, running, succeeded, failed };

class ExecutiveNode : public rclcpp::Node {
 public:
  explicit ExecutiveNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

  // second-phase startup: registers BT nodes and builds the tree once the node is owned
  // by a shared_ptr, so RosActionNode's RosNodeParams can take shared_from_this()
  void initialize();

  CycleState cycle_state() const;
  const ExecutionReport& report() const;

 private:
  void tick();
  void check_readiness();
  void fail_startup(const std::string& message);
  bool controller_response_is_ready(const controller_manager_msgs::srv::ListControllers::Response& response) const;
  void log_terminal(BT::NodeStatus status);

  ExecutiveConfig config_;
  std::optional<AssignmentPlanner> planner_;
  std::shared_ptr<ExecutionReport> report_ = std::make_shared<ExecutionReport>();

  rclcpp::Client<sorting_arm_interfaces::srv::DetectObjects>::SharedPtr detect_client_;
  rclcpp::Client<sorting_arm_interfaces::srv::SyncObjects>::SharedPtr sync_client_;
  rclcpp::Client<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_client_;
  std::optional<rclcpp::Client<controller_manager_msgs::srv::ListControllers>::FutureAndRequestId> controller_request_;

  BT::BehaviorTreeFactory factory_;
  BT::Tree tree_;
  rclcpp::TimerBase::SharedPtr timer_;
  CycleState state_ = CycleState::waiting_for_readiness;
  bool controllers_ready_ = false;
  // readiness is "is my ROS graph up", not simulated-world progress - stays wall time so
  // a Gazebo that never starts (never publishes /clock) still fails loud instead of hanging
  std::chrono::steady_clock::time_point readiness_deadline_;
  std::chrono::steady_clock::time_point next_controller_query_;
};

}  // namespace sorting_arm_executive
