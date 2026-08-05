#pragma once

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <future>
#include <memory>
#include <optional>
#include <set>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/client.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp_action/client.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_interfaces/action/pick.hpp"
#include "sorting_arm_interfaces/action/place.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_interfaces/srv/detect_objects.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"

namespace sorting_arm_executive {

struct AdaptiveCycleState {
  struct PendingJob {
    SortJob job;
    sorting_arm_interfaces::msg::DetectedObject object;
  };

  explicit AdaptiveCycleState(std::size_t initial_count) : remaining_count(initial_count) {}

  std::size_t remaining_count = 0;
  std::size_t scan_number = 1;
  std::set<std::size_t> used_destination_slots;
  std::vector<sorting_arm_interfaces::msg::DetectedObject> placed_objects;
  std::optional<PendingJob> pending_job;
};

class RemainingCountNode : public BT::SyncActionNode {
 public:
  RemainingCountNode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<AdaptiveCycleState> state,
                     std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;

  std::shared_ptr<AdaptiveCycleState> state_;
  std::shared_ptr<ExecutionReport> report_;
};

class PlanNextJobNode : public BT::SyncActionNode {
 public:
  PlanNextJobNode(const std::string& name, const BT::NodeConfig& config, AssignmentPlanner planner,
                  std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;

  AssignmentPlanner planner_;
  std::shared_ptr<AdaptiveCycleState> state_;
  std::shared_ptr<ExecutionReport> report_;
};

class CommitPlacedJobNode : public BT::SyncActionNode {
 public:
  CommitPlacedJobNode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<AdaptiveCycleState> state,
                      std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;

  std::shared_ptr<AdaptiveCycleState> state_;
  std::shared_ptr<ExecutionReport> report_;
};

class DetectObjectsNode : public BT::StatefulActionNode {
 public:
  using Service = sorting_arm_interfaces::srv::DetectObjects;

  DetectObjectsNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Client<Service>::SharedPtr client,
                    double timeout_s, std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Client<Service>::SharedPtr client_;
  double timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  std::chrono::steady_clock::time_point deadline_;
};

class SyncObjectsNode : public BT::StatefulActionNode {
 public:
  using Service = sorting_arm_interfaces::srv::SyncObjects;

  SyncObjectsNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Client<Service>::SharedPtr client,
                  double timeout_s, std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Client<Service>::SharedPtr client_;
  double timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  std::chrono::steady_clock::time_point deadline_;
};

class HomeNode : public BT::StatefulActionNode {
 public:
  using Action = sorting_arm_interfaces::action::Home;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  HomeNode(const std::string& name, const BT::NodeConfig& config, rclcpp_action::Client<Action>::SharedPtr client,
           double action_timeout_s, double cancel_timeout_s, std::shared_ptr<ExecutionReport> report, rclcpp::Logger logger);

  static BT::PortsList providedPorts();

 private:
  enum class Stage { waiting_goal, waiting_result, canceling };

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void request_cancel(const std::string& reason);
  BT::NodeStatus finish(const GoalHandle::WrappedResult& wrapped);

  rclcpp_action::Client<Action>::SharedPtr client_;
  double action_timeout_s_ = 0.0;
  double cancel_timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  rclcpp::Logger logger_;
  Stage stage_ = Stage::waiting_goal;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point deadline_;
};

class PickNode : public BT::StatefulActionNode {
 public:
  using Action = sorting_arm_interfaces::action::Pick;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  PickNode(const std::string& name, const BT::NodeConfig& config, rclcpp_action::Client<Action>::SharedPtr client,
           double action_timeout_s, double cancel_timeout_s, std::shared_ptr<ExecutionReport> report, rclcpp::Logger logger);

  static BT::PortsList providedPorts();

 private:
  enum class Stage { waiting_goal, waiting_result, canceling };

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void request_cancel(const std::string& reason);
  BT::NodeStatus finish(const GoalHandle::WrappedResult& wrapped);

  rclcpp_action::Client<Action>::SharedPtr client_;
  double action_timeout_s_ = 0.0;
  double cancel_timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  rclcpp::Logger logger_;
  Stage stage_ = Stage::waiting_goal;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point deadline_;
};

class PlaceNode : public BT::StatefulActionNode {
 public:
  using Action = sorting_arm_interfaces::action::Place;
  using GoalHandle = rclcpp_action::ClientGoalHandle<Action>;

  PlaceNode(const std::string& name, const BT::NodeConfig& config, rclcpp_action::Client<Action>::SharedPtr client,
            double action_timeout_s, double cancel_timeout_s, std::shared_ptr<ExecutionReport> report,
            rclcpp::Logger logger);

  static BT::PortsList providedPorts();

 private:
  enum class Stage { waiting_goal, waiting_result, canceling };

  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;
  void request_cancel(const std::string& reason);
  BT::NodeStatus finish(const GoalHandle::WrappedResult& wrapped);

  rclcpp_action::Client<Action>::SharedPtr client_;
  double action_timeout_s_ = 0.0;
  double cancel_timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  rclcpp::Logger logger_;
  Stage stage_ = Stage::waiting_goal;
  std::shared_future<GoalHandle::SharedPtr> goal_future_;
  std::shared_future<GoalHandle::WrappedResult> result_future_;
  GoalHandle::SharedPtr goal_handle_;
  std::chrono::steady_clock::time_point deadline_;
};

void register_policy_nodes(BT::BehaviorTreeFactory& factory, AssignmentPlanner planner,
                           std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report);

}  // namespace sorting_arm_executive
