#pragma once

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
#include "behaviortree_ros2/bt_action_node.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_action/client.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_interfaces/action/sort.hpp"
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
                    double timeout_s, std::shared_ptr<ExecutionReport> report, rclcpp::Clock::SharedPtr clock);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Client<Service>::SharedPtr client_;
  double timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  rclcpp::Clock::SharedPtr clock_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  rclcpp::Time deadline_;
};

class SyncObjectsNode : public BT::StatefulActionNode {
 public:
  using Service = sorting_arm_interfaces::srv::SyncObjects;

  SyncObjectsNode(const std::string& name, const BT::NodeConfig& config, rclcpp::Client<Service>::SharedPtr client,
                  double timeout_s, std::shared_ptr<ExecutionReport> report, rclcpp::Clock::SharedPtr clock);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus onStart() override;
  BT::NodeStatus onRunning() override;
  void onHalted() override;

  rclcpp::Client<Service>::SharedPtr client_;
  double timeout_s_ = 0.0;
  std::shared_ptr<ExecutionReport> report_;
  rclcpp::Clock::SharedPtr clock_;
  std::optional<rclcpp::Client<Service>::FutureAndRequestId> pending_;
  rclcpp::Time deadline_;
};

class HomeNode : public BT::RosActionNode<sorting_arm_interfaces::action::Home> {
 public:
  using Action = sorting_arm_interfaces::action::Home;

  HomeNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
           std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) override;

 private:
  std::shared_ptr<ExecutionReport> report_;
};

class SortNode : public BT::RosActionNode<sorting_arm_interfaces::action::Sort> {
 public:
  using Action = sorting_arm_interfaces::action::Sort;

  SortNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
           std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

  bool setGoal(Goal& goal) override;
  BT::NodeStatus onFeedback(const std::shared_ptr<const Feedback> feedback) override;
  BT::NodeStatus onResultReceived(const WrappedResult& result) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error) override;
  BT::NodeStatus onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) override;

 private:
  std::shared_ptr<ExecutionReport> report_;
};

void register_policy_nodes(BT::BehaviorTreeFactory& factory, AssignmentPlanner planner,
                           std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report);

}  // namespace sorting_arm_executive
