#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_ros2/bt_action_node.hpp"
#include "behaviortree_ros2/bt_service_node.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_interfaces/action/sort.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"
#include "sorting_arm_interfaces/srv/detect_objects.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"

namespace sorting_arm_executive {

struct SortJob {
  std::string object_id;
  std::string label;
  geometry_msgs::msg::PoseStamped destination;
};

struct ExecutionReport {
  std::size_t total_jobs = 0;
  std::size_t completed_jobs = 0;
  std::string operation;
  std::string object_id;
  std::string phase;
  bool has_failure = false;
  sorting_arm_interfaces::msg::SkillResult failure;
};

// Cubes are placed on a spaced grid inside their colour's tray so the gripper clears
// already-placed neighbours on retreat. Geometry mirrors the trays in sorting_cell.sdf.
// kTrayCapacity is the hard limit on object_count: a run may be all one colour.
constexpr std::size_t kTrayCapacity = 8;

geometry_msgs::msg::PoseStamped tray_slot(const std::string& label, std::size_t index_in_tray);

struct AdaptiveCycleState {
  struct PendingJob {
    SortJob job;
    sorting_arm_interfaces::msg::DetectedObject object;
  };

  explicit AdaptiveCycleState(std::size_t initial_count) : remaining_count(initial_count) {}

  std::size_t remaining_count = 0;
  std::size_t scan_number = 1;
  std::size_t red_placed = 0;
  std::size_t blue_placed = 0;
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
  PlanNextJobNode(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<AdaptiveCycleState> state,
                  std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

 private:
  BT::NodeStatus tick() override;

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

class DetectObjectsNode : public BT::RosServiceNode<sorting_arm_interfaces::srv::DetectObjects> {
 public:
  using Service = sorting_arm_interfaces::srv::DetectObjects;

  DetectObjectsNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                    std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

 private:
  std::shared_ptr<ExecutionReport> report_;
};

class SyncObjectsNode : public BT::RosServiceNode<sorting_arm_interfaces::srv::SyncObjects> {
 public:
  using Service = sorting_arm_interfaces::srv::SyncObjects;

  SyncObjectsNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                  std::shared_ptr<ExecutionReport> report);

  static BT::PortsList providedPorts();

  bool setRequest(Request::SharedPtr& request) override;
  BT::NodeStatus onResponseReceived(const Response::SharedPtr& response) override;
  BT::NodeStatus onFailure(BT::ServiceNodeErrorCode error) override;

 private:
  std::shared_ptr<ExecutionReport> report_;
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

void register_policy_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<AdaptiveCycleState> state,
                           std::shared_ptr<ExecutionReport> report);

}  // namespace sorting_arm_executive
