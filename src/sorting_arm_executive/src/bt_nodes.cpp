#include "sorting_arm_executive/bt_nodes.hpp"

#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace sorting_arm_executive {
namespace {

// Tray grid geometry must match sorting_cell.sdf and SceneManager's collision objects.
// Two columns by two rows hold the most an equal red/blue split sends to one tray.
constexpr std::size_t kTrayColumns = 2;
constexpr double kPlacePitch = 0.08;
constexpr double kRedTrayCentreX = 0.70;
constexpr double kRedTrayCentreY = 0.36;
constexpr double kBlueTrayCentreX = 0.70;
constexpr double kBlueTrayCentreY = -0.36;
constexpr double kPlaceZ = 0.525;

sorting_arm_interfaces::msg::SkillResult failure_result(std::string phase, std::string message) {
  sorting_arm_interfaces::msg::SkillResult result;
  result.ok = false;
  result.phase = std::move(phase);
  result.message = std::move(message);
  return result;
}

void record_failure(const std::shared_ptr<ExecutionReport>& report,
                    const sorting_arm_interfaces::msg::SkillResult& failure) {
  if (!report->has_failure) {
    report->has_failure = true;
    report->failure = failure;
  }
}

// RosActionNode routes only SUCCEEDED to onResultReceived; a server can still finish cleanly
// with a logical ok=false, so a non-ok payload is a FAILURE carrying that message.
BT::NodeStatus record_result(const std::shared_ptr<ExecutionReport>& report,
                             const sorting_arm_interfaces::msg::SkillResult& result) {
  if (result.ok) {
    return BT::NodeStatus::SUCCESS;
  }
  record_failure(report, result);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus report_error(const std::shared_ptr<ExecutionReport>& report, const std::string& phase,
                            BT::ActionNodeErrorCode error) {
  record_failure(report, failure_result(phase, phase + " " + BT::toStr(error)));
  return BT::NodeStatus::FAILURE;
}

}  // namespace

geometry_msgs::msg::PoseStamped tray_slot(const std::string& label, std::size_t index_in_tray) {
  const double centre_x = label == "red" ? kRedTrayCentreX : kBlueTrayCentreX;
  const double centre_y = label == "red" ? kRedTrayCentreY : kBlueTrayCentreY;
  const std::size_t column = index_in_tray % kTrayColumns;
  const std::size_t row = index_in_tray / kTrayColumns;

  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = centre_x + (static_cast<double>(column) - (kTrayColumns - 1) / 2.0) * kPlacePitch;
  pose.pose.position.y = centre_y + (static_cast<double>(row) - 0.5) * kPlacePitch;
  pose.pose.position.z = kPlaceZ;
  pose.pose.orientation.w = 1.0;
  return pose;
}

RemainingCountNode::RemainingCountNode(const std::string& name, const BT::NodeConfig& config,
                                       std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report)
    : BT::SyncActionNode(name, config), state_(std::move(state)), report_(std::move(report)) {}

BT::PortsList RemainingCountNode::providedPorts() { return {BT::OutputPort<std::uint32_t>("expected_count")}; }

BT::NodeStatus RemainingCountNode::tick() {
  report_->operation = "RemainingCount";
  setOutput("expected_count", static_cast<std::uint32_t>(state_->remaining_count));
  return BT::NodeStatus::SUCCESS;
}

PlanNextJobNode::PlanNextJobNode(const std::string& name, const BT::NodeConfig& config,
                                 std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report)
    : BT::SyncActionNode(name, config), state_(std::move(state)), report_(std::move(report)) {}

BT::PortsList PlanNextJobNode::providedPorts() {
  return {BT::InputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects"), BT::OutputPort<SortJob>("job"),
          BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("scene_objects")};
}

BT::NodeStatus PlanNextJobNode::tick() {
  report_->operation = "PlanNextJob";
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
  getInput("objects", objects);
  if (objects.size() != state_->remaining_count) {
    record_failure(report_, failure_result("allocation", "detection count does not match remaining cycle count"));
    return BT::NodeStatus::FAILURE;
  }

  const std::string prefix = "scan_" + std::to_string(state_->scan_number) + "_";
  for (auto& object : objects) {
    object.id = prefix + object.id;
  }

  // pick any remaining cube; its colour fixes the tray, its per-tray placed count fixes the slot
  const auto& next = objects.front();
  const std::size_t index_in_tray = next.label == "red" ? state_->red_placed : state_->blue_placed;
  SortJob job{next.id, next.label, tray_slot(next.label, index_in_tray)};

  std::vector<sorting_arm_interfaces::msg::DetectedObject> scene_objects = state_->placed_objects;
  scene_objects.insert(scene_objects.end(), objects.begin(), objects.end());
  setOutput("job", job);
  setOutput("scene_objects", scene_objects);
  state_->pending_job = AdaptiveCycleState::PendingJob{job, next};
  ++state_->scan_number;
  return BT::NodeStatus::SUCCESS;
}

CommitPlacedJobNode::CommitPlacedJobNode(const std::string& name, const BT::NodeConfig& config,
                                         std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report)
    : BT::SyncActionNode(name, config), state_(std::move(state)), report_(std::move(report)) {}

BT::PortsList CommitPlacedJobNode::providedPorts() { return {BT::InputPort<SortJob>("job")}; }

BT::NodeStatus CommitPlacedJobNode::tick() {
  report_->operation = "CommitPlacedJob";
  SortJob job;
  getInput("job", job);

  auto placed_object = state_->pending_job->object;
  placed_object.centre = job.destination;
  state_->placed_objects.push_back(std::move(placed_object));
  if (job.label == "red") {
    ++state_->red_placed;
  } else {
    ++state_->blue_placed;
  }
  --state_->remaining_count;
  state_->pending_job.reset();
  return BT::NodeStatus::SUCCESS;
}

DetectObjectsNode::DetectObjectsNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                                     std::shared_ptr<ExecutionReport> report)
    : BT::RosServiceNode<Service>(name, config, params), report_(std::move(report)) {}

BT::PortsList DetectObjectsNode::providedPorts() {
  return providedBasicPorts({BT::InputPort<std::uint32_t>("expected_count"),
                             BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")});
}

bool DetectObjectsNode::setRequest(Request::SharedPtr& request) {
  report_->operation = "DetectObjects";
  report_->object_id.clear();
  report_->phase.clear();
  std::uint32_t expected_count = 0;
  getInput("expected_count", expected_count);
  request->expected_count = expected_count;
  return true;
}

BT::NodeStatus DetectObjectsNode::onResponseReceived(const Response::SharedPtr& response) {
  if (!response->result.ok) {
    record_failure(report_, response->result);
    return BT::NodeStatus::FAILURE;
  }
  setOutput("objects", response->objects);
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus DetectObjectsNode::onFailure(BT::ServiceNodeErrorCode error) {
  record_failure(report_, failure_result("detect", std::string("DetectObjects ") + BT::toStr(error)));
  return BT::NodeStatus::FAILURE;
}

SyncObjectsNode::SyncObjectsNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                                 std::shared_ptr<ExecutionReport> report)
    : BT::RosServiceNode<Service>(name, config, params), report_(std::move(report)) {}

BT::PortsList SyncObjectsNode::providedPorts() {
  return providedBasicPorts({BT::InputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")});
}

bool SyncObjectsNode::setRequest(Request::SharedPtr& request) {
  report_->operation = "SyncObjects";
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
  getInput("objects", objects);
  request->objects = std::move(objects);
  return true;
}

BT::NodeStatus SyncObjectsNode::onResponseReceived(const Response::SharedPtr& response) {
  if (!response->result.ok) {
    record_failure(report_, response->result);
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

BT::NodeStatus SyncObjectsNode::onFailure(BT::ServiceNodeErrorCode error) {
  record_failure(report_, failure_result("scene_apply", std::string("SyncObjects ") + BT::toStr(error)));
  return BT::NodeStatus::FAILURE;
}

HomeNode::HomeNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                   std::shared_ptr<ExecutionReport> report)
    : BT::RosActionNode<Action>(name, config, params), report_(std::move(report)) {}

BT::PortsList HomeNode::providedPorts() { return providedBasicPorts({}); }

bool HomeNode::setGoal(Goal&) {
  report_->operation = "Home";
  report_->object_id.clear();
  report_->phase.clear();
  return true;
}

BT::NodeStatus HomeNode::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  report_->phase = feedback->phase;
  RCLCPP_INFO(logger(), "Home phase=%s", feedback->phase.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus HomeNode::onResultReceived(const WrappedResult& result) {
  return record_result(report_, result.result->result);
}

BT::NodeStatus HomeNode::onFailure(BT::ActionNodeErrorCode error) { return report_error(report_, "home", error); }

BT::NodeStatus HomeNode::onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) {
  if (result && !result->result->result.ok) {
    record_failure(report_, result->result->result);
    return BT::NodeStatus::FAILURE;
  }
  return report_error(report_, "home", error);
}

SortNode::SortNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                   std::shared_ptr<ExecutionReport> report)
    : BT::RosActionNode<Action>(name, config, params), report_(std::move(report)) {}

BT::PortsList SortNode::providedPorts() { return providedBasicPorts({BT::InputPort<SortJob>("job")}); }

bool SortNode::setGoal(Goal& goal) {
  SortJob job;
  getInput("job", job);
  report_->operation = "Sort";
  report_->object_id = job.object_id;
  report_->phase.clear();
  goal.object_id = job.object_id;
  goal.destination = job.destination;
  return true;
}

BT::NodeStatus SortNode::onFeedback(const std::shared_ptr<const Feedback> feedback) {
  report_->phase = feedback->phase;
  RCLCPP_INFO(logger(), "Sort object=%s phase=%s", report_->object_id.c_str(), feedback->phase.c_str());
  return BT::NodeStatus::RUNNING;
}

BT::NodeStatus SortNode::onResultReceived(const WrappedResult& result) {
  const auto status = record_result(report_, result.result->result);
  if (status == BT::NodeStatus::SUCCESS) {
    ++report_->completed_jobs;
  }
  return status;
}

BT::NodeStatus SortNode::onFailure(BT::ActionNodeErrorCode error) { return report_error(report_, "sort", error); }

BT::NodeStatus SortNode::onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) {
  if (result && !result->result->result.ok) {
    record_failure(report_, result->result->result);
    return BT::NodeStatus::FAILURE;
  }
  return report_error(report_, "sort", error);
}

void register_policy_nodes(BT::BehaviorTreeFactory& factory, std::shared_ptr<AdaptiveCycleState> state,
                           std::shared_ptr<ExecutionReport> report) {
  factory.registerNodeType<RemainingCountNode>("RemainingCount", state, report);
  factory.registerNodeType<PlanNextJobNode>("PlanNextJob", state, report);
  factory.registerNodeType<CommitPlacedJobNode>("CommitPlacedJob", std::move(state), std::move(report));
}

}  // namespace sorting_arm_executive
