#include "sorting_arm_executive/bt_nodes.hpp"

#include <algorithm>
#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <utility>

namespace sorting_arm_executive {
namespace {

using namespace std::chrono_literals;

// action/service progress is measured in sim time, so timeout deadlines have to be too -
// a steady_clock deadline shrinks under a slow-RTF sim and fires for no real reason.
rclcpp::Time deadline_after(const rclcpp::Clock::SharedPtr& clock, double seconds) {
  return clock->now() + rclcpp::Duration::from_seconds(seconds);
}

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

// RosActionNode routes only SUCCEEDED here; a server can still finish cleanly with a
// logical ok=false, so a non-ok payload is a FAILURE carrying that message.
template <typename WrappedResult>
BT::NodeStatus report_success_result(const std::shared_ptr<ExecutionReport>& report, const WrappedResult& wrapped) {
  if (wrapped.result->result.ok) {
    return BT::NodeStatus::SUCCESS;
  }
  record_failure(report, wrapped.result->result);
  return BT::NodeStatus::FAILURE;
}

BT::NodeStatus report_error(const std::shared_ptr<ExecutionReport>& report, const std::string& phase,
                            BT::ActionNodeErrorCode error) {
  record_failure(report, failure_result(phase, phase + " " + BT::toStr(error)));
  return BT::NodeStatus::FAILURE;
}

// abort/cancel carry the server's real SkillResult; a transport error (reject/timeout/
// unreachable) has no payload, so fall back to the error-code name.
template <typename WrappedResult>
BT::NodeStatus report_failure(const std::shared_ptr<ExecutionReport>& report, const std::string& phase,
                              BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) {
  if (result && !result->result->result.ok) {
    record_failure(report, result->result->result);
    return BT::NodeStatus::FAILURE;
  }
  return report_error(report, phase, error);
}

}  // namespace

RemainingCountNode::RemainingCountNode(const std::string& name, const BT::NodeConfig& config,
                                       std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report)
    : BT::SyncActionNode(name, config), state_(std::move(state)), report_(std::move(report)) {}

BT::PortsList RemainingCountNode::providedPorts() { return {BT::OutputPort<std::uint32_t>("expected_count")}; }

BT::NodeStatus RemainingCountNode::tick() {
  report_->operation = "RemainingCount";
  if (state_->remaining_count == 0U || state_->pending_job) {
    record_failure(report_, failure_result("cycle_state", "remaining-count state is inconsistent"));
    return BT::NodeStatus::FAILURE;
  }
  const auto output = setOutput("expected_count", static_cast<std::uint32_t>(state_->remaining_count));
  if (!output) {
    record_failure(report_, failure_result("cycle_state", output.error()));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::SUCCESS;
}

PlanNextJobNode::PlanNextJobNode(const std::string& name, const BT::NodeConfig& config, AssignmentPlanner planner,
                                 std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report)
    : BT::SyncActionNode(name, config), planner_(std::move(planner)), state_(std::move(state)), report_(std::move(report)) {}

BT::PortsList PlanNextJobNode::providedPorts() {
  return {BT::InputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects"), BT::OutputPort<SortJob>("job"),
          BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("scene_objects")};
}

BT::NodeStatus PlanNextJobNode::tick() {
  report_->operation = "PlanNextJob";
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
  const auto input = getInput("objects", objects);
  if (!input) {
    record_failure(report_, failure_result("allocation", input.error()));
    return BT::NodeStatus::FAILURE;
  }
  if (objects.size() != state_->remaining_count) {
    record_failure(report_, failure_result("allocation", "detection count does not match remaining cycle count"));
    return BT::NodeStatus::FAILURE;
  }

  const std::string prefix = "scan_" + std::to_string(state_->scan_number) + "_";
  for (auto& object : objects) {
    object.id = prefix + object.id;
  }
  const auto allocation = planner_.plan(objects, state_->used_destination_slots);
  if (!allocation.ok) {
    record_failure(report_, failure_result("allocation", allocation.message));
    return BT::NodeStatus::FAILURE;
  }
  const SortJob& job = allocation.jobs.front();
  const auto selected =
      std::find_if(objects.begin(), objects.end(), [&job](const auto& object) { return object.id == job.object_id; });
  if (selected == objects.end()) {
    record_failure(report_, failure_result("allocation", "allocated object is absent from the detected snapshot"));
    return BT::NodeStatus::FAILURE;
  }

  std::vector<sorting_arm_interfaces::msg::DetectedObject> scene_objects = state_->placed_objects;
  scene_objects.insert(scene_objects.end(), objects.begin(), objects.end());
  const auto job_output = setOutput("job", job);
  const auto scene_output = setOutput("scene_objects", scene_objects);
  if (!job_output || !scene_output) {
    record_failure(report_, failure_result("allocation", !job_output ? job_output.error() : scene_output.error()));
    return BT::NodeStatus::FAILURE;
  }
  state_->pending_job = AdaptiveCycleState::PendingJob{job, *selected};
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
  const auto input = getInput("job", job);
  if (!input) {
    record_failure(report_, failure_result("cycle_state", input.error()));
    return BT::NodeStatus::FAILURE;
  }
  if (!state_->pending_job || state_->pending_job->job.object_id != job.object_id ||
      state_->pending_job->job.destination_slot_index != job.destination_slot_index) {
    record_failure(report_, failure_result("cycle_state", "placed job does not match the planned object"));
    return BT::NodeStatus::FAILURE;
  }

  auto placed_object = state_->pending_job->object;
  placed_object.centre = job.destination;
  state_->placed_objects.push_back(std::move(placed_object));
  state_->used_destination_slots.insert(job.destination_slot_index);
  --state_->remaining_count;
  state_->pending_job.reset();
  return BT::NodeStatus::SUCCESS;
}

DetectObjectsNode::DetectObjectsNode(const std::string& name, const BT::NodeConfig& config,
                                     rclcpp::Client<Service>::SharedPtr client, double timeout_s,
                                     std::shared_ptr<ExecutionReport> report, rclcpp::Clock::SharedPtr clock)
    : BT::StatefulActionNode(name, config),
      client_(std::move(client)),
      timeout_s_(timeout_s),
      report_(std::move(report)),
      clock_(std::move(clock)) {}

BT::PortsList DetectObjectsNode::providedPorts() {
  return {BT::InputPort<std::uint32_t>("expected_count"),
          BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")};
}

BT::NodeStatus DetectObjectsNode::onStart() {
  report_->operation = "DetectObjects";
  report_->object_id.clear();
  report_->phase.clear();
  std::uint32_t expected_count = 0;
  const auto input = getInput("expected_count", expected_count);
  if (!input || expected_count == 0U) {
    record_failure(report_, failure_result("detect", input ? "expected_count must be positive" : input.error()));
    return BT::NodeStatus::FAILURE;
  }
  auto request = std::make_shared<Service::Request>();
  request->expected_count = expected_count;
  try {
    pending_.emplace(client_->async_send_request(request));
    deadline_ = deadline_after(clock_, timeout_s_);
    return BT::NodeStatus::RUNNING;
  } catch (const std::exception& error) {
    record_failure(report_, failure_result("detect", "DetectObjects request failed: " + std::string(error.what())));
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus DetectObjectsNode::onRunning() {
  if (pending_->future.wait_for(0s) == std::future_status::ready) {
    const auto response = pending_->future.get();
    pending_.reset();
    if (response == nullptr) {
      record_failure(report_, failure_result("detect", "DetectObjects returned no response"));
      return BT::NodeStatus::FAILURE;
    }
    if (!response->result.ok) {
      record_failure(report_, response->result);
      return BT::NodeStatus::FAILURE;
    }
    const auto output = setOutput("objects", response->objects);
    if (!output) {
      record_failure(report_, failure_result("detect", output.error()));
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
  if (clock_->now() >= deadline_) {
    client_->remove_pending_request(*pending_);
    pending_.reset();
    record_failure(report_, failure_result("detect", "DetectObjects response timed out"));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void DetectObjectsNode::onHalted() {
  if (pending_) {
    client_->remove_pending_request(*pending_);
    pending_.reset();
  }
}

SyncObjectsNode::SyncObjectsNode(const std::string& name, const BT::NodeConfig& config,
                                 rclcpp::Client<Service>::SharedPtr client, double timeout_s,
                                 std::shared_ptr<ExecutionReport> report, rclcpp::Clock::SharedPtr clock)
    : BT::StatefulActionNode(name, config),
      client_(std::move(client)),
      timeout_s_(timeout_s),
      report_(std::move(report)),
      clock_(std::move(clock)) {}

BT::PortsList SyncObjectsNode::providedPorts() {
  return {BT::InputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")};
}

BT::NodeStatus SyncObjectsNode::onStart() {
  report_->operation = "SyncObjects";
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
  const auto input = getInput("objects", objects);
  if (!input) {
    record_failure(report_, failure_result("scene_apply", input.error()));
    return BT::NodeStatus::FAILURE;
  }
  auto request = std::make_shared<Service::Request>();
  request->objects = std::move(objects);
  try {
    pending_.emplace(client_->async_send_request(request));
    deadline_ = deadline_after(clock_, timeout_s_);
    return BT::NodeStatus::RUNNING;
  } catch (const std::exception& error) {
    record_failure(report_, failure_result("scene_apply", "SyncObjects request failed: " + std::string(error.what())));
    return BT::NodeStatus::FAILURE;
  }
}

BT::NodeStatus SyncObjectsNode::onRunning() {
  if (pending_->future.wait_for(0s) == std::future_status::ready) {
    const auto response = pending_->future.get();
    pending_.reset();
    if (response == nullptr) {
      record_failure(report_, failure_result("scene_apply", "SyncObjects returned no response"));
      return BT::NodeStatus::FAILURE;
    }
    if (!response->result.ok) {
      record_failure(report_, response->result);
      return BT::NodeStatus::FAILURE;
    }
    return BT::NodeStatus::SUCCESS;
  }
  if (clock_->now() >= deadline_) {
    client_->remove_pending_request(*pending_);
    pending_.reset();
    record_failure(report_, failure_result("scene_apply", "SyncObjects response timed out"));
    return BT::NodeStatus::FAILURE;
  }
  return BT::NodeStatus::RUNNING;
}

void SyncObjectsNode::onHalted() {
  if (pending_) {
    client_->remove_pending_request(*pending_);
    pending_.reset();
  }
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

BT::NodeStatus HomeNode::onResultReceived(const WrappedResult& result) { return report_success_result(report_, result); }

BT::NodeStatus HomeNode::onFailure(BT::ActionNodeErrorCode error) { return report_error(report_, "home", error); }

BT::NodeStatus HomeNode::onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) {
  return report_failure(report_, "home", error, result);
}

SortNode::SortNode(const std::string& name, const BT::NodeConfig& config, const BT::RosNodeParams& params,
                   std::shared_ptr<ExecutionReport> report)
    : BT::RosActionNode<Action>(name, config, params), report_(std::move(report)) {}

BT::PortsList SortNode::providedPorts() { return providedBasicPorts({BT::InputPort<SortJob>("job")}); }

bool SortNode::setGoal(Goal& goal) {
  SortJob job;
  const auto input = getInput("job", job);
  if (!input) {
    record_failure(report_, failure_result("sort", input.error()));
    return false;
  }
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
  const auto status = report_success_result(report_, result);
  if (status == BT::NodeStatus::SUCCESS) {
    ++report_->completed_jobs;
  }
  return status;
}

BT::NodeStatus SortNode::onFailure(BT::ActionNodeErrorCode error) { return report_error(report_, "sort", error); }

BT::NodeStatus SortNode::onFailure(BT::ActionNodeErrorCode error, const std::optional<WrappedResult>& result) {
  return report_failure(report_, "sort", error, result);
}

void register_policy_nodes(BT::BehaviorTreeFactory& factory, AssignmentPlanner planner,
                           std::shared_ptr<AdaptiveCycleState> state, std::shared_ptr<ExecutionReport> report) {
  factory.registerNodeType<RemainingCountNode>("RemainingCount", state, report);
  factory.registerNodeType<PlanNextJobNode>("PlanNextJob", std::move(planner), state, report);
  factory.registerNodeType<CommitPlacedJobNode>("CommitPlacedJob", std::move(state), std::move(report));
}

}  // namespace sorting_arm_executive
