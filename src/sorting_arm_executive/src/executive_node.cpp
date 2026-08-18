#include "sorting_arm_executive/executive_node.hpp"

#include <array>
#include <chrono>
#include <future>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

#include "ament_index_cpp/get_package_share_directory.hpp"
#include "sorting_arm_executive/bt_nodes.hpp"

namespace sorting_arm_executive {
namespace {

using namespace std::chrono_literals;

// readiness is discovery, not simulated-world progress - stays wall time so a Gazebo that
// never comes up still times out instead of waiting on a /clock that will never publish
std::chrono::steady_clock::time_point deadline_after(double seconds) {
  const auto duration =
      std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
  return std::chrono::steady_clock::now() + duration;
}

sorting_arm_interfaces::msg::SkillResult startup_failure(const std::string& message) {
  sorting_arm_interfaces::msg::SkillResult result;
  result.ok = false;
  result.phase = "readiness";
  result.message = message;
  return result;
}

}  // namespace

ExecutiveNode::ExecutiveNode(const rclcpp::NodeOptions& options) : Node("sorting_arm_executive", options) {
  config_ = load_executive_config(*this);
  planner_.emplace(config_.destination_slots);
  report_->total_jobs = config_.cycle_object_count;

  detect_client_ = create_client<sorting_arm_interfaces::srv::DetectObjects>("detect_objects");
  sync_client_ = create_client<sorting_arm_interfaces::srv::SyncObjects>("sync_objects");
  controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
}

void ExecutiveNode::initialize() {
  const auto cycle_state = std::make_shared<AdaptiveCycleState>(config_.cycle_object_count);
  register_policy_nodes(factory_, *planner_, cycle_state, report_);
  factory_.registerNodeType<DetectObjectsNode>("DetectObjects", detect_client_, config_.detect_timeout_s, report_,
                                               get_clock());
  factory_.registerNodeType<SyncObjectsNode>("SyncObjects", sync_client_, config_.sync_timeout_s, report_, get_clock());

  const auto server_timeout =
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(config_.cancel_timeout_s));
  auto make_params = [this, server_timeout](const std::string& action_name) {
    BT::RosNodeParams params;
    params.nh = shared_from_this();
    params.default_port_value = action_name;
    params.server_timeout = server_timeout;
    return params;
  };
  factory_.registerNodeType<HomeNode>("Home", make_params("home"), report_);
  factory_.registerNodeType<PickNode>("Pick", make_params("pick"), report_);
  factory_.registerNodeType<PlaceNode>("Place", make_params("place"), report_);

  const auto tree_path =
      ament_index_cpp::get_package_share_directory("sorting_arm_executive") + "/behavior_trees/sorting_cycle.xml";
  auto blackboard = BT::Blackboard::create();
  blackboard->set("cycle_object_count", static_cast<int>(config_.cycle_object_count));
  tree_ = factory_.createTreeFromFile(tree_path, blackboard);
  readiness_deadline_ = deadline_after(config_.readiness_timeout_s);
  timer_ = create_wall_timer(20ms, [this] { tick(); });
  RCLCPP_INFO(get_logger(), "executive loaded; waiting for controllers and application endpoints");
}

CycleState ExecutiveNode::cycle_state() const { return state_; }

const ExecutionReport& ExecutiveNode::report() const { return *report_; }

void ExecutiveNode::tick() {
  if (state_ == CycleState::waiting_for_readiness) {
    check_readiness();
    return;
  }
  if (state_ != CycleState::running) {
    return;
  }

  try {
    const auto status = tree_.tickExactlyOnce();
    if (status == BT::NodeStatus::RUNNING) {
      return;
    }
    state_ = status == BT::NodeStatus::SUCCESS ? CycleState::succeeded : CycleState::failed;
    timer_->cancel();
    log_terminal(status);
  } catch (const std::exception& error) {
    fail_startup("BehaviorTree tick failed: " + std::string(error.what()));
  }
}

void ExecutiveNode::check_readiness() {
  const auto now = std::chrono::steady_clock::now();
  if (!controllers_ready_ && controller_request_ && controller_request_->future.wait_for(0s) == std::future_status::ready) {
    const auto response = controller_request_->future.get();
    controller_request_.reset();
    controllers_ready_ = response != nullptr && controller_response_is_ready(*response);
  }

  if (!controllers_ready_ && !controller_request_ && controller_client_->service_is_ready() &&
      now >= next_controller_query_) {
    controller_request_.emplace(
        controller_client_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>()));
    next_controller_query_ = now + 200ms;
  }

  // Pick/Place/Home live in the same node as sync_objects and are created before it, so
  // sync readiness stands in for the action servers; RosActionNode covers the rare residual.
  const bool endpoints_ready = detect_client_->service_is_ready() && sync_client_->service_is_ready();
  if (controllers_ready_ && endpoints_ready) {
    state_ = CycleState::running;
    RCLCPP_INFO(get_logger(), "readiness complete; starting the one sorting cycle");
    return;
  }

  if (now >= readiness_deadline_) {
    if (controller_request_) {
      controller_client_->remove_pending_request(*controller_request_);
      controller_request_.reset();
    }
    std::string missing;
    if (!controllers_ready_) {
      missing += " controllers";
    }
    if (!detect_client_->service_is_ready()) {
      missing += " detect_objects";
    }
    if (!sync_client_->service_is_ready()) {
      missing += " sync_objects";
    }
    fail_startup("readiness timeout; unavailable:" + missing);
  }
}

void ExecutiveNode::fail_startup(const std::string& message) {
  if (!report_->has_failure) {
    report_->has_failure = true;
    report_->failure = startup_failure(message);
  }
  state_ = CycleState::failed;
  timer_->cancel();
  RCLCPP_ERROR(get_logger(), "sorting cycle failed before completion: %s", message.c_str());
}

bool ExecutiveNode::controller_response_is_ready(
    const controller_manager_msgs::srv::ListControllers::Response& response) const {
  static constexpr std::array<const char*, 3> kRequiredControllers = {"joint_state_broadcaster", "arm_controller",
                                                                      "gripper_controller"};
  std::set<std::string> active;
  for (const auto& controller : response.controller) {
    if (controller.state == "active") {
      active.insert(controller.name);
    }
  }
  for (const auto* name : kRequiredControllers) {
    if (!active.contains(name)) {
      return false;
    }
  }
  return true;
}

void ExecutiveNode::log_terminal(BT::NodeStatus status) {
  if (status == BT::NodeStatus::SUCCESS) {
    RCLCPP_INFO(get_logger(), "sorting cycle succeeded: completed=%zu total=%zu; remaining idle for inspection",
                report_->completed_jobs, report_->total_jobs);
    return;
  }
  if (report_->has_failure) {
    RCLCPP_ERROR(get_logger(),
                 "sorting cycle failed: completed=%zu total=%zu operation=%s object=%s phase=%s native_code=%d message=%s",
                 report_->completed_jobs, report_->total_jobs, report_->operation.c_str(), report_->object_id.c_str(),
                 report_->failure.phase.c_str(), report_->failure.native_code, report_->failure.message.c_str());
    return;
  }
  RCLCPP_ERROR(get_logger(), "sorting cycle failed without a recorded child result");
}

}  // namespace sorting_arm_executive
