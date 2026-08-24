#include "sorting_arm_executive/executive_node.hpp"

#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <future>
#include <memory>
#include <set>
#include <stdexcept>
#include <string>

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
  const auto object_count = declare_parameter<std::int64_t>("object_count");
  if (object_count < 2 || static_cast<std::size_t>(object_count) > kTrayCapacity) {
    throw std::runtime_error("object_count must be between 2 and the tray capacity (" + std::to_string(kTrayCapacity) + ")");
  }
  object_count_ = static_cast<std::size_t>(object_count);
  readiness_timeout_s_ = declare_parameter<double>("readiness_timeout_s");
  detect_timeout_s_ = declare_parameter<double>("detect_timeout_s");
  sync_timeout_s_ = declare_parameter<double>("sync_timeout_s");
  cancel_timeout_s_ = declare_parameter<double>("cancel_timeout_s");
  report_->total_jobs = object_count_;

  detect_client_ = create_client<sorting_arm_interfaces::srv::DetectObjects>("detect_objects");
  sync_client_ = create_client<sorting_arm_interfaces::srv::SyncObjects>("sync_objects");
  controller_client_ = create_client<controller_manager_msgs::srv::ListControllers>("/controller_manager/list_controllers");
}

void ExecutiveNode::initialize() {
  const auto cycle_state = std::make_shared<AdaptiveCycleState>(object_count_);
  register_policy_nodes(factory_, cycle_state, report_);

  auto make_params = [this](const std::string& name, double timeout_s) {
    BT::RosNodeParams params;
    params.nh = shared_from_this();
    params.default_port_value = name;
    params.server_timeout = std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(timeout_s));
    return params;
  };
  factory_.registerNodeType<DetectObjectsNode>("DetectObjects", make_params("detect_objects", detect_timeout_s_), report_);
  factory_.registerNodeType<SyncObjectsNode>("SyncObjects", make_params("sync_objects", sync_timeout_s_), report_);
  factory_.registerNodeType<HomeNode>("Home", make_params("home", cancel_timeout_s_), report_);
  factory_.registerNodeType<SortNode>("Sort", make_params("sort", cancel_timeout_s_), report_);

  // build the tree only after readiness (see check_readiness) - each RosActionNode probes its
  // server in its constructor, so building here would log "not reachable" before the servers are up
  tree_path_ = ament_index_cpp::get_package_share_directory("sorting_arm_executive") + "/behavior_trees/sorting_cycle.xml";
  readiness_deadline_ = deadline_after(readiness_timeout_s_);
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
    controllers_ready_ = controller_response_is_ready(*response);
  }

  if (!controllers_ready_ && !controller_request_ && controller_client_->service_is_ready() &&
      now >= next_controller_query_) {
    controller_request_.emplace(
        controller_client_->async_send_request(std::make_shared<controller_manager_msgs::srv::ListControllers::Request>()));
    next_controller_query_ = now + 200ms;
  }

  // sync_objects shares skill_server_node with home/pick/place; building the tree here, not in
  // initialize, lets each action node's constructor probe find its server instead of logging at cold start
  const bool endpoints_ready = detect_client_->service_is_ready() && sync_client_->service_is_ready();
  if (controllers_ready_ && endpoints_ready) {
    auto blackboard = BT::Blackboard::create();
    blackboard->set("object_count", static_cast<int>(object_count_));
    try {
      tree_ = factory_.createTreeFromFile(tree_path_, blackboard);
    } catch (const std::exception& error) {
      fail_startup("BehaviorTree construction failed: " + std::string(error.what()));
      return;
    }
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
  // every failing leaf records into report_, so has_failure holds on any tree FAILURE
  RCLCPP_ERROR(get_logger(),
               "sorting cycle failed: completed=%zu total=%zu operation=%s object=%s phase=%s native_code=%d message=%s",
               report_->completed_jobs, report_->total_jobs, report_->operation.c_str(), report_->object_id.c_str(),
               report_->failure.phase.c_str(), report_->failure.native_code, report_->failure.message.c_str());
}

}  // namespace sorting_arm_executive

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sorting_arm_executive::ExecutiveNode>();
    node->initialize();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sorting_arm_executive"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
