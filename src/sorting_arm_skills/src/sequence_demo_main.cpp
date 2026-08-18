#include <chrono>
#include <exception>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sorting_arm_interfaces/action/home.hpp"
#include "sorting_arm_interfaces/action/pick.hpp"
#include "sorting_arm_interfaces/action/place.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"

// One row of the objects/destinations catalogue — see config/sequence_demo.yaml.
struct ObjectSpec {
  std::string id;
  std::string label;
  double centre_x = 0.0;
  double centre_y = 0.0;
  double centre_z = 0.0;
  double size_x = 0.0;
  double size_y = 0.0;
  double size_z = 0.0;
  double destination_x = 0.0;
  double destination_y = 0.0;
  double destination_z = 0.0;
};

struct ObjectOutcome {
  std::string id;
  bool pick_ok = false;
  bool place_ok = false;
  std::string message;
};

// Home/Pick/Place all mirror the same shape (empty-or-simple goal, SkillResult
// result, string phase feedback), so one send/wait/log path covers all three —
// no shared library needed, this is the only caller.
template <typename ActionT>
bool run_action(const rclcpp::Node::SharedPtr& node, const typename rclcpp_action::Client<ActionT>::SharedPtr& client,
                const typename ActionT::Goal& goal, double timeout_s, const std::string& label,
                sorting_arm_interfaces::msg::SkillResult& out_result) {
  using GoalHandle = rclcpp_action::ClientGoalHandle<ActionT>;
  // shared so a late callback firing after a timeout writes to live state, not our stack
  auto last_phase = std::make_shared<std::string>();
  auto rejected = std::make_shared<bool>(false);
  auto result = std::make_shared<std::optional<typename GoalHandle::WrappedResult>>();

  typename rclcpp_action::Client<ActionT>::SendGoalOptions options;
  options.goal_response_callback = [node, label, rejected](typename GoalHandle::SharedPtr handle) {
    *rejected = handle == nullptr;
    RCLCPP_INFO(node->get_logger(), "%s: goal %s", label.c_str(), *rejected ? "rejected" : "accepted");
  };
  options.feedback_callback = [node, label, last_phase](typename GoalHandle::SharedPtr,
                                                        const std::shared_ptr<const typename ActionT::Feedback> feedback) {
    if (feedback->phase != *last_phase) {
      *last_phase = feedback->phase;
      RCLCPP_INFO(node->get_logger(), "%s: phase=%s", label.c_str(), last_phase->c_str());
    }
  };
  options.result_callback = [result](const typename GoalHandle::WrappedResult& wrapped) { *result = wrapped; };

  client->async_send_goal(goal, options);

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  const auto deadline = std::chrono::steady_clock::now() + std::chrono::duration_cast<std::chrono::steady_clock::duration>(
                                                               std::chrono::duration<double>(timeout_s));
  while (rclcpp::ok() && !*rejected && !result->has_value() && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(std::chrono::milliseconds(100));
  }
  executor.remove_node(node);

  if (*rejected) {
    return false;
  }
  if (!result->has_value()) {
    RCLCPP_ERROR(node->get_logger(), "%s: timed out", label.c_str());
    return false;
  }

  const auto& wrapped = result->value();
  out_result = wrapped.result->result;
  RCLCPP_INFO(node->get_logger(), "%s result: code=%d ok=%s phase=%s native_code=%d message=%s", label.c_str(),
              static_cast<int>(wrapped.code), out_result.ok ? "true" : "false", out_result.phase.c_str(),
              out_result.native_code, out_result.message.c_str());
  return wrapped.code == rclcpp_action::ResultCode::SUCCEEDED && out_result.ok;
}

// Hand-run verification harness, same mould as motion_demo: sync every
// configured object, home, then pick+place each in order, home again, print a
// summary. No routing policy, no recovery — destinations come entirely from
// config. Step 12's sorting_arm_executive replaces this; it doesn't extend it.
class SequenceDemo {
 public:
  using Home = sorting_arm_interfaces::action::Home;
  using Pick = sorting_arm_interfaces::action::Pick;
  using Place = sorting_arm_interfaces::action::Place;
  using SyncObjects = sorting_arm_interfaces::srv::SyncObjects;

  explicit SequenceDemo(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
    const std::vector<std::string> no_strings;
    const std::vector<double> no_doubles;

    const auto ids = node_->declare_parameter<std::vector<std::string>>("objects", no_strings);
    const auto labels = node_->declare_parameter<std::vector<std::string>>("object_label", no_strings);
    const auto centre_x = node_->declare_parameter<std::vector<double>>("object_centre_x", no_doubles);
    const auto centre_y = node_->declare_parameter<std::vector<double>>("object_centre_y", no_doubles);
    const auto centre_z = node_->declare_parameter<std::vector<double>>("object_centre_z", no_doubles);
    const auto size_x = node_->declare_parameter<std::vector<double>>("object_size_x", no_doubles);
    const auto size_y = node_->declare_parameter<std::vector<double>>("object_size_y", no_doubles);
    const auto size_z = node_->declare_parameter<std::vector<double>>("object_size_z", no_doubles);
    const auto dest_x = node_->declare_parameter<std::vector<double>>("destination_x", no_doubles);
    const auto dest_y = node_->declare_parameter<std::vector<double>>("destination_y", no_doubles);
    const auto dest_z = node_->declare_parameter<std::vector<double>>("destination_z", no_doubles);
    result_timeout_s_ = node_->declare_parameter<double>("result_timeout_s", 180.0);
    stop_on_failure_ = node_->declare_parameter<bool>("stop_on_failure", true);

    const std::size_t n = ids.size();
    for (std::size_t i = 0; i < n; ++i) {
      objects_.push_back(ObjectSpec{ids[i], labels[i], centre_x[i], centre_y[i], centre_z[i], size_x[i], size_y[i],
                                    size_z[i], dest_x[i], dest_y[i], dest_z[i]});
    }

    sync_client_ = node_->create_client<SyncObjects>("sync_objects");
    home_client_ = rclcpp_action::create_client<Home>(node_, "home");
    pick_client_ = rclcpp_action::create_client<Pick>(node_, "pick");
    place_client_ = rclcpp_action::create_client<Place>(node_, "place");
  }

  bool run() {
    const auto wait_timeout = std::chrono::duration<double>(result_timeout_s_);
    if (!sync_client_->wait_for_service(wait_timeout)) {
      RCLCPP_ERROR(node_->get_logger(), "sync_objects service unavailable");
      return false;
    }
    if (!home_client_->wait_for_action_server(wait_timeout) || !pick_client_->wait_for_action_server(wait_timeout) ||
        !place_client_->wait_for_action_server(wait_timeout)) {
      RCLCPP_ERROR(node_->get_logger(), "home/pick/place action server unavailable");
      return false;
    }

    if (!sync_all_objects()) {
      return false;
    }

    sorting_arm_interfaces::msg::SkillResult home_result;
    if (!run_action<Home>(node_, home_client_, Home::Goal{}, result_timeout_s_, "home", home_result)) {
      return false;
    }

    std::vector<ObjectOutcome> outcomes;
    bool all_ok = true;
    for (const auto& spec : objects_) {
      ObjectOutcome outcome;
      outcome.id = spec.id;

      Pick::Goal pick_goal;
      pick_goal.object_id = spec.id;
      sorting_arm_interfaces::msg::SkillResult pick_result;
      outcome.pick_ok = run_action<Pick>(node_, pick_client_, pick_goal, result_timeout_s_, "pick " + spec.id, pick_result);
      outcome.message = pick_result.message;

      if (outcome.pick_ok) {
        Place::Goal place_goal;
        place_goal.object_id = spec.id;
        place_goal.destination.header.frame_id = "world";
        place_goal.destination.pose.position.x = spec.destination_x;
        place_goal.destination.pose.position.y = spec.destination_y;
        place_goal.destination.pose.position.z = spec.destination_z;
        place_goal.destination.pose.orientation.w = 1.0;
        sorting_arm_interfaces::msg::SkillResult place_result;
        outcome.place_ok =
            run_action<Place>(node_, place_client_, place_goal, result_timeout_s_, "place " + spec.id, place_result);
        if (!outcome.place_ok) {
          outcome.message = place_result.message;
        }
      }

      outcomes.push_back(outcome);
      if (!(outcome.pick_ok && outcome.place_ok)) {
        all_ok = false;
        if (stop_on_failure_) {
          break;
        }
      }
    }

    sorting_arm_interfaces::msg::SkillResult final_home_result;
    run_action<Home>(node_, home_client_, Home::Goal{}, result_timeout_s_, "home", final_home_result);

    print_summary(outcomes);
    return all_ok;
  }

 private:
  bool sync_all_objects() {
    auto request = std::make_shared<SyncObjects::Request>();
    for (const auto& spec : objects_) {
      sorting_arm_interfaces::msg::DetectedObject detected;
      detected.id = spec.id;
      detected.label = spec.label;
      detected.centre.header.frame_id = "world";
      detected.centre.pose.position.x = spec.centre_x;
      detected.centre.pose.position.y = spec.centre_y;
      detected.centre.pose.position.z = spec.centre_z;
      detected.centre.pose.orientation.w = 1.0;
      detected.primitive_type = shape_msgs::msg::SolidPrimitive::BOX;
      detected.dimensions = {spec.size_x, spec.size_y, spec.size_z};
      request->objects.push_back(detected);
    }

    auto future = sync_client_->async_send_request(request);
    if (rclcpp::spin_until_future_complete(node_, future, std::chrono::duration<double>(result_timeout_s_)) !=
        rclcpp::FutureReturnCode::SUCCESS) {
      RCLCPP_ERROR(node_->get_logger(), "sync_objects response timed out");
      return false;
    }
    const auto response = future.get();
    RCLCPP_INFO(node_->get_logger(), "sync_objects result: ok=%s message=%s", response->result.ok ? "true" : "false",
                response->result.message.c_str());
    return response->result.ok;
  }

  void print_summary(const std::vector<ObjectOutcome>& outcomes) const {
    RCLCPP_INFO(node_->get_logger(), "sequence summary:");
    for (const auto& outcome : outcomes) {
      RCLCPP_INFO(node_->get_logger(), "  %s: pick=%s place=%s %s", outcome.id.c_str(), outcome.pick_ok ? "ok" : "FAIL",
                  outcome.place_ok ? "ok" : "FAIL", outcome.message.c_str());
    }
  }

  rclcpp::Node::SharedPtr node_;
  std::vector<ObjectSpec> objects_;
  double result_timeout_s_ = 0.0;
  bool stop_on_failure_ = true;
  rclcpp::Client<SyncObjects>::SharedPtr sync_client_;
  rclcpp_action::Client<Home>::SharedPtr home_client_;
  rclcpp_action::Client<Pick>::SharedPtr pick_client_;
  rclcpp_action::Client<Place>::SharedPtr place_client_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("sequence_demo");

  int exit_code = 1;
  try {
    SequenceDemo demo(node);
    exit_code = demo.run() ? 0 : 1;
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "sequence_demo failed: %s", e.what());
  }

  rclcpp::shutdown();
  return exit_code;
}
