#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>

#include "behaviortree_cpp/bt_factory.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_client.hpp"
#include "rclcpp_action/create_server.hpp"
#include "sorting_arm_executive/bt_nodes.hpp"

namespace sorting_arm_executive {
namespace {

using namespace std::chrono_literals;
using Pick = sorting_arm_interfaces::action::Pick;
using ServerGoalHandle = rclcpp_action::ServerGoalHandle<Pick>;

class PickEndpoint : public rclcpp::Node {
 public:
  explicit PickEndpoint(bool abort_goal) : Node("pick_transport_endpoint"), abort_goal_(abort_goal) {
    server_ = rclcpp_action::create_server<Pick>(
        this, "pick",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [this](std::shared_ptr<ServerGoalHandle>) {
          cancel_seen_ = true;
          return rclcpp_action::CancelResponse::ACCEPT;
        },
        [this](std::shared_ptr<ServerGoalHandle> goal_handle) {
          goal_handle_ = std::move(goal_handle);
          if (abort_goal_) {
            auto result = std::make_shared<Pick::Result>();
            result->result.ok = false;
            result->result.native_code = 73;
            result->result.phase = "close_gripper";
            result->result.message = "controlled missed grasp";
            goal_handle_->abort(result);
            terminal_sent_ = true;
          }
        });
  }

  void complete_cancellation() {
    if (!cancel_seen_ || terminal_sent_ || goal_handle_ == nullptr) {
      return;
    }
    auto result = std::make_shared<Pick::Result>();
    result->result.ok = false;
    result->result.phase = "cancel";
    result->result.message = "controlled cancellation";
    goal_handle_->canceled(result);
    terminal_sent_ = true;
  }

  bool cancel_seen() const { return cancel_seen_; }

 private:
  bool abort_goal_ = false;
  bool cancel_seen_ = false;
  bool terminal_sent_ = false;
  std::shared_ptr<ServerGoalHandle> goal_handle_;
  rclcpp_action::Server<Pick>::SharedPtr server_;
};

class TransportTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

BT::Tree pick_tree(const rclcpp::Node::SharedPtr& client_node, const std::shared_ptr<ExecutionReport>& report,
                   double action_timeout_s, double cancel_timeout_s) {
  auto client = rclcpp_action::create_client<Pick>(client_node, "pick");
  BT::BehaviorTreeFactory factory;
  factory.registerNodeType<PickNode>("Pick", client, action_timeout_s, cancel_timeout_s, report, client_node->get_logger(),
                                     client_node->get_clock());
  auto blackboard = BT::Blackboard::create();
  SortJob job;
  job.object_id = "box_1";
  job.label = "red";
  blackboard->set("job", job);
  return factory.createTreeFromText(
      R"(<root BTCPP_format="4"><BehaviorTree ID="Main"><Pick job="{job}"/></BehaviorTree></root>)", blackboard);
}

BT::NodeStatus drive(BT::Tree& tree, rclcpp::executors::SingleThreadedExecutor& executor, PickEndpoint& endpoint) {
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  const auto deadline = std::chrono::steady_clock::now() + 2s;
  while (std::chrono::steady_clock::now() < deadline) {
    status = tree.tickExactlyOnce();
    executor.spin_once(1ms);
    endpoint.complete_cancellation();
    if (status != BT::NodeStatus::RUNNING) {
      return status;
    }
  }
  return status;
}

TEST_F(TransportTest, AbortedPickPreservesEmbeddedSkillResult) {
  auto endpoint = std::make_shared<PickEndpoint>(true);
  auto client_node = std::make_shared<rclcpp::Node>("pick_transport_client_abort");
  auto report = std::make_shared<ExecutionReport>();
  auto tree = pick_tree(client_node, report, 1.0, 0.2);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(endpoint);
  executor.add_node(client_node);

  EXPECT_EQ(drive(tree, executor, *endpoint), BT::NodeStatus::FAILURE);
  ASSERT_TRUE(report->has_failure);
  EXPECT_EQ(report->failure.native_code, 73);
  EXPECT_EQ(report->failure.phase, "close_gripper");
  EXPECT_EQ(report->failure.message, "controlled missed grasp");
}

TEST_F(TransportTest, PickTimeoutRequestsCancellationAndStops) {
  auto endpoint = std::make_shared<PickEndpoint>(false);
  auto client_node = std::make_shared<rclcpp::Node>("pick_transport_client_timeout");
  auto report = std::make_shared<ExecutionReport>();
  auto tree = pick_tree(client_node, report, 0.05, 0.2);
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(endpoint);
  executor.add_node(client_node);

  EXPECT_EQ(drive(tree, executor, *endpoint), BT::NodeStatus::FAILURE);
  EXPECT_TRUE(endpoint->cancel_seen());
  ASSERT_TRUE(report->has_failure);
  EXPECT_NE(report->failure.message.find("timed out"), std::string::npos);
}

}  // namespace
}  // namespace sorting_arm_executive
