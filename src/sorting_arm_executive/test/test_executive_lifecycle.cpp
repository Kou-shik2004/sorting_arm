#include <gtest/gtest.h>

#include <chrono>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "controller_manager_msgs/msg/controller_state.hpp"
#include "controller_manager_msgs/srv/list_controllers.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/create_server.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sorting_arm_executive/executive_node.hpp"

namespace sorting_arm_executive {
namespace {

using namespace std::chrono_literals;

geometry_msgs::msg::PoseStamped centre(double x, double y, double z) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

sorting_arm_interfaces::msg::DetectedObject object(std::string id, std::string label, double x, double y) {
  sorting_arm_interfaces::msg::DetectedObject detected;
  detected.id = std::move(id);
  detected.label = std::move(label);
  detected.centre = centre(x, y, 0.52);
  detected.primitive_type = shape_msgs::msg::SolidPrimitive::BOX;
  detected.dimensions = {0.04, 0.04, 0.04};
  return detected;
}

class ApplicationEndpoints : public rclcpp::Node {
 public:
  ApplicationEndpoints() : Node("executive_lifecycle_endpoints") {
    controller_service_ = create_service<controller_manager_msgs::srv::ListControllers>(
        "/controller_manager/list_controllers",
        [](std::shared_ptr<controller_manager_msgs::srv::ListControllers::Request>,
           std::shared_ptr<controller_manager_msgs::srv::ListControllers::Response> response) {
          for (const char* name : {"joint_state_broadcaster", "arm_controller", "gripper_controller"}) {
            controller_manager_msgs::msg::ControllerState controller;
            controller.name = name;
            controller.state = "active";
            response->controller.push_back(controller);
          }
        });
    detect_service_ = create_service<sorting_arm_interfaces::srv::DetectObjects>(
        "detect_objects", [this](std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Request>,
                                 std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Response> response) {
          trace_.push_back("DetectObjects");
          response->objects = {object("box_1", "blue", 0.40, -0.12), object("box_2", "red", 0.40, 0.12),
                               object("box_3", "red", 0.52, -0.12), object("box_4", "blue", 0.52, 0.12)};
          response->result.ok = true;
        });
    sync_service_ = create_service<sorting_arm_interfaces::srv::SyncObjects>(
        "sync_objects", [this](std::shared_ptr<sorting_arm_interfaces::srv::SyncObjects::Request> request,
                               std::shared_ptr<sorting_arm_interfaces::srv::SyncObjects::Response> response) {
          trace_.push_back("SyncObjects:" + std::to_string(request->objects.size()));
          response->result.ok = true;
        });
    create_actions();
  }

  const std::vector<std::string>& trace() const { return trace_; }

 private:
  void create_actions() {
    using Home = sorting_arm_interfaces::action::Home;
    using HomeHandle = rclcpp_action::ServerGoalHandle<Home>;
    home_server_ = rclcpp_action::create_server<Home>(
        this, "home",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Home::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<HomeHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<HomeHandle> handle) {
          trace_.push_back("Home");
          auto result = std::make_shared<Home::Result>();
          result->result.ok = true;
          handle->succeed(result);
        });

    using Pick = sorting_arm_interfaces::action::Pick;
    using PickHandle = rclcpp_action::ServerGoalHandle<Pick>;
    pick_server_ = rclcpp_action::create_server<Pick>(
        this, "pick",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Pick::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<PickHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<PickHandle> handle) {
          trace_.push_back("Pick:" + handle->get_goal()->object_id);
          auto result = std::make_shared<Pick::Result>();
          result->result.ok = true;
          handle->succeed(result);
        });

    using Place = sorting_arm_interfaces::action::Place;
    using PlaceHandle = rclcpp_action::ServerGoalHandle<Place>;
    place_server_ = rclcpp_action::create_server<Place>(
        this, "place",
        [](const rclcpp_action::GoalUUID&, std::shared_ptr<const Place::Goal>) {
          return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
        },
        [](std::shared_ptr<PlaceHandle>) { return rclcpp_action::CancelResponse::ACCEPT; },
        [this](std::shared_ptr<PlaceHandle> handle) {
          trace_.push_back("Place:" + handle->get_goal()->object_id);
          auto result = std::make_shared<Place::Result>();
          result->result.ok = true;
          handle->succeed(result);
        });
  }

  std::vector<std::string> trace_;
  rclcpp::Service<controller_manager_msgs::srv::ListControllers>::SharedPtr controller_service_;
  rclcpp::Service<sorting_arm_interfaces::srv::DetectObjects>::SharedPtr detect_service_;
  rclcpp::Service<sorting_arm_interfaces::srv::SyncObjects>::SharedPtr sync_service_;
  rclcpp_action::Server<sorting_arm_interfaces::action::Home>::SharedPtr home_server_;
  rclcpp_action::Server<sorting_arm_interfaces::action::Pick>::SharedPtr pick_server_;
  rclcpp_action::Server<sorting_arm_interfaces::action::Place>::SharedPtr place_server_;
};

class ExecutiveLifecycleTest : public testing::Test {
 protected:
  static void SetUpTestSuite() {
    if (!rclcpp::ok()) {
      rclcpp::init(0, nullptr);
    }
  }

  static void TearDownTestSuite() { rclcpp::shutdown(); }
};

rclcpp::NodeOptions executive_options() {
  rclcpp::NodeOptions options;
  options.parameter_overrides(
      {rclcpp::Parameter("readiness_timeout_s", 1.0), rclcpp::Parameter("detect_timeout_s", 1.0),
       rclcpp::Parameter("sync_timeout_s", 1.0), rclcpp::Parameter("action_timeout_s", 1.0),
       rclcpp::Parameter("cancel_timeout_s", 0.2),
       rclcpp::Parameter("destination_slots.labels", std::vector<std::string>{"red", "red", "blue", "blue"}),
       rclcpp::Parameter("destination_slots.centre_x", std::vector<double>{0.58, 0.58, 0.58, 0.58}),
       rclcpp::Parameter("destination_slots.centre_y", std::vector<double>{0.38, 0.32, -0.32, -0.38}),
       rclcpp::Parameter("destination_slots.centre_z", std::vector<double>{0.525, 0.525, 0.525, 0.525})});
  return options;
}

TEST_F(ExecutiveLifecycleTest, RunsOneCompleteCycleThenRemainsIdle) {
  auto endpoints = std::make_shared<ApplicationEndpoints>();
  auto executive = std::make_shared<ExecutiveNode>(executive_options());
  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(endpoints);
  executor.add_node(executive);

  const auto deadline = std::chrono::steady_clock::now() + 3s;
  while (executive->cycle_state() != CycleState::succeeded && std::chrono::steady_clock::now() < deadline) {
    executor.spin_once(1ms);
  }

  ASSERT_EQ(executive->cycle_state(), CycleState::succeeded);
  EXPECT_EQ(executive->report().completed_jobs, 4U);
  EXPECT_EQ(endpoints->trace(),
            (std::vector<std::string>{"Home", "DetectObjects", "SyncObjects:4", "Pick:box_1", "Place:box_1", "Pick:box_2",
                                      "Place:box_2", "Pick:box_3", "Place:box_3", "Pick:box_4", "Place:box_4", "Home"}));

  const auto terminal_trace = endpoints->trace();
  const auto idle_deadline = std::chrono::steady_clock::now() + 100ms;
  while (std::chrono::steady_clock::now() < idle_deadline) {
    executor.spin_once(1ms);
  }
  EXPECT_EQ(endpoints->trace(), terminal_trace);
}

}  // namespace
}  // namespace sorting_arm_executive
