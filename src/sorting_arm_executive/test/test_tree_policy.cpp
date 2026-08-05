#include <gtest/gtest.h>

#include <cstdint>
#include <memory>
#include <string>
#include <utility>
#include <vector>

#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"
#include "sorting_arm_executive/bt_nodes.hpp"

namespace sorting_arm_executive {
namespace {

struct FakeContext {
  std::vector<std::string> trace;
  std::vector<std::vector<sorting_arm_interfaces::msg::DetectedObject>> observations;
  std::vector<std::uint32_t> expected_counts;
  std::vector<std::vector<sorting_arm_interfaces::msg::DetectedObject>> synced_objects;
  std::string failing_step;
};

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

AssignmentPlanner planner() {
  return AssignmentPlanner(
      {DestinationSlot{"red", centre(0.58, 0.38, 0.525)}, DestinationSlot{"red", centre(0.58, 0.32, 0.525)},
       DestinationSlot{"blue", centre(0.58, -0.32, 0.525)}, DestinationSlot{"blue", centre(0.58, -0.38, 0.525)}});
}

class FakeHome : public BT::SyncActionNode {
 public:
  FakeHome(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<FakeContext> context)
      : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() { return {}; }

 private:
  BT::NodeStatus tick() override {
    context_->trace.push_back(name());
    return context_->failing_step == name() ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<FakeContext> context_;
};

class FakeDetect : public BT::SyncActionNode {
 public:
  FakeDetect(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<FakeContext> context)
      : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::uint32_t>("expected_count"),
            BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")};
  }

 private:
  BT::NodeStatus tick() override {
    std::uint32_t expected_count = 0;
    if (!getInput("expected_count", expected_count)) {
      return BT::NodeStatus::FAILURE;
    }
    context_->expected_counts.push_back(expected_count);
    context_->trace.push_back("DetectObjects:" + std::to_string(expected_count));
    if (context_->failing_step == "DetectObjects") {
      return BT::NodeStatus::FAILURE;
    }
    const std::size_t observation_index = context_->expected_counts.size() - 1U;
    if (observation_index >= context_->observations.size()) {
      return BT::NodeStatus::FAILURE;
    }
    const auto output = setOutput("objects", context_->observations[observation_index]);
    return output ? BT::NodeStatus::SUCCESS : BT::NodeStatus::FAILURE;
  }

  std::shared_ptr<FakeContext> context_;
};

class FakeSync : public BT::SyncActionNode {
 public:
  FakeSync(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<FakeContext> context)
      : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() {
    return {BT::InputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")};
  }

 private:
  BT::NodeStatus tick() override {
    std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
    if (!getInput("objects", objects)) {
      return BT::NodeStatus::FAILURE;
    }
    context_->synced_objects.push_back(objects);
    context_->trace.push_back("SyncObjects:" + std::to_string(objects.size()));
    return context_->failing_step == "SyncObjects" ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<FakeContext> context_;
};

class FakePick : public BT::SyncActionNode {
 public:
  FakePick(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<FakeContext> context)
      : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() { return {BT::InputPort<SortJob>("job")}; }

 private:
  BT::NodeStatus tick() override {
    SortJob job;
    if (!getInput("job", job)) {
      return BT::NodeStatus::FAILURE;
    }
    const std::string step = "Pick:" + job.object_id;
    context_->trace.push_back(step);
    return context_->failing_step == step ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<FakeContext> context_;
};

class FakePlace : public BT::SyncActionNode {
 public:
  FakePlace(const std::string& name, const BT::NodeConfig& config, std::shared_ptr<FakeContext> context)
      : BT::SyncActionNode(name, config), context_(std::move(context)) {}

  static BT::PortsList providedPorts() { return {BT::InputPort<SortJob>("job")}; }

 private:
  BT::NodeStatus tick() override {
    SortJob job;
    if (!getInput("job", job)) {
      return BT::NodeStatus::FAILURE;
    }
    const std::string step = "Place:" + job.object_id;
    context_->trace.push_back(step);
    return context_->failing_step == step ? BT::NodeStatus::FAILURE : BT::NodeStatus::SUCCESS;
  }

  std::shared_ptr<FakeContext> context_;
};

BT::Tree make_tree(const std::shared_ptr<FakeContext>& context) {
  BT::BehaviorTreeFactory factory;
  const auto report = std::make_shared<ExecutionReport>();
  const auto cycle_state = std::make_shared<AdaptiveCycleState>(4U);
  register_policy_nodes(factory, planner(), cycle_state, report);
  factory.registerNodeType<FakeHome>("Home", context);
  factory.registerNodeType<FakeDetect>("DetectObjects", context);
  factory.registerNodeType<FakeSync>("SyncObjects", context);
  factory.registerNodeType<FakePick>("Pick", context);
  factory.registerNodeType<FakePlace>("Place", context);
  auto blackboard = BT::Blackboard::create();
  blackboard->set("cycle_object_count", 4);
  return factory.createTreeFromFile(SORTING_TREE_PATH, blackboard);
}

BT::NodeStatus run_tree(BT::Tree& tree) {
  BT::NodeStatus status = BT::NodeStatus::IDLE;
  do {
    status = tree.tickExactlyOnce();
  } while (status == BT::NodeStatus::RUNNING);
  return status;
}

std::shared_ptr<FakeContext> context() {
  auto result = std::make_shared<FakeContext>();
  result->observations = {
      {object("box_1", "blue", 0.40, -0.12), object("box_2", "red", 0.40, 0.12), object("box_3", "red", 0.52, -0.12),
       object("box_4", "blue", 0.52, 0.12)},
      {object("box_1", "red", 0.30, 0.22), object("box_2", "red", 0.52, -0.12), object("box_3", "blue", 0.52, 0.12)},
      {object("box_1", "red", 0.52, -0.12), object("box_2", "blue", 0.52, 0.12)},
      {object("box_1", "blue", 0.52, 0.12)}};
  return result;
}

TEST(TreePolicy, ReobservesAfterEveryPlacedObject) {
  auto fake = context();
  auto tree = make_tree(fake);

  EXPECT_EQ(run_tree(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->expected_counts, (std::vector<std::uint32_t>{4U, 3U, 2U, 1U}));
  EXPECT_EQ(fake->trace,
            (std::vector<std::string>{"ObservationHome",    "DetectObjects:4",    "SyncObjects:4",      "Pick:scan_1_box_1",
                                      "Place:scan_1_box_1", "ReobserveHome",      "DetectObjects:3",    "SyncObjects:4",
                                      "Pick:scan_2_box_1",  "Place:scan_2_box_1", "ReobserveHome",      "DetectObjects:2",
                                      "SyncObjects:4",      "Pick:scan_3_box_1",  "Place:scan_3_box_1", "ReobserveHome",
                                      "DetectObjects:1",    "SyncObjects:4",      "Pick:scan_4_box_1",  "Place:scan_4_box_1",
                                      "ReobserveHome"}));
  ASSERT_EQ(fake->synced_objects.size(), 4U);
  EXPECT_EQ(fake->synced_objects[1][0].id, "scan_1_box_1");
  EXPECT_DOUBLE_EQ(fake->synced_objects[1][0].centre.pose.position.x, 0.58);
  EXPECT_EQ(fake->synced_objects[1][1].id, "scan_2_box_1");
  EXPECT_DOUBLE_EQ(fake->synced_objects[1][1].centre.pose.position.x, 0.30);
}

TEST(TreePolicy, FirstChildFailureStopsEveryLaterOperation) {
  auto fake = context();
  fake->failing_step = "Pick:scan_2_box_1";
  auto tree = make_tree(fake);

  EXPECT_EQ(run_tree(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fake->trace, (std::vector<std::string>{"ObservationHome", "DetectObjects:4", "SyncObjects:4",
                                                   "Pick:scan_1_box_1", "Place:scan_1_box_1", "ReobserveHome",
                                                   "DetectObjects:3", "SyncObjects:4", "Pick:scan_2_box_1"}));
}

}  // namespace
}  // namespace sorting_arm_executive
