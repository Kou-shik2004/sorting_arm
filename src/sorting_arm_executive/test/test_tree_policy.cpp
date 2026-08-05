#include <gtest/gtest.h>

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
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
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
    return {BT::OutputPort<std::vector<sorting_arm_interfaces::msg::DetectedObject>>("objects")};
  }

 private:
  BT::NodeStatus tick() override {
    context_->trace.push_back("DetectObjects");
    if (context_->failing_step == "DetectObjects") {
      return BT::NodeStatus::FAILURE;
    }
    const auto output = setOutput("objects", context_->objects);
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
    context_->trace.push_back("SyncObjects");
    std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
    if (!getInput("objects", objects) || objects != context_->objects) {
      return BT::NodeStatus::FAILURE;
    }
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
  register_policy_nodes(factory, planner(), report);
  factory.registerNodeType<FakeHome>("Home", context);
  factory.registerNodeType<FakeDetect>("DetectObjects", context);
  factory.registerNodeType<FakeSync>("SyncObjects", context);
  factory.registerNodeType<FakePick>("Pick", context);
  factory.registerNodeType<FakePlace>("Place", context);
  return factory.createTreeFromFile(SORTING_TREE_PATH);
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
  result->objects = {object("box_1", "blue", 0.40, -0.12), object("box_2", "red", 0.40, 0.12),
                     object("box_3", "red", 0.52, -0.12), object("box_4", "blue", 0.52, 0.12)};
  return result;
}

TEST(TreePolicy, ExecutesCompleteOneCycleInOrder) {
  auto fake = context();
  auto tree = make_tree(fake);

  EXPECT_EQ(run_tree(tree), BT::NodeStatus::SUCCESS);
  EXPECT_EQ(fake->trace, (std::vector<std::string>{"ObservationHome", "DetectObjects", "SyncObjects", "Pick:box_1",
                                                   "Place:box_1", "Pick:box_2", "Place:box_2", "Pick:box_3", "Place:box_3",
                                                   "Pick:box_4", "Place:box_4", "FinalHome"}));
}

TEST(TreePolicy, FirstChildFailureStopsEveryLaterOperation) {
  auto fake = context();
  fake->failing_step = "Pick:box_2";
  auto tree = make_tree(fake);

  EXPECT_EQ(run_tree(tree), BT::NodeStatus::FAILURE);
  EXPECT_EQ(fake->trace, (std::vector<std::string>{"ObservationHome", "DetectObjects", "SyncObjects", "Pick:box_1",
                                                   "Place:box_1", "Pick:box_2"}));
}

}  // namespace
}  // namespace sorting_arm_executive
