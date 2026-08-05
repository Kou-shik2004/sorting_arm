#include <gtest/gtest.h>

#include <string>
#include <utility>
#include <vector>

#include "shape_msgs/msg/solid_primitive.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"

namespace sorting_arm_executive {
namespace {

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

std::vector<sorting_arm_interfaces::msg::DetectedObject> snapshot() {
  return {object("box_1", "blue", 0.40, -0.12), object("box_2", "red", 0.40, 0.12), object("box_3", "red", 0.52, -0.12),
          object("box_4", "blue", 0.52, 0.12)};
}

TEST(AssignmentPlanner, AllocatesFourObjectsInSnapshotAndSlotOrder) {
  const auto result = planner().plan(snapshot());

  ASSERT_TRUE(result.ok) << result.message;
  ASSERT_EQ(result.jobs.size(), 4U);
  EXPECT_EQ(result.jobs[0].object_id, "box_1");
  EXPECT_DOUBLE_EQ(result.jobs[0].destination.pose.position.y, -0.32);
  EXPECT_EQ(result.jobs[1].object_id, "box_2");
  EXPECT_DOUBLE_EQ(result.jobs[1].destination.pose.position.y, 0.38);
  EXPECT_EQ(result.jobs[2].object_id, "box_3");
  EXPECT_DOUBLE_EQ(result.jobs[2].destination.pose.position.y, 0.32);
  EXPECT_EQ(result.jobs[3].object_id, "box_4");
  EXPECT_DOUBLE_EQ(result.jobs[3].destination.pose.position.y, -0.38);
}

TEST(AssignmentPlanner, InsufficientMatchingCapacityReturnsNoJobs) {
  auto objects = snapshot();
  objects[3].label = "red";

  const auto result = planner().plan(objects);

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.jobs.empty());
  EXPECT_NE(result.message.find("no unused destination"), std::string::npos);
}

TEST(AssignmentPlanner, DuplicateIdReturnsNoJobs) {
  auto objects = snapshot();
  objects[1].id = objects[0].id;

  const auto result = planner().plan(objects);

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.jobs.empty());
}

TEST(AssignmentPlanner, UnsupportedLabelReturnsNoJobs) {
  auto objects = snapshot();
  objects[0].label = "green";

  const auto result = planner().plan(objects);

  EXPECT_FALSE(result.ok);
  EXPECT_TRUE(result.jobs.empty());
}

TEST(AssignmentPlanner, RepeatedInputProducesIdenticalJobs) {
  const auto first = planner().plan(snapshot());
  const auto second = planner().plan(snapshot());

  ASSERT_TRUE(first.ok);
  ASSERT_TRUE(second.ok);
  ASSERT_EQ(first.jobs.size(), second.jobs.size());
  for (std::size_t index = 0; index < first.jobs.size(); ++index) {
    EXPECT_EQ(first.jobs[index].object_id, second.jobs[index].object_id);
    EXPECT_EQ(first.jobs[index].label, second.jobs[index].label);
    EXPECT_EQ(first.jobs[index].destination, second.jobs[index].destination);
  }
}

}  // namespace
}  // namespace sorting_arm_executive
