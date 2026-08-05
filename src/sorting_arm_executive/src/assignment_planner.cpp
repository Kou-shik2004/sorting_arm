#include "sorting_arm_executive/assignment_planner.hpp"

#include <cmath>
#include <map>
#include <set>
#include <stdexcept>
#include <utility>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace sorting_arm_executive {
namespace {

bool finite_pose(const geometry_msgs::msg::PoseStamped& pose) {
  const auto& position = pose.pose.position;
  const auto& orientation = pose.pose.orientation;
  return std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) &&
         std::isfinite(orientation.x) && std::isfinite(orientation.y) && std::isfinite(orientation.z) &&
         std::isfinite(orientation.w) &&
         (orientation.x * orientation.x + orientation.y * orientation.y + orientation.z * orientation.z +
          orientation.w * orientation.w) > 0.0;
}

bool same_position(const geometry_msgs::msg::PoseStamped& first, const geometry_msgs::msg::PoseStamped& second) {
  return first.pose.position.x == second.pose.position.x && first.pose.position.y == second.pose.position.y &&
         first.pose.position.z == second.pose.position.z;
}

}  // namespace

AssignmentPlanner::AssignmentPlanner(std::vector<DestinationSlot> slots) : slots_(std::move(slots)) {
  if (slots_.empty()) {
    throw std::invalid_argument("destination slot list is empty");
  }
  for (std::size_t index = 0; index < slots_.size(); ++index) {
    const auto& slot = slots_[index];
    if (slot.label.empty()) {
      throw std::invalid_argument("destination slot has an empty label");
    }
    if (slot.centre.header.frame_id != "world" || !finite_pose(slot.centre)) {
      throw std::invalid_argument("destination slot has an invalid world-frame pose");
    }
    for (std::size_t earlier = 0; earlier < index; ++earlier) {
      if (same_position(slot.centre, slots_[earlier].centre)) {
        throw std::invalid_argument("destination slots contain a duplicate centre");
      }
    }
  }
}

AssignmentResult AssignmentPlanner::plan(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects) const {
  if (objects.empty()) {
    return {false, "detected snapshot is empty", {}};
  }

  std::set<std::string> object_ids;
  std::map<std::string, std::size_t> next_slot;
  std::vector<SortJob> jobs;

  for (const auto& object : objects) {
    if (object.id.empty() || !object_ids.insert(object.id).second) {
      return {false, "detected snapshot contains an empty or duplicate object id", {}};
    }
    if (object.label.empty()) {
      return {false, "object '" + object.id + "' has an empty label", {}};
    }
    if (object.centre.header.frame_id != "world" || !finite_pose(object.centre)) {
      return {false, "object '" + object.id + "' has an invalid world-frame centre", {}};
    }
    if (object.primitive_type != shape_msgs::msg::SolidPrimitive::BOX || object.dimensions.size() != 3) {
      return {false, "object '" + object.id + "' is not a three-dimensional box", {}};
    }
    for (const double dimension : object.dimensions) {
      if (!std::isfinite(dimension) || dimension <= 0.0) {
        return {false, "object '" + object.id + "' has an invalid dimension", {}};
      }
    }

    std::vector<const DestinationSlot*> matching_slots;
    for (const auto& slot : slots_) {
      if (slot.label == object.label) {
        matching_slots.push_back(&slot);
      }
    }
    auto& index = next_slot[object.label];
    if (index >= matching_slots.size()) {
      return {false, "no unused destination remains for label '" + object.label + "'", {}};
    }

    jobs.push_back(SortJob{object.id, object.label, matching_slots[index]->centre});
    ++index;
  }

  return {true, {}, std::move(jobs)};
}

}  // namespace sorting_arm_executive
