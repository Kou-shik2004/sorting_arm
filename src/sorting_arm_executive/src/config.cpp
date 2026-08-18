#include "sorting_arm_executive/config.hpp"

#include <cmath>
#include <cstddef>
#include <cstdint>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "shape_msgs/msg/solid_primitive.hpp"

namespace sorting_arm_executive {
namespace {

void require_positive(const std::string& name, double value) {
  if (!std::isfinite(value) || value <= 0.0) {
    throw std::runtime_error(name + " must be positive and finite");
  }
}

void require_same_size(const std::string& group, std::size_t expected,
                       const std::vector<std::pair<std::string, std::size_t>>& sizes) {
  for (const auto& entry : sizes) {
    if (entry.second != expected) {
      throw std::runtime_error(group + "." + entry.first + " must contain " + std::to_string(expected) + " values");
    }
  }
}

geometry_msgs::msg::PoseStamped world_centre(double x, double y, double z) {
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "world";
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  pose.pose.position.z = z;
  pose.pose.orientation.w = 1.0;
  return pose;
}

}  // namespace

ExecutiveConfig load_executive_config(rclcpp::Node& node) {
  ExecutiveConfig config;
  const auto cycle_object_count = node.declare_parameter<std::int64_t>("cycle_object_count");
  if (cycle_object_count <= 0) {
    throw std::runtime_error("cycle_object_count must be positive");
  }
  config.cycle_object_count = static_cast<std::size_t>(cycle_object_count);
  config.readiness_timeout_s = node.declare_parameter<double>("readiness_timeout_s");
  config.detect_timeout_s = node.declare_parameter<double>("detect_timeout_s");
  config.sync_timeout_s = node.declare_parameter<double>("sync_timeout_s");
  config.cancel_timeout_s = node.declare_parameter<double>("cancel_timeout_s");
  require_positive("readiness_timeout_s", config.readiness_timeout_s);
  require_positive("detect_timeout_s", config.detect_timeout_s);
  require_positive("sync_timeout_s", config.sync_timeout_s);
  require_positive("cancel_timeout_s", config.cancel_timeout_s);

  const auto labels = node.declare_parameter<std::vector<std::string>>("destination_slots.labels");
  const auto x = node.declare_parameter<std::vector<double>>("destination_slots.centre_x");
  const auto y = node.declare_parameter<std::vector<double>>("destination_slots.centre_y");
  const auto z = node.declare_parameter<std::vector<double>>("destination_slots.centre_z");
  if (labels.empty()) {
    throw std::runtime_error("destination_slots.labels must not be empty");
  }
  require_same_size("destination_slots", labels.size(),
                    {{"centre_x", x.size()}, {"centre_y", y.size()}, {"centre_z", z.size()}});
  for (std::size_t index = 0; index < labels.size(); ++index) {
    config.destination_slots.push_back(DestinationSlot{labels[index], world_centre(x[index], y[index], z[index])});
  }
  if (config.cycle_object_count > config.destination_slots.size()) {
    throw std::runtime_error("cycle_object_count exceeds destination slot capacity");
  }
  return config;
}

std::vector<sorting_arm_interfaces::msg::DetectedObject> load_fixed_objects(rclcpp::Node& node) {
  const auto ids = node.declare_parameter<std::vector<std::string>>("fixed_objects.ids");
  const auto labels = node.declare_parameter<std::vector<std::string>>("fixed_objects.labels");
  const auto x = node.declare_parameter<std::vector<double>>("fixed_objects.centre_x");
  const auto y = node.declare_parameter<std::vector<double>>("fixed_objects.centre_y");
  const auto z = node.declare_parameter<std::vector<double>>("fixed_objects.centre_z");
  const auto size_x = node.declare_parameter<std::vector<double>>("fixed_objects.size_x");
  const auto size_y = node.declare_parameter<std::vector<double>>("fixed_objects.size_y");
  const auto size_z = node.declare_parameter<std::vector<double>>("fixed_objects.size_z");
  if (ids.empty()) {
    throw std::runtime_error("fixed_objects.ids must not be empty");
  }
  require_same_size("fixed_objects", ids.size(),
                    {{"labels", labels.size()},
                     {"centre_x", x.size()},
                     {"centre_y", y.size()},
                     {"centre_z", z.size()},
                     {"size_x", size_x.size()},
                     {"size_y", size_y.size()},
                     {"size_z", size_z.size()}});

  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects;
  for (std::size_t index = 0; index < ids.size(); ++index) {
    sorting_arm_interfaces::msg::DetectedObject object;
    object.id = ids[index];
    object.label = labels[index];
    object.centre = world_centre(x[index], y[index], z[index]);
    object.primitive_type = shape_msgs::msg::SolidPrimitive::BOX;
    object.dimensions = {size_x[index], size_y[index], size_z[index]};
    objects.push_back(object);
  }
  return objects;
}

}  // namespace sorting_arm_executive
