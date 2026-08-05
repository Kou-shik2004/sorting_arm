#pragma once

#include <vector>

#include "rclcpp/node.hpp"
#include "sorting_arm_executive/types.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"

namespace sorting_arm_executive {

struct ExecutiveConfig {
  std::vector<DestinationSlot> destination_slots;
  double readiness_timeout_s = 0.0;
  double detect_timeout_s = 0.0;
  double sync_timeout_s = 0.0;
  double action_timeout_s = 0.0;
  double cancel_timeout_s = 0.0;
};

ExecutiveConfig load_executive_config(rclcpp::Node& node);
std::vector<sorting_arm_interfaces::msg::DetectedObject> load_fixed_objects(rclcpp::Node& node);

}  // namespace sorting_arm_executive
