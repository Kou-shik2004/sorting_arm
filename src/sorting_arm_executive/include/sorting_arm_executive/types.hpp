#pragma once

#include <cstddef>
#include <string>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sorting_arm_interfaces/msg/skill_result.hpp"

namespace sorting_arm_executive {

struct DestinationSlot {
  std::string label;
  geometry_msgs::msg::PoseStamped centre;
};

struct SortJob {
  std::string object_id;
  std::string label;
  geometry_msgs::msg::PoseStamped destination;
};

struct ExecutionReport {
  std::size_t total_jobs = 0;
  std::size_t completed_jobs = 0;
  std::string operation;
  std::string object_id;
  std::string phase;
  bool has_failure = false;
  sorting_arm_interfaces::msg::SkillResult failure;
};

}  // namespace sorting_arm_executive
