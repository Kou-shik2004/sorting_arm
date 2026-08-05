#pragma once

#include <string>
#include <vector>

#include "sorting_arm_executive/types.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"

namespace sorting_arm_executive {

struct AssignmentResult {
  bool ok = false;
  std::string message;
  std::vector<SortJob> jobs;
};

class AssignmentPlanner {
 public:
  explicit AssignmentPlanner(std::vector<DestinationSlot> slots);

  AssignmentResult plan(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects) const;

 private:
  std::vector<DestinationSlot> slots_;
};

}  // namespace sorting_arm_executive
