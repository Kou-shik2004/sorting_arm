#include <algorithm>
#include <exception>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_executive/assignment_planner.hpp"
#include "sorting_arm_executive/config.hpp"
#include "sorting_arm_interfaces/srv/detect_objects.hpp"

namespace sorting_arm_executive {

class FixedObjectProvider : public rclcpp::Node {
 public:
  FixedObjectProvider() : Node("fixed_object_provider") {
    const auto config = load_executive_config(*this);
    objects_ = load_fixed_objects(*this);
    if (objects_.size() != config.cycle_object_count) {
      throw std::runtime_error("fixed object count must equal cycle_object_count");
    }
    const AssignmentPlanner planner(config.destination_slots);
    const auto validation = planner.plan(objects_);
    if (!validation.ok) {
      throw std::runtime_error("fixed object configuration failed validation: " + validation.message);
    }
    std::sort(objects_.begin(), objects_.end(), [](const auto& first, const auto& second) {
      if (first.centre.pose.position.x != second.centre.pose.position.x) {
        return first.centre.pose.position.x < second.centre.pose.position.x;
      }
      return first.centre.pose.position.y < second.centre.pose.position.y;
    });
    service_ = create_service<sorting_arm_interfaces::srv::DetectObjects>(
        "detect_objects", [this](std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Request> request,
                                 std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Response> response) {
          const std::size_t expected_count = request->expected_count;
          if (expected_count == 0U || expected_count > objects_.size()) {
            response->result.ok = false;
            response->result.phase = "detect";
            response->result.message = "requested fixed-object count is unavailable";
            return;
          }
          if (!previous_expected_count_.has_value()) {
            if (expected_count != objects_.size()) {
              response->result.ok = false;
              response->result.phase = "detect";
              response->result.message = "first fixed observation must request every configured object";
              return;
            }
          } else {
            if (expected_count + 1U != *previous_expected_count_) {
              response->result.ok = false;
              response->result.phase = "detect";
              response->result.message = "fixed observations must decrease by one object per cycle";
              return;
            }
            objects_.erase(objects_.begin());
          }
          response->objects = objects_;
          const auto stamp = now();
          for (auto& object : response->objects) {
            object.centre.header.stamp = stamp;
          }
          response->result.ok = true;
          response->result.phase = "detect";
          previous_expected_count_ = expected_count;
        });
  }

 private:
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects_;
  std::optional<std::size_t> previous_expected_count_;
  rclcpp::Service<sorting_arm_interfaces::srv::DetectObjects>::SharedPtr service_;
};

}  // namespace sorting_arm_executive

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sorting_arm_executive::FixedObjectProvider>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("fixed_object_provider"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
