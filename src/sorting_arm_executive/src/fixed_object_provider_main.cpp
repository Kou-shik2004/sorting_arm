#include <algorithm>
#include <exception>
#include <memory>
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
        "detect_objects", [this](std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Request>,
                                 std::shared_ptr<sorting_arm_interfaces::srv::DetectObjects::Response> response) {
          response->objects = objects_;
          const auto stamp = now();
          for (auto& object : response->objects) {
            object.centre.header.stamp = stamp;
          }
          response->result.ok = true;
          response->result.phase = "detect";
        });
  }

 private:
  std::vector<sorting_arm_interfaces::msg::DetectedObject> objects_;
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
