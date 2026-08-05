#include <exception>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_executive/executive_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  try {
    auto node = std::make_shared<sorting_arm_executive::ExecutiveNode>();
    rclcpp::spin(node);
  } catch (const std::exception& error) {
    RCLCPP_FATAL(rclcpp::get_logger("sorting_arm_executive"), "startup failed: %s", error.what());
    rclcpp::shutdown();
    return 1;
  }
  rclcpp::shutdown();
  return 0;
}
