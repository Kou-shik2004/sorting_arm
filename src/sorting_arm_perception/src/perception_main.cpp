#include <exception>
#include <memory>
#include <rclcpp/rclcpp.hpp>

#include "sorting_arm_perception/perception_node.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);

  try {
    auto node = std::make_shared<sorting_arm_perception::PerceptionNode>();
    rclcpp::executors::MultiThreadedExecutor executor(rclcpp::ExecutorOptions(), 2);
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& exception) {
    RCLCPP_FATAL(rclcpp::get_logger("camera_object_provider"), "startup failed: %s", exception.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
