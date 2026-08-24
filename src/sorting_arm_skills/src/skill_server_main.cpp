#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"
#include "sorting_arm_skills/home_server.hpp"
#include "sorting_arm_skills/mtc_pick_place.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/sort_server.hpp"
#include "sorting_arm_skills/types.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("skill_server_node");

  try {
    // SceneManager before the planner so its support-surface ids are ready to hand over.
    sorting_arm::SceneManager scene;
    sorting_arm::MtcPickPlace planner(node, scene.support_surface_ids());

    const auto scene_result = scene.apply_static_scene();
    if (!scene_result.ok) {
      RCLCPP_FATAL(node->get_logger(), "static scene apply failed: %s", scene_result.message.c_str());
      throw std::runtime_error("sorting_arm_skills: " + scene_result.message);
    }

    // shared across both servers: one manipulation goal at a time (D6)
    sorting_arm::SkillState state;

    // created after scene/planner/state exist, so no goal can arrive too early
    sorting_arm::SortServerNode sort_server(node, planner, scene, state);
    sorting_arm::HomeServerNode home_server(node, state);

    // stateless service; lock_if_idle() keeps a running worker off SceneManager
    auto sync_objects_service = node->create_service<sorting_arm_interfaces::srv::SyncObjects>(
        "sync_objects", [&scene, &state](std::shared_ptr<sorting_arm_interfaces::srv::SyncObjects::Request> request,
                                         std::shared_ptr<sorting_arm_interfaces::srv::SyncObjects::Response> response) {
          auto lock = state.lock_if_idle();
          if (!lock) {
            response->result = sorting_arm::to_msg(
                sorting_arm::skill_error("scene_apply", "cannot sync objects while a manipulation goal is active"));
            return;
          }

          response->result = sorting_arm::to_msg(scene.sync_objects(request->objects));
        });

    // MultiThreaded: MTC plan/execute block the worker while move_group replies
    // arrive as callbacks here — a single-threaded spin would deadlock.
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
