#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"
#include "sorting_arm_skills/helpers.hpp"
#include "sorting_arm_skills/home_server.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/mtc_pick_place.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"
#include "sorting_arm_skills/sort_server.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("skill_server_node");

  try {
    // MotionCommander first: it raw-declares the planning.* params MtcPickPlace
    // then reads with declare_or_get. SceneManager before the planner so its
    // support-surface ids are ready to hand over.
    sorting_arm::MotionCommander motion(node);
    sorting_arm::SceneManager scene(node);
    sorting_arm::MtcPickPlace planner(node, scene.support_surface_ids());

    const auto scene_result = scene.apply_static_scene();
    if (!scene_result.ok) {
      RCLCPP_FATAL(node->get_logger(), "static scene apply failed: %s", scene_result.message.c_str());
      throw std::runtime_error("sorting_arm_skills: " + scene_result.message);
    }

    // shared across both servers: one manipulation goal at a time (D6)
    sorting_arm::SkillState state;

    // servers are created after motion/scene/planner/state and the static
    // collision scene all already exist — nothing can accept a goal before then
    sorting_arm::SortServerNode sort_server(node, planner, scene, state);
    sorting_arm::HomeServerNode home_server(node, motion, state);

    // SyncObjects is a small, stateless service call — it doesn't need a class
    // of its own, just the lock_if_idle() guard sync needs to be safe against a
    // running Sort/Home worker touching SceneManager at the same time.
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

    // MultiThreadedExecutor: MTC plan()/execute() block the worker jthread while
    // move_group service and action replies arrive as callbacks on this node —
    // a single-threaded spin would deadlock.
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
