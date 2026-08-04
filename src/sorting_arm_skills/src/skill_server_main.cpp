#include <memory>
#include <stdexcept>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sorting_arm_interfaces/srv/sync_objects.hpp"
#include "sorting_arm_skills/gripper_commander.hpp"
#include "sorting_arm_skills/helpers.hpp"
#include "sorting_arm_skills/home_server.hpp"
#include "sorting_arm_skills/motion_commander.hpp"
#include "sorting_arm_skills/pick_server.hpp"
#include "sorting_arm_skills/place_server.hpp"
#include "sorting_arm_skills/scene_manager.hpp"
#include "sorting_arm_skills/skill_state.hpp"

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<rclcpp::Node>("skill_server_node");

  try {
    sorting_arm::MotionCommander motion(node);
    sorting_arm::GripperCommander gripper(node);
    sorting_arm::SceneManager scene(node);

    const auto scene_result = scene.apply_static_scene();
    if (!scene_result.ok) {
      RCLCPP_FATAL(node->get_logger(), "static scene apply failed: %s", scene_result.message.c_str());
      throw std::runtime_error("sorting_arm_skills: " + scene_result.message);
    }

    // shared across all four servers: one manipulation goal at a time (D6),
    // and the object id Pick hands to Place
    sorting_arm::SkillState state;

    // servers are created after motion/scene/gripper/state and the static
    // collision scene all already exist — nothing can accept a goal before then
    sorting_arm::PickServerNode pick_server(node, motion, scene, gripper, state);
    sorting_arm::PlaceServerNode place_server(node, motion, scene, gripper, state);
    sorting_arm::HomeServerNode home_server(node, motion, state);

    // SyncObjects is a small, stateless service call — it doesn't need a
    // class of its own, just the lock_if_idle() guard sync needs to be safe
    // against a running Pick/Place/Home worker touching SceneManager at the
    // same time.
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

    rclcpp::spin(node);
  } catch (const std::exception& e) {
    RCLCPP_FATAL(node->get_logger(), "startup failed: %s", e.what());
    rclcpp::shutdown();
    return 1;
  }

  rclcpp::shutdown();
  return 0;
}
