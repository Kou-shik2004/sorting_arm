#pragma once

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/allowed_collision_matrix.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "moveit_msgs/srv/get_planning_scene.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one PlanningSceneInterface for the whole process (D6). Every
// mutation here is apply-then-query, never apply-then-sleep.
class SceneManager {
 public:
  SceneManager(std::shared_ptr<rclcpp::Node> node, const std::string& planning_frame, const std::string& tcp_link);

  // Applies the table and both trays as compound collision objects and
  // verifies all three ids are present. Called once at server startup.
  SkillResult apply_static_scene();

  // Reconciles the world with one detection snapshot: adds/updates every
  // object in `objects`, removes any stale id. Never touches table/tray.
  SkillResult sync_objects(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects);

  // Attaches a previously-synced object to the tcp link, then verifies
  // attached-present/world-absent.
  SkillResult attach(const std::string& object_id);

  // we allow only the target/touch-link pairs near grasp, then restore the
  // exact matrix so table and pedestal checks never disappear
  SkillResult begin_grasp_contacts(const std::string& object_id);
  SkillResult end_grasp_contacts();

  // Detaches and reinserts the object at placed_centre using its originally-
  // synced geometry, then verifies world-present/attached-absent.
  SkillResult detach_and_place(const std::string& object_id, const geometry_msgs::msg::PoseStamped& placed_centre);

  // World-frame centre and half-height of a synced dynamic object, or
  // nullopt if unsynced or not a box.
  struct ObjectGeometry {
    geometry_msgs::msg::PoseStamped centre;
    double half_height_m = 0.0;
  };
  std::optional<ObjectGeometry> known_object_geometry(const std::string& object_id) const;

 private:
  SkillResult query_allowed_collision_matrix(moveit_msgs::msg::AllowedCollisionMatrix& matrix);
  SkillResult apply_and_verify_allowed_collision_matrix(const moveit_msgs::msg::AllowedCollisionMatrix& matrix,
                                                        const std::string& operation);

  std::shared_ptr<rclcpp::Node> node_;
  std::string planning_frame_;
  std::string tcp_link_;
  std::vector<std::string> touch_links_;
  double service_timeout_s_ = 0.0;

  std::vector<moveit_msgs::msg::CollisionObject> static_objects_;

  std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> scene_interface_;
  rclcpp::Client<moveit_msgs::srv::GetPlanningScene>::SharedPtr get_scene_client_;

  std::optional<moveit_msgs::msg::AllowedCollisionMatrix> grasp_contacts_baseline_;
  std::optional<std::string> grasp_contacts_object_id_;

  // Geometry of every object this manager has synced, keyed by id — populated
  // by sync_objects(), never by configuration.
  std::map<std::string, moveit_msgs::msg::CollisionObject> known_dynamic_objects_;
};

}  // namespace sorting_arm
