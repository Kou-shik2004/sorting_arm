#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one PlanningSceneInterface (D6); synchronous applies, no sleeps.
// Grasp attach/detach and contact allowances live in the MTC task now (D31).
class SceneManager {
 public:
  SceneManager();

  // Applies table and both trays as compound collision objects, once at startup.
  SkillResult apply_static_scene();

  // Reconciles the scene with one detection snapshot; never touches table/tray.
  SkillResult sync_objects(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects);

  // Ids of the static support surfaces a grasped object may rest against.
  std::vector<std::string> support_surface_ids() const;

  // Centre and half-height of a synced object, or nullopt if never synced.
  struct ObjectGeometry {
    geometry_msgs::msg::PoseStamped centre;
    double half_height_m = 0.0;
  };
  std::optional<ObjectGeometry> known_object_geometry(const std::string& object_id) const;

 private:
  std::vector<moveit_msgs::msg::CollisionObject> static_objects_;

  moveit::planning_interface::PlanningSceneInterface scene_interface_;

  // Every synced object's geometry, keyed by id; set by sync_objects().
  std::map<std::string, moveit_msgs::msg::CollisionObject> known_dynamic_objects_;
};

}  // namespace sorting_arm
