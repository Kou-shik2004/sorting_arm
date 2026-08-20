#pragma once

#include <map>
#include <optional>
#include <string>
#include <vector>

#include "geometry_msgs/msg/pose_stamped.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "rclcpp/node.hpp"
#include "sorting_arm_interfaces/msg/detected_object.hpp"
#include "sorting_arm_skills/types.hpp"

namespace sorting_arm {

// Owns the one PlanningSceneInterface for the whole process (D6). Scene
// mutations use MoveIt's synchronous apply calls, never timing sleeps. Grasp
// attach/detach and grasp-contact allowances now live inside the MTC task and
// its scene diffs (D31), not here.
class SceneManager {
 public:
  explicit SceneManager(rclcpp::Node::SharedPtr node);

  // Applies the table and both trays as compound collision objects. Called
  // once at server startup.
  SkillResult apply_static_scene();

  // Reconciles the world with one detection snapshot: adds/updates every
  // object in `objects`, removes any stale id. Never touches table/tray.
  SkillResult sync_objects(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects);

  // Ids of the static support surfaces (table and both trays) a grasped object
  // may rest against.
  std::vector<std::string> support_surface_ids() const;

  // World-frame centre and half-height of a synced dynamic object, or nullopt
  // if the object has not been synced.
  struct ObjectGeometry {
    geometry_msgs::msg::PoseStamped centre;
    double half_height_m = 0.0;
  };
  std::optional<ObjectGeometry> known_object_geometry(const std::string& object_id) const;

 private:
  moveit_msgs::msg::CollisionObject load_box_set(const std::string& prefix);

  rclcpp::Node::SharedPtr node_;

  std::vector<moveit_msgs::msg::CollisionObject> static_objects_;

  moveit::planning_interface::PlanningSceneInterface scene_interface_;

  // Geometry of every object this manager has synced, keyed by id — populated
  // by sync_objects(), never by configuration.
  std::map<std::string, moveit_msgs::msg::CollisionObject> known_dynamic_objects_;
};

}  // namespace sorting_arm
