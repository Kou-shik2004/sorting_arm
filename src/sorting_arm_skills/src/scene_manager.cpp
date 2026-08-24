#include "sorting_arm_skills/scene_manager.hpp"

#include <cstddef>
#include <map>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace sorting_arm {

namespace {

// one static collision box: dimensions and world-frame centre
struct StaticBox {
  double size_x;
  double size_y;
  double size_z;
  double centre_x;
  double centre_y;
  double centre_z;
};

moveit_msgs::msg::CollisionObject make_static_object(const std::string& id, const std::vector<StaticBox>& boxes) {
  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  for (const auto& box : boxes) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {box.size_x, box.size_y, box.size_z};

    geometry_msgs::msg::Pose pose;
    pose.position.x = box.centre_x;
    pose.position.y = box.centre_y;
    pose.position.z = box.centre_z;
    pose.orientation.w = 1.0;

    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
  }
  return object;
}

}  // namespace

// Fixed furniture duplicates sorting_cell.sdf for MoveIt; update both representations together.
SceneManager::SceneManager() {
  static_objects_.push_back(make_static_object("table", {{1.10, 1.00, 0.05, 0.35, 0.00, 0.475},
                                                         {0.05, 0.05, 0.45, 0.85, 0.45, 0.225},
                                                         {0.05, 0.05, 0.45, 0.85, -0.45, 0.225},
                                                         {0.05, 0.05, 0.45, -0.15, 0.45, 0.225},
                                                         {0.05, 0.05, 0.45, -0.15, -0.45, 0.225}}));

  static_objects_.push_back(make_static_object("red_tray", {{0.24, 0.24, 0.005, 0.70, 0.36, 0.5025},
                                                            {0.005, 0.24, 0.05, 0.8175, 0.36, 0.525},
                                                            {0.005, 0.24, 0.05, 0.5825, 0.36, 0.525},
                                                            {0.245, 0.005, 0.05, 0.70, 0.4775, 0.525},
                                                            {0.245, 0.005, 0.05, 0.70, 0.2425, 0.525}}));

  static_objects_.push_back(make_static_object("blue_tray", {{0.24, 0.24, 0.005, 0.70, -0.36, 0.5025},
                                                             {0.005, 0.24, 0.05, 0.8175, -0.36, 0.525},
                                                             {0.005, 0.24, 0.05, 0.5825, -0.36, 0.525},
                                                             {0.245, 0.005, 0.05, 0.70, -0.2425, 0.525},
                                                             {0.245, 0.005, 0.05, 0.70, -0.4775, 0.525}}));
}

SkillResult SceneManager::apply_static_scene() {
  if (!scene_interface_.applyCollisionObjects(static_objects_)) {
    return skill_error("scene_apply", "applyCollisionObjects failed for the static table/tray scene");
  }

  return skill_ok("scene_apply");
}

SkillResult SceneManager::sync_objects(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects) {
  std::map<std::string, moveit_msgs::msg::CollisionObject> new_objects;
  for (const auto& detected : objects) {
    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = "world";
    collision_object.id = detected.id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;
    // object frame at the cube centre so GenerateGraspPose samples at the object
    collision_object.pose = detected.centre.pose;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = detected.primitive_type;
    primitive.dimensions.assign(detected.dimensions.begin(), detected.dimensions.end());
    collision_object.primitives.push_back(primitive);

    geometry_msgs::msg::Pose primitive_pose;
    primitive_pose.orientation.w = 1.0;
    collision_object.primitive_poses.push_back(primitive_pose);

    new_objects.emplace(detected.id, collision_object);
  }

  std::vector<moveit_msgs::msg::CollisionObject> to_apply;
  for (const auto& entry : new_objects) {
    to_apply.push_back(entry.second);
  }

  for (const auto& entry : known_dynamic_objects_) {
    if (new_objects.find(entry.first) == new_objects.end()) {
      moveit_msgs::msg::CollisionObject remove_object;
      remove_object.header.frame_id = "world";
      remove_object.id = entry.first;
      remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      to_apply.push_back(remove_object);
    }
  }

  if (!to_apply.empty() && !scene_interface_.applyCollisionObjects(to_apply)) {
    return skill_error("scene_apply", "applyCollisionObjects failed while syncing detected objects");
  }

  known_dynamic_objects_ = std::move(new_objects);
  return skill_ok("scene_apply");
}

std::vector<std::string> SceneManager::support_surface_ids() const {
  std::vector<std::string> ids;
  for (const auto& object : static_objects_) {
    ids.push_back(object.id);
  }
  return ids;
}

std::optional<SceneManager::ObjectGeometry> SceneManager::known_object_geometry(const std::string& object_id) const {
  const auto it = known_dynamic_objects_.find(object_id);
  if (it == known_dynamic_objects_.end()) {
    return std::nullopt;
  }
  const auto& object = it->second;
  const auto& primitive = object.primitives.front();

  ObjectGeometry geometry;
  geometry.centre.header.frame_id = object.header.frame_id;
  geometry.centre.pose = object.pose;
  geometry.half_height_m = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2.0;
  return geometry;
}

}  // namespace sorting_arm
