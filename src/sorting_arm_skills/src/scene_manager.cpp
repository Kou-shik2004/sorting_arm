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

// Reads one compound box set (table or a tray: id + six parallel box arrays)
// and builds its CollisionObject. See config/skills.yaml for what the numbers mean.
moveit_msgs::msg::CollisionObject SceneManager::load_box_set(const std::string& prefix) {
  const std::vector<double> no_boxes;
  const auto id = node_->declare_parameter<std::string>(prefix + ".id", "");
  const auto size_x = node_->declare_parameter<std::vector<double>>(prefix + ".box_size_x", no_boxes);
  const auto size_y = node_->declare_parameter<std::vector<double>>(prefix + ".box_size_y", no_boxes);
  const auto size_z = node_->declare_parameter<std::vector<double>>(prefix + ".box_size_z", no_boxes);
  const auto centre_x = node_->declare_parameter<std::vector<double>>(prefix + ".box_centre_x", no_boxes);
  const auto centre_y = node_->declare_parameter<std::vector<double>>(prefix + ".box_centre_y", no_boxes);
  const auto centre_z = node_->declare_parameter<std::vector<double>>(prefix + ".box_centre_z", no_boxes);

  const std::size_t n = size_x.size();

  moveit_msgs::msg::CollisionObject object;
  object.header.frame_id = "world";
  object.id = id;
  object.operation = moveit_msgs::msg::CollisionObject::ADD;

  for (std::size_t i = 0; i < n; ++i) {
    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = shape_msgs::msg::SolidPrimitive::BOX;
    primitive.dimensions = {size_x[i], size_y[i], size_z[i]};
    geometry_msgs::msg::Pose pose;
    pose.position.x = centre_x[i];
    pose.position.y = centre_y[i];
    pose.position.z = centre_z[i];
    pose.orientation.w = 1.0;
    object.primitives.push_back(primitive);
    object.primitive_poses.push_back(pose);
  }
  return object;
}

SceneManager::SceneManager(rclcpp::Node::SharedPtr node) : node_(std::move(node)) {
  static_objects_.push_back(load_box_set("scene.table"));
  static_objects_.push_back(load_box_set("scene.red_tray"));
  static_objects_.push_back(load_box_set("scene.blue_tray"));
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
    // the object frame sits at the cube centre so MTC's GenerateGraspPose samples
    // grasps at the object, not at the world origin
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
