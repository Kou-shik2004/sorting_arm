#include "sorting_arm_skills/scene_manager.hpp"

#include <cstddef>
#include <optional>
#include <stdexcept>

#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

namespace {

// Reads one compound box set (table or a tray: id + six parallel box arrays)
// and builds its CollisionObject. See config/skills.yaml for what the numbers mean.
moveit_msgs::msg::CollisionObject load_box_set(rclcpp::Node& node, const std::string& prefix,
                                               const std::string& frame_id) {
  const std::vector<double> no_boxes;
  const auto id = node.declare_parameter<std::string>(prefix + ".id", "");
  const auto size_x = node.declare_parameter<std::vector<double>>(prefix + ".box_size_x", no_boxes);
  const auto size_y = node.declare_parameter<std::vector<double>>(prefix + ".box_size_y", no_boxes);
  const auto size_z = node.declare_parameter<std::vector<double>>(prefix + ".box_size_z", no_boxes);
  const auto centre_x = node.declare_parameter<std::vector<double>>(prefix + ".box_centre_x", no_boxes);
  const auto centre_y = node.declare_parameter<std::vector<double>>(prefix + ".box_centre_y", no_boxes);
  const auto centre_z = node.declare_parameter<std::vector<double>>(prefix + ".box_centre_z", no_boxes);

  if (id.empty()) {
    throw std::runtime_error(prefix + ".id must be non-empty");
  }
  const std::size_t n = size_x.size();
  const bool consistent = n > 0 && size_y.size() == n && size_z.size() == n && centre_x.size() == n &&
                          centre_y.size() == n && centre_z.size() == n;
  if (!consistent) {
    throw std::runtime_error(prefix + ": box_size_*/box_centre_* arrays must all be the same, non-zero length");
  }

  std::vector<BoxPrimitivePose> boxes;
  for (std::size_t i = 0; i < n; ++i) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = centre_x[i];
    pose.position.y = centre_y[i];
    pose.position.z = centre_z[i];
    boxes.push_back(BoxPrimitivePose{size_x[i], size_y[i], size_z[i], pose});
  }
  return make_compound_box_object(id, frame_id, boxes);
}

}  // namespace

SceneManager::SceneManager(std::shared_ptr<rclcpp::Node> node, const std::string& planning_frame,
                           const std::string& tcp_link) {
  node_ = node;
  planning_frame_ = planning_frame;
  tcp_link_ = tcp_link;

  touch_links_ = node_->declare_parameter<std::vector<std::string>>("scene.touch_links", std::vector<std::string>{});
  if (touch_links_.empty()) {
    throw std::runtime_error("scene.touch_links must list at least one gripper touch link");
  }

  static_objects_.push_back(load_box_set(*node_, "scene.table", planning_frame_));
  static_objects_.push_back(load_box_set(*node_, "scene.red_tray", planning_frame_));
  static_objects_.push_back(load_box_set(*node_, "scene.blue_tray", planning_frame_));

  scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
}

SkillResult SceneManager::apply_static_scene() {
  if (!scene_interface_->applyCollisionObjects(static_objects_)) {
    return skill_error("scene_apply", "applyCollisionObjects failed for the static table/tray scene");
  }

  std::vector<std::string> ids;
  for (const auto& object : static_objects_) {
    ids.push_back(object.id);
  }
  const auto known = scene_interface_->getObjects(ids);
  if (known.size() != ids.size()) {
    return skill_error("scene_apply", "static scene objects missing from a getObjects query after apply");
  }
  return skill_ok("scene_apply");
}

SkillResult SceneManager::sync_objects(const std::vector<sorting_arm_interfaces::msg::DetectedObject>& objects) {
  std::map<std::string, moveit_msgs::msg::CollisionObject> new_objects;
  for (const auto& detected : objects) {
    if (detected.id.empty()) {
      return skill_error("scene_apply", "a detected object has an empty id");
    }
    if (!validate_pose(detected.centre, planning_frame_)) {
      return skill_error("scene_apply", "object '" + detected.id + "' centre pose failed frame/finite validation");
    }
    // SolidPrimitive bounds dimensions to 3; reject, don't truncate silently.
    if (detected.dimensions.size() > 3) {
      return skill_error("scene_apply", "object '" + detected.id + "' has more than 3 primitive dimensions");
    }

    moveit_msgs::msg::CollisionObject collision_object;
    collision_object.header.frame_id = planning_frame_;
    collision_object.id = detected.id;
    collision_object.operation = moveit_msgs::msg::CollisionObject::ADD;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = detected.primitive_type;
    primitive.dimensions.assign(detected.dimensions.begin(), detected.dimensions.end());
    collision_object.primitives.push_back(primitive);
    collision_object.primitive_poses.push_back(detected.centre.pose);

    new_objects.emplace(detected.id, collision_object);
  }

  std::vector<moveit_msgs::msg::CollisionObject> to_apply;
  for (const auto& entry : new_objects) {
    to_apply.push_back(entry.second);
  }

  std::vector<std::string> stale_ids;
  for (const auto& entry : known_dynamic_objects_) {
    if (new_objects.find(entry.first) == new_objects.end()) {
      moveit_msgs::msg::CollisionObject remove_object;
      remove_object.header.frame_id = planning_frame_;
      remove_object.id = entry.first;
      remove_object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
      to_apply.push_back(remove_object);
      stale_ids.push_back(entry.first);
    }
  }

  if (!scene_interface_->applyCollisionObjects(to_apply)) {
    return skill_error("scene_apply", "applyCollisionObjects failed while syncing detected objects");
  }

  std::vector<std::string> expected_ids;
  for (const auto& entry : new_objects) {
    expected_ids.push_back(entry.first);
  }
  const auto present = scene_interface_->getObjects(expected_ids);
  if (present.size() != new_objects.size()) {
    return skill_error("scene_apply", "not every synced object was present in a getObjects query after apply");
  }

  for (const auto& id : stale_ids) {
    known_dynamic_objects_.erase(id);
  }
  for (auto& [id, object] : new_objects) {
    known_dynamic_objects_[id] = object;
  }
  return skill_ok("scene_apply");
}

SkillResult SceneManager::attach(const std::string& object_id) {
  const auto it = known_dynamic_objects_.find(object_id);
  if (it == known_dynamic_objects_.end()) {
    return skill_error("attach", "object '" + object_id + "' was never synced into the scene");
  }

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = tcp_link_;
  attached.touch_links = touch_links_;
  attached.object = it->second;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (!scene_interface_->applyAttachedCollisionObject(attached)) {
    return skill_error("attach", "applyAttachedCollisionObject failed for '" + object_id + "'");
  }

  const auto attached_now = scene_interface_->getAttachedObjects({object_id});
  const auto world_now = scene_interface_->getObjects({object_id});
  if (attached_now.count(object_id) != 1 || world_now.count(object_id) != 0) {
    return skill_error("attach", "attach did not produce attached-present/world-absent for '" + object_id + "'");
  }
  return skill_ok("attach");
}

SkillResult SceneManager::detach_and_place(const std::string& object_id,
                                           const geometry_msgs::msg::PoseStamped& placed_centre) {
  const auto it = known_dynamic_objects_.find(object_id);
  if (it == known_dynamic_objects_.end()) {
    return skill_error("detach_reinsert", "object '" + object_id + "' has no known geometry to reinsert");
  }
  if (!validate_pose(placed_centre, planning_frame_)) {
    return skill_error("detach_reinsert", "placed centre pose failed frame/finite validation");
  }

  moveit_msgs::msg::AttachedCollisionObject detach_msg;
  detach_msg.link_name = tcp_link_;
  detach_msg.object.id = object_id;
  detach_msg.object.header.frame_id = planning_frame_;
  detach_msg.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!scene_interface_->applyAttachedCollisionObject(detach_msg)) {
    return skill_error("detach_reinsert", "detaching '" + object_id + "' failed");
  }

  moveit_msgs::msg::CollisionObject reinsert = it->second;
  reinsert.header.frame_id = planning_frame_;
  reinsert.operation = moveit_msgs::msg::CollisionObject::ADD;
  // every dynamic object synced through sync_objects has exactly one primitive
  if (!reinsert.primitive_poses.empty()) {
    reinsert.primitive_poses[0] = placed_centre.pose;
  }
  if (!scene_interface_->applyCollisionObject(reinsert)) {
    return skill_error("detach_reinsert", "reinserting '" + object_id + "' failed");
  }

  const auto attached_now = scene_interface_->getAttachedObjects({object_id});
  const auto world_now = scene_interface_->getObjects({object_id});
  if (attached_now.count(object_id) != 0 || world_now.count(object_id) != 1) {
    return skill_error("detach_reinsert",
                       "detach/reinsert did not produce world-present/attached-absent for '" + object_id + "'");
  }

  known_dynamic_objects_[object_id] = reinsert;
  return skill_ok("detach_reinsert");
}

std::optional<SceneManager::ObjectGeometry> SceneManager::known_object_geometry(const std::string& object_id) const {
  const auto it = known_dynamic_objects_.find(object_id);
  if (it == known_dynamic_objects_.end()) {
    return std::nullopt;
  }
  const auto& object = it->second;
  if (object.primitives.size() != 1 || object.primitive_poses.size() != 1) {
    return std::nullopt;
  }
  const auto& primitive = object.primitives.front();
  if (primitive.type != shape_msgs::msg::SolidPrimitive::BOX ||
      primitive.dimensions.size() <= shape_msgs::msg::SolidPrimitive::BOX_Z) {
    return std::nullopt;
  }

  ObjectGeometry geometry;
  geometry.centre.header.frame_id = object.header.frame_id;
  geometry.centre.pose = object.primitive_poses.front();
  geometry.half_height_m = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2.0;
  return geometry;
}

}  // namespace sorting_arm
