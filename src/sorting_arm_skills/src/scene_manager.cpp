#include "sorting_arm_skills/scene_manager.hpp"

#include <chrono>
#include <cstddef>
#include <future>
#include <memory>
#include <optional>
#include <string>
#include <utility>
#include <vector>

#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
#include "shape_msgs/msg/solid_primitive.hpp"

namespace sorting_arm {

static const std::vector<std::string> kTouchLinks{"robotiq_85_left_finger_link", "robotiq_85_left_finger_tip_link",
                                                  "robotiq_85_right_finger_link", "robotiq_85_right_finger_tip_link"};

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
  service_timeout_s_ = node_->declare_parameter<double>("scene.service_timeout_s", 5.0);

  static_objects_.push_back(load_box_set("scene.table"));
  static_objects_.push_back(load_box_set("scene.red_tray"));
  static_objects_.push_back(load_box_set("scene.blue_tray"));

  get_scene_client_ = node_->create_client<moveit_msgs::srv::GetPlanningScene>("get_planning_scene");
}

SkillResult SceneManager::query_allowed_collision_matrix(moveit_msgs::msg::AllowedCollisionMatrix& matrix) {
  const auto timeout = std::chrono::duration<double>(service_timeout_s_);
  if (!get_scene_client_->wait_for_service(timeout)) {
    return skill_error("grasp_contacts", "get_planning_scene service unavailable");
  }

  auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
  request->components.components = moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX;
  auto future = get_scene_client_->async_send_request(request);
  if (future.wait_for(timeout) != std::future_status::ready) {
    return skill_error("grasp_contacts", "get_planning_scene response timed out");
  }

  const auto response = future.get();
  matrix = response->scene.allowed_collision_matrix;
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::apply_allowed_collision_matrix(const moveit_msgs::msg::AllowedCollisionMatrix& matrix,
                                                         const std::string& operation) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix = matrix;
  if (!scene_interface_.applyPlanningScene(scene)) {
    return skill_error("grasp_contacts", operation + ": applyPlanningScene failed");
  }
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::begin_grasp_contacts(const std::string& object_id) {
  if (grasp_contacts_baseline_) {
    return skill_error("grasp_contacts", "a grasp-contact allowance is already active");
  }

  moveit_msgs::msg::AllowedCollisionMatrix baseline;
  const auto query_result = query_allowed_collision_matrix(baseline);
  if (!query_result.ok) {
    return query_result;
  }

  collision_detection::AllowedCollisionMatrix modified(baseline);
  modified.setEntry(object_id, kTouchLinks, true);
  moveit_msgs::msg::AllowedCollisionMatrix modified_msg;
  modified.getMessage(modified_msg);

  const auto apply_result = apply_allowed_collision_matrix(modified_msg, "enable grasp contacts");
  if (!apply_result.ok) {
    return apply_result;
  }

  grasp_contacts_baseline_ = std::move(baseline);
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::end_grasp_contacts() {
  const auto restore_result = apply_allowed_collision_matrix(*grasp_contacts_baseline_, "restore grasp contacts");
  if (!restore_result.ok) {
    return restore_result;
  }

  grasp_contacts_baseline_.reset();
  return skill_ok("grasp_contacts");
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

SkillResult SceneManager::attach_at_pose(const std::string& object_id,
                                         const geometry_msgs::msg::PoseStamped& object_centre) {
  moveit_msgs::msg::CollisionObject object = known_dynamic_objects_.at(object_id);
  object.header.frame_id = "world";
  object.primitive_poses.front() = object_centre.pose;

  moveit_msgs::msg::AttachedCollisionObject attached;
  attached.link_name = "tcp";
  attached.touch_links = kTouchLinks;
  attached.object = object;
  attached.object.operation = moveit_msgs::msg::CollisionObject::ADD;

  if (!scene_interface_.applyAttachedCollisionObject(attached)) {
    return skill_error("attach", "applyAttachedCollisionObject failed for '" + object_id + "'");
  }

  known_dynamic_objects_[object_id] = std::move(object);
  return skill_ok("attach");
}

SkillResult SceneManager::detach_and_place(const std::string& object_id,
                                           const geometry_msgs::msg::PoseStamped& placed_centre) {
  moveit_msgs::msg::AttachedCollisionObject detach_msg;
  detach_msg.link_name = "tcp";
  detach_msg.object.id = object_id;
  detach_msg.object.header.frame_id = "world";
  detach_msg.object.operation = moveit_msgs::msg::CollisionObject::REMOVE;
  if (!scene_interface_.applyAttachedCollisionObject(detach_msg)) {
    return skill_error("detach_reinsert", "detaching '" + object_id + "' failed");
  }

  moveit_msgs::msg::CollisionObject reinsert = known_dynamic_objects_.at(object_id);
  reinsert.header.frame_id = "world";
  reinsert.operation = moveit_msgs::msg::CollisionObject::ADD;
  reinsert.primitive_poses.front() = placed_centre.pose;
  if (!scene_interface_.applyCollisionObject(reinsert)) {
    return skill_error("detach_reinsert", "reinserting '" + object_id + "' failed");
  }

  known_dynamic_objects_[object_id] = std::move(reinsert);
  return skill_ok("detach_reinsert");
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
  geometry.centre.pose = object.primitive_poses.front();
  geometry.half_height_m = primitive.dimensions[shape_msgs::msg::SolidPrimitive::BOX_Z] / 2.0;
  return geometry;
}

}  // namespace sorting_arm
