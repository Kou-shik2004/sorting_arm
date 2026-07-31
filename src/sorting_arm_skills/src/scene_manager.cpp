#include "sorting_arm_skills/scene_manager.hpp"

#include <chrono>
#include <cstddef>
#include <future>
#include <optional>
#include <stdexcept>

#include "moveit/collision_detection/collision_matrix.hpp"
#include "moveit/planning_scene_interface/planning_scene_interface.hpp"
#include "moveit_msgs/msg/attached_collision_object.hpp"
#include "moveit_msgs/msg/planning_scene.hpp"
#include "moveit_msgs/msg/planning_scene_components.hpp"
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
  service_timeout_s_ = node_->declare_parameter<double>("scene.service_timeout_s", 5.0);
  if (touch_links_.empty()) {
    throw std::runtime_error("scene.touch_links must list at least one gripper touch link");
  }
  if (service_timeout_s_ <= 0.0) {
    throw std::runtime_error("scene.service_timeout_s must be positive");
  }

  static_objects_.push_back(load_box_set(*node_, "scene.table", planning_frame_));
  static_objects_.push_back(load_box_set(*node_, "scene.red_tray", planning_frame_));
  static_objects_.push_back(load_box_set(*node_, "scene.blue_tray", planning_frame_));

  scene_interface_ = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
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

  matrix = future.get()->scene.allowed_collision_matrix;
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::apply_and_verify_allowed_collision_matrix(
    const moveit_msgs::msg::AllowedCollisionMatrix& matrix, const std::string& operation) {
  moveit_msgs::msg::PlanningScene scene;
  scene.is_diff = true;
  scene.allowed_collision_matrix = matrix;
  if (!scene_interface_->applyPlanningScene(scene)) {
    return skill_error("grasp_contacts", operation + ": applyPlanningScene failed");
  }

  moveit_msgs::msg::AllowedCollisionMatrix observed;
  const auto query_result = query_allowed_collision_matrix(observed);
  if (!query_result.ok) {
    return skill_error("grasp_contacts", operation + ": " + query_result.message, query_result.native_code);
  }
  if (observed != matrix) {
    return skill_error("grasp_contacts", operation + ": allowed collision matrix verification failed");
  }
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::begin_grasp_contacts(const std::string& object_id) {
  if (grasp_contacts_baseline_ || grasp_contacts_object_id_) {
    return skill_error("grasp_contacts", "a grasp-contact allowance is already active");
  }
  if (known_dynamic_objects_.find(object_id) == known_dynamic_objects_.end()) {
    return skill_error("grasp_contacts", "object '" + object_id + "' was never synced into the scene");
  }
  if (scene_interface_->getObjects({object_id}).count(object_id) != 1 ||
      scene_interface_->getAttachedObjects({object_id}).count(object_id) != 0) {
    return skill_error("grasp_contacts", "object '" + object_id + "' is not world-present/attached-absent");
  }

  moveit_msgs::msg::AllowedCollisionMatrix baseline;
  const auto query_result = query_allowed_collision_matrix(baseline);
  if (!query_result.ok) {
    return query_result;
  }

  collision_detection::AllowedCollisionMatrix modified(baseline);
  modified.setEntry(object_id, touch_links_, true);
  moveit_msgs::msg::AllowedCollisionMatrix modified_msg;
  modified.getMessage(modified_msg);

  const auto apply_result = apply_and_verify_allowed_collision_matrix(modified_msg, "enable grasp contacts");
  if (!apply_result.ok) {
    const auto restore_result = apply_and_verify_allowed_collision_matrix(baseline, "rollback grasp contacts");
    if (!restore_result.ok) {
      return skill_error("grasp_contacts", apply_result.message + "; rollback failed: " + restore_result.message,
                         restore_result.native_code);
    }
    return apply_result;
  }

  grasp_contacts_baseline_ = std::move(baseline);
  grasp_contacts_object_id_ = object_id;
  return skill_ok("grasp_contacts");
}

SkillResult SceneManager::end_grasp_contacts() {
  if (!grasp_contacts_baseline_ || !grasp_contacts_object_id_) {
    return skill_error("grasp_contacts", "no grasp-contact allowance is active");
  }

  const auto restore_result =
      apply_and_verify_allowed_collision_matrix(*grasp_contacts_baseline_, "restore grasp contacts");
  if (!restore_result.ok) {
    return restore_result;
  }

  grasp_contacts_baseline_.reset();
  grasp_contacts_object_id_.reset();
  return skill_ok("grasp_contacts");
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
