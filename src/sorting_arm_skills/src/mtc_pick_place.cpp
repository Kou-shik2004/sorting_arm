#include "sorting_arm_skills/mtc_pick_place.hpp"

#include <Eigen/Geometry>
#include <sstream>
#include <utility>
#include <vector>

#include "geometry_msgs/msg/vector3_stamped.hpp"
#include "moveit/planning_scene/planning_scene.hpp"
#include "moveit/robot_model/robot_model.hpp"
#include "moveit/robot_state/robot_state.hpp"
#include "moveit/task_constructor/solvers.h"
#include "moveit/task_constructor/stages.h"
#include "moveit/task_constructor/task.h"
#include "moveit/trajectory_processing/time_optimal_trajectory_generation.hpp"
#include "sorting_arm_skills/helpers.hpp"

namespace sorting_arm {

namespace mtc = moveit::task_constructor;

namespace {

constexpr char kArmGroup[] = "arm";
constexpr char kGripperGroup[] = "gripper";
constexpr char kTcpFrame[] = "tcp";
constexpr char kOpenState[] = "gripper_open";

// each Cartesian approach/lift must travel at least this far to be a real motion
constexpr double kMinTravelM = 0.03;

class StoppedBoundaryTimeParameterization final : public trajectory_processing::TimeOptimalTrajectoryGeneration {
 public:
  using TimeOptimalTrajectoryGeneration::computeTimeStamps;

  bool computeTimeStamps(robot_trajectory::RobotTrajectory& trajectory, double max_velocity_scaling_factor,
                         double max_acceleration_scaling_factor) const override {
    const bool success = TimeOptimalTrajectoryGeneration::computeTimeStamps(trajectory, max_velocity_scaling_factor,
                                                                            max_acceleration_scaling_factor);
    if (!success) {
      return false;
    }

    // Each MTC stage is a separate controller goal, so Cartesian stages stop at both boundaries.
    // Jazzy TOTG can leave numerical boundary derivatives that violate that contract.
    const auto& variable_indices = trajectory.getGroup()->getVariableIndexList();
    for (auto* waypoint : {trajectory.getFirstWayPointPtr().get(), trajectory.getLastWayPointPtr().get()}) {
      for (const int variable_index : variable_indices) {
        waypoint->setVariableVelocity(variable_index, 0.0);
        waypoint->setVariableAcceleration(variable_index, 0.0);
      }
    }
    return true;
  }
};

// tcp z points down at the object; z holds the grasp height the old code used
Eigen::Isometry3d grasp_frame_transform(double half_height_m, double grasp_offset_m) {
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() = Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()).matrix();
  transform.translation() = Eigen::Vector3d(0.0, 0.0, half_height_m + grasp_offset_m);
  return transform;
}

geometry_msgs::msg::Vector3Stamped axis(const std::string& frame_id, double x, double y, double z) {
  geometry_msgs::msg::Vector3Stamped vector;
  vector.header.frame_id = frame_id;
  vector.vector.x = x;
  vector.vector.y = y;
  vector.vector.z = z;
  return vector;
}

// one active knuckle joint mimic-drives all 8 gripper links, so no group query
// returns them (moveit's JMG only walks joint_roots' descendants) — walk the
// kinematic subtree from the gripper base instead
std::vector<std::string> gripper_collision_links(const moveit::core::RobotModelConstPtr& robot_model) {
  std::vector<std::string> links;
  const auto* base_joint = robot_model->getLinkModel("robotiq_85_base_link")->getParentJointModel();
  for (const auto* link : base_joint->getDescendantLinkModels()) {
    if (!link->getShapes().empty()) {
      links.push_back(link->getName());
    }
  }
  return links;
}

std::shared_ptr<mtc::solvers::PipelinePlanner> make_sampling_planner(const rclcpp::Node::SharedPtr& node,
                                                                     double velocity_scaling, double acceleration_scaling) {
  auto planner = std::make_shared<mtc::solvers::PipelinePlanner>(node);
  planner->setMaxVelocityScalingFactor(velocity_scaling);
  planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  return planner;
}

std::shared_ptr<mtc::solvers::CartesianPath> make_cartesian_planner(double velocity_scaling, double acceleration_scaling,
                                                                    double eef_step_m) {
  auto planner = std::make_shared<mtc::solvers::CartesianPath>();
  planner->setTimeParameterization(std::make_shared<StoppedBoundaryTimeParameterization>());
  planner->setMaxVelocityScalingFactor(velocity_scaling);
  planner->setMaxAccelerationScalingFactor(acceleration_scaling);
  planner->setStepSize(eef_step_m);
  return planner;
}

// task properties + current state, shared by both tasks. Returns the
// current-state stage so a generator can monitor it. Mimic normalization is a
// separate stage each task places for itself — the place task must allow the
// grasp contact before this collision-checking stage runs.
mtc::Stage* add_task_prefix(mtc::Task& task, const rclcpp::Node::SharedPtr& node) {
  task.loadRobotModel(node);
  task.setProperty("group", kArmGroup);
  task.setProperty("eef", kGripperGroup);
  task.setProperty("ik_frame", kTcpFrame);

  auto current = std::make_unique<mtc::stages::CurrentState>("current state");
  mtc::Stage* current_ptr = current.get();
  task.add(std::move(current));

  return current_ptr;
}

// Gazebo doesn't hold the URDF mimic joints exactly; the drift crosses MTC Connect's
// 1e-4 tolerance after a few cycles, so rewrite each follower from its master. This
// stage collision-checks its output, so it must run after any needed ACM allowance.
std::unique_ptr<mtc::stages::ModifyPlanningScene> make_normalize_mimics() {
  auto mimics = std::make_unique<mtc::stages::ModifyPlanningScene>("normalize gripper mimics");
  mimics->setCallback([](const planning_scene::PlanningScenePtr& scene, const mtc::PropertyMap&) {
    auto& state = scene->getCurrentStateNonConst();
    for (const auto* jm : state.getRobotModel()->getJointModels()) {
      const auto* master = jm->getMimic();
      if (master == nullptr) {
        continue;
      }
      double value = *state.getJointPositions(master) * jm->getMimicFactor() + jm->getMimicOffset();
      state.setJointPositions(jm, &value);
    }
    state.update();
  });
  return mimics;
}

}  // namespace

MtcPickPlace::MtcPickPlace(rclcpp::Node::SharedPtr node, std::vector<std::string> support_surfaces)
    : node_(std::move(node)), support_surfaces_(std::move(support_surfaces)), gripper_(node_) {
  approach_height_m_ = declare_or_get<double>(*node_, "targets.approach_height_m", 0.12);
  retreat_height_m_ = declare_or_get<double>(*node_, "targets.retreat_height_m", 0.12);
  grasp_offset_m_ = declare_or_get<double>(*node_, "targets.grasp_offset_m", -0.036);
  grasp_angle_delta_rad_ = declare_or_get<double>(*node_, "targets.grasp_angle_delta_rad", M_PI / 12.0);
  velocity_scaling_ = declare_or_get<double>(*node_, "planning.velocity_scaling", 0.15);
  acceleration_scaling_ = declare_or_get<double>(*node_, "planning.acceleration_scaling", 0.15);
  eef_step_m_ = declare_or_get<double>(*node_, "cartesian.eef_step_m", 0.005);
  max_solutions_ = declare_or_get<int>(*node_, "mtc.max_solutions", 10);
}

// Task A: reach the pick and descend onto the grasp, jaws open. Ends at the
// grasp pose so our GripperCommander::close() runs next.
std::shared_ptr<mtc::Task> MtcPickPlace::build_reach_task(const std::string& object_id, double half_height_m) {
  auto task = std::make_shared<mtc::Task>();
  task->stages()->setName("reach " + object_id);

  mtc::Stage* current_ptr = add_task_prefix(*task, node_);

  task->add(make_normalize_mimics());

  auto sampling_planner = make_sampling_planner(node_, velocity_scaling_, acceleration_scaling_);
  auto cartesian_planner = make_cartesian_planner(velocity_scaling_, acceleration_scaling_, eef_step_m_);

  const Eigen::Isometry3d grasp_tf = grasp_frame_transform(half_height_m, grasp_offset_m_);

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to pick", mtc::stages::Connect::GroupPlannerVector{{kArmGroup, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task->add(std::move(stage));
  }

  {
    auto pick = std::make_unique<mtc::SerialContainer>("pick");
    task->properties().exposeTo(pick->properties(), {"eef", "group", "ik_frame"});
    pick->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("approach", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(kMinTravelM, approach_height_m_);
      stage->setIKFrame(kTcpFrame);
      stage->setDirection(axis(kTcpFrame, 0.0, 0.0, 1.0));
      pick->insert(std::move(stage));
    }

    {
      auto generator = std::make_unique<mtc::stages::GenerateGraspPose>("generate grasp pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT);
      generator->setPreGraspPose(kOpenState);
      generator->setObject(object_id);
      generator->setAngleDelta(grasp_angle_delta_rad_);
      // the native open stage is gone; monitor the current state instead
      generator->setMonitoredStage(current_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("grasp ik", std::move(generator));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      wrapper->setIKFrame(grasp_tf, kTcpFrame);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      pick->insert(std::move(wrapper));
    }

    task->add(std::move(pick));
  }

  return task;
}

// Task B: the object is physically grasped. Attach it, lift, move to the place,
// lower, release (native), detach, and retreat.
std::shared_ptr<mtc::Task> MtcPickPlace::build_place_task(const std::string& object_id,
                                                          const geometry_msgs::msg::PoseStamped& destination) {
  auto task = std::make_shared<mtc::Task>();
  task->stages()->setName("place " + object_id);

  add_task_prefix(*task, node_);

  auto sampling_planner = make_sampling_planner(node_, velocity_scaling_, acceleration_scaling_);
  auto cartesian_planner = make_cartesian_planner(velocity_scaling_, acceleration_scaling_, eef_step_m_);
  auto interpolation_planner = std::make_shared<mtc::solvers::JointInterpolationPlanner>();

  {
    // the grasp closed onto the cube and the cube rests on its support, so allow
    // both contacts before attach/lift
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("allow grasp collisions");
    stage->allowCollisions(object_id, gripper_collision_links(task->getRobotModel()), true);
    stage->allowCollisions(object_id, support_surfaces_, true);
    task->add(std::move(stage));
  }

  task->add(make_normalize_mimics());

  // kept so GeneratePlacePose monitors the grasp that was attached
  mtc::Stage* attach_stage_ptr = nullptr;
  {
    auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("attach object");
    stage->attachObject(object_id, kTcpFrame);
    attach_stage_ptr = stage.get();
    task->add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::MoveRelative>("lift", cartesian_planner);
    stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
    stage->setMinMaxDistance(kMinTravelM, retreat_height_m_);
    stage->setIKFrame(kTcpFrame);
    stage->setDirection(axis("world", 0.0, 0.0, 1.0));
    task->add(std::move(stage));
  }

  {
    auto stage = std::make_unique<mtc::stages::Connect>(
        "move to place", mtc::stages::Connect::GroupPlannerVector{{kArmGroup, sampling_planner}});
    stage->setTimeout(5.0);
    stage->properties().configureInitFrom(mtc::Stage::PARENT);
    task->add(std::move(stage));
  }

  {
    auto place = std::make_unique<mtc::SerialContainer>("place");
    task->properties().exposeTo(place->properties(), {"eef", "group", "ik_frame"});
    place->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group", "ik_frame"});

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("lower", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(kMinTravelM, retreat_height_m_);
      stage->setIKFrame(kTcpFrame);
      stage->setDirection(axis("world", 0.0, 0.0, -1.0));
      place->insert(std::move(stage));
    }

    {
      auto generator = std::make_unique<mtc::stages::GeneratePlacePose>("generate place pose");
      generator->properties().configureInitFrom(mtc::Stage::PARENT, {"ik_frame"});
      generator->setObject(object_id);
      generator->setPose(destination);
      generator->setMonitoredStage(attach_stage_ptr);

      auto wrapper = std::make_unique<mtc::stages::ComputeIK>("place ik", std::move(generator));
      wrapper->setMaxIKSolutions(8);
      wrapper->setMinSolutionDistance(1.0);
      // move the attached object's own frame (its centre) to the destination
      wrapper->setIKFrame(object_id);
      wrapper->properties().configureInitFrom(mtc::Stage::PARENT, {"eef", "group"});
      wrapper->properties().configureInitFrom(mtc::Stage::INTERFACE, {"target_pose"});
      place->insert(std::move(wrapper));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveTo>("open gripper", interpolation_planner);
      stage->setGroup(kGripperGroup);
      stage->setGoal(kOpenState);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::ModifyPlanningScene>("detach object");
      stage->detachObject(object_id, kTcpFrame);
      place->insert(std::move(stage));
    }

    {
      auto stage = std::make_unique<mtc::stages::MoveRelative>("retreat", cartesian_planner);
      stage->properties().configureInitFrom(mtc::Stage::PARENT, {"group"});
      stage->setMinMaxDistance(kMinTravelM, retreat_height_m_);
      stage->setIKFrame(kTcpFrame);
      stage->setDirection(axis("world", 0.0, 0.0, 1.0));
      place->insert(std::move(stage));
    }

    task->add(std::move(place));
  }

  return task;
}

SkillResult MtcPickPlace::run_task(const std::shared_ptr<mtc::Task>& task, const std::string& phase) {
  try {
    task->init();
  } catch (const mtc::InitStageException& error) {
    return skill_error(phase, "MTC task init failed: " + std::string(error.what()));
  }

  {
    std::lock_guard<std::mutex> lock(task_mutex_);
    task_ = task;
  }

  const auto plan_result = task->plan(static_cast<std::size_t>(max_solutions_));
  if (!plan_result) {
    std::ostringstream reason;
    task->explainFailure(reason);
    return skill_error(phase, "MTC found no solution: " + reason.str(), plan_result.val);
  }

  const auto exec_result = task->execute(*task->solutions().front());
  if (!exec_result) {
    return skill_error(phase, "MTC solution execution failed", exec_result.val);
  }
  return skill_ok(phase);
}

SkillResult MtcPickPlace::sort(const std::string& object_id, double half_height_m,
                               const geometry_msgs::msg::PoseStamped& destination,
                               const std::function<void(const std::string&)>& report_phase) {
  report_phase("open");
  const auto open_result = gripper_.open();
  if (!open_result.ok) {
    return open_result;
  }

  report_phase("reach");
  const auto reach_result = run_task(build_reach_task(object_id, half_height_m), "reach");
  if (!reach_result.ok) {
    return reach_result;
  }

  // custom stall-close; a missed grasp aborts here, before the arm lifts
  report_phase("grasp");
  const auto close_result = gripper_.close();
  if (!close_result.ok) {
    return close_result;
  }

  report_phase("place");
  const auto place_result = run_task(build_place_task(object_id, destination), "place");
  if (!place_result.ok) {
    return place_result;
  }

  return skill_ok("sort");
}

void MtcPickPlace::cancel() {
  std::lock_guard<std::mutex> lock(task_mutex_);
  if (task_) {
    task_->preempt();
  }
}

}  // namespace sorting_arm
