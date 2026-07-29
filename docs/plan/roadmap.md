# Implementation roadmap

This is the active implementation sequence for the sorting-arm application. Each
step has an explicit verification gate; later steps remain planned until the
preceding gate passes.

## How to use this roadmap

Complete one step at a time. Review the technical focus and intended files, implement
the checklist, retain reproducible verification evidence, and advance only after the
step's gate passes.

“Future” means no implementation should be added merely because its design appears
below.

## Step 0 — runtime prerequisites

**Status:** In progress. The read-only MoveIt readiness check exists; its runtime
evidence and the remaining controller, home-state, and gripper measurements are
pending.

**Purpose:** Establish trustworthy controller, frame, home-state, and gripper behavior
before judging motion code.

**Files:** `src/sorting_arm_skills/src/moveit_ready_check.cpp` and the existing
`sorting_arm_moveit/config/sorting_arm.srdf` and
`sorting_arm_bringup/config/ros2_controllers.yaml`. Configuration changes require
measured evidence.

**Learn:** Controller manager states, named SRDF states, joint-state readings,
`GripperCommand` result fields, planning frame, and end-effector link.

**Work checklist:**

- Confirm joint state, arm trajectory, and gripper controllers are active.
- Confirm MoveIt reports planning frame `world`.
- Confirm the commanded end-effector link is `tcp`.
- Jog to a collision-safe overhead pose and record all six measured arm joints.
- Adopt the measured joint state as SRDF `home` only after it is reviewed as a safe
  and repeatable state.
- Command an empty close and record position, effort, `stalled`, and `reached_goal`.
- Close on one cube and record the same result fields.
- Change configuration only in response to the evidence and an explicit request.

**Failure symptoms:** Inactive controllers, stale joint state, wrong planning frame,
unknown named target, gripper goal rejection, or identical empty/contact result
semantics.

**Verification:** Save the relevant controller list, MoveIt frame/link output, six
home joint values, and both gripper results.

**Stop gate:** The runtime facts above are measured and understood. Do not infer them
from configuration alone.

## Step 1 — internal result types

**Status:** Future; begin after the runtime-prerequisite gate passes.

**Purpose:** Prevent a single boolean from erasing whether failure came from input
validation, planning, execution, Cartesian coverage, timing, gripper behavior, or the
planning scene.

**Files:** `include/sorting_arm_skills/types.hpp`, `src/types.cpp`.

**Learn:** `enum class`, small value types, `std::string`, const correctness, and the
difference between an operation category and a lower-level MoveIt/controller code.
No MoveIt planning study is required yet.

**Work checklist:**

- Name the internal success/failure categories before assigning values.
- Define one result value that always carries a category and may carry a native code,
  failed phase, and human-readable detail.
- Make success and failure construction unambiguous.
- Provide category/phase-to-string conversion for logs.
- Keep ROS action messages out of the internal type.
- Compile with warnings enabled.

**Failure symptoms:** Callers must parse text, success can carry a failure category,
native error information is discarded, or internal types depend on future interfaces.

**Verification:** Build `sorting_arm_skills`; inspect a tiny compile-time or later unit
example for success and representative failures; run formatting/lint checks when
available.

**Stop gate:** Representative callers demonstrate that downstream code does not need
a boolean or string parser.

## Step 2 — configuration model

**Status:** Future.

**Purpose:** Keep frames, link/group names, planner settings, scaling, Cartesian
limits, scene geometry, and targets out of motion algorithms.

**Files:** `include/sorting_arm_skills/config.hpp`, `src/config.cpp`; populate
`config/skills.yaml` only after the model and validation rules are stable.

**Learn:** ROS 2 parameter declaration/retrieval, parameter arrays, validation,
fail-fast construction, and separating immutable configuration data from node logic.

**Work checklist:**

- Group values by frames, planning, Cartesian behavior, scene, gripper, and targets.
- Declare every parameter with one canonical name.
- Validate non-empty names, scaling ranges, positive lengths/times, quaternion norms,
  exact six-joint targets, unique IDs, and consistent geometry dimensions.
- Return typed validation failures with the offending parameter name.
- Add YAML values only after each has provenance.
- Do not hide fallback values that make a malformed configuration look valid.

**Failure symptoms:** Magic numbers remain in C++, malformed arrays are accepted,
configuration errors appear only during motion, or parameter names drift between C++
and YAML.

**Verification:** Valid configuration loads; focused invalid cases fail before any
MoveIt object or controller client is used.

**Stop gate:** Every later algorithmic value has one validated configuration owner.

## Step 3 — pure pose helpers

**Status:** Future.

**Purpose:** Prove grasp, approach, retreat, placement, and collision geometry without
moving a robot.

**Files:** `include/sorting_arm_skills/pose_helpers.hpp`,
`src/pose_helpers.cpp`, `test/test_pose_helpers.cpp`.

**Learn:** `PoseStamped`, frames, quaternions, composing translational offsets,
object-center versus TCP poses, `shape_msgs/SolidPrimitive`, and MoveIt collision
object messages.

**Work checklist:**

- Validate the expected frame and finite pose values.
- Build a normalized top-down tool orientation.
- Derive grasp TCP height from object center, object half-height, and configured
  tool/grasp offset.
- Derive pre-grasp, place, pre-place, and retreat by explicit world-Z offsets.
- Construct primitive collision objects with matching dimensions and center poses.
- Keep helpers deterministic and free of nodes, clocks, MoveGroup, and scene clients.
- Test positions, orientations, frames, IDs, dimensions, and invalid inputs.

**Failure symptoms:** Z offsets use the wrong sign, object-center poses are commanded
as TCP poses, quaternions are invalid, or tests need a ROS graph.

**Verification:** `test_pose_helpers` passes entirely without MoveIt or Gazebo
runtime nodes.

**Stop gate:** All pose/geometry math needed by later motion and scene code is covered
by readable unit tests.

## Step 4A — named motion

**Status:** Future.

**Purpose:** Provide the operation needed to reach the measured SRDF `home`.

**Files:** `include/sorting_arm_skills/motion_commander.hpp`,
`src/motion_commander.cpp`.

**Learn:** What `move_group` owns, how `MoveGroupInterface` communicates with it,
planning groups, named states, `setNamedTarget`, `setStartStateToCurrentState`,
`plan`, `execute`, and MoveIt error codes.

**Work checklist:**

- Configure planning pipeline, planner ID, planning time, and scaling once.
- Reject an empty or unknown named target.
- Check the target setter’s result.
- Set the current start state immediately before planning.
- Plan once and preserve its returned trajectory.
- Execute that exact plan.
- Distinguish planning from execution failure.
- Clear state that could leak into the next operation.

**Failure symptoms:** An unknown target reaches the planner, `move()` replans,
planning and execution failures look identical, or stale pose targets affect a named
request.

**Verification:** The one-shot demo later reaches a safe named state; an unknown name
returns a typed validation/target failure without execution.

**Stop gate:** Named motion has independent success and failure evidence.

## Step 4B — six-joint motion

**Status:** Future.

**Purpose:** Provide a direct joint-space API for controlled test poses and diagnosis.

**Files:** The same `motion_commander.hpp` and `motion_commander.cpp`.

**Learn:** Joint model groups, active variable count/order, current robot state,
`setJointValueTarget`, joint bounds, and why vector position is a contract.

**Work checklist:**

- Require exactly six arm values.
- Confirm the configured group has the expected six active variables.
- Reject non-finite or out-of-bounds targets before planning.
- Use the same plan-once/execute-that-plan lifecycle as named motion.
- Report which validation or MoveIt stage failed.

**Failure symptoms:** Too few values silently preserve old joints, extra values are
ignored, wrong joint ordering moves unexpectedly, or invalid bounds reach execution.

**Verification:** One safe six-joint target executes; five-value, seven-value, NaN,
and out-of-bounds inputs fail without moving.

**Stop gate:** Exactly-six input validation and one successful execution are proven.

## Step 4C — TCP pose motion

**Status:** Future.

**Purpose:** Support pre-grasp, transfer, and pre-place movements expressed as TCP
poses.

**Files:** The same `motion_commander.hpp` and `motion_commander.cpp`.

**Learn:** Planning frame versus commanded link, `setPoseTarget`,
`clearPoseTargets`, IK as part of planning, and target quaternion/frame validation.

**Work checklist:**

- Require planning frame `world` and command link `tcp`.
- Validate finite position and normalized orientation.
- Set the TCP pose target and check setter success.
- Set current start state immediately before planning.
- Plan once and execute the returned plan.
- Clear pose targets on every exit path.
- Preserve distinct target, planning, and execution errors.

**Failure symptoms:** A pose in another frame is silently accepted, the wrist link is
commanded instead of `tcp`, pose targets leak into later operations, or unreachable
poses look like execution failures.

**Verification:** One reachable overhead TCP pose succeeds; malformed frame,
quaternion, and unreachable-pose cases return the expected category.

**Stop gate:** Pose motion and its error cases are independently demonstrated.

## Step 4D — Cartesian motion

**Status:** Future.

**Purpose:** Keep short descend and retreat segments vertical, collision checked,
fully covered, timed, and duration bounded.

**Files:** The same MotionCommander files and
`test/test_motion_decisions.cpp`.

**Learn:** Cartesian waypoints, `computeCartesianPath`, coverage fractions, a path
versus a timed trajectory, `RobotTrajectory`, TOTG, duration inspection, and
collision checking.

**Work checklist:**

- Use the installed Jazzy overload without deprecated jump threshold.
- Compute with collision avoidance enabled and capture `MoveItErrorCodes`.
- Reject error returns and coverage below the configured minimum.
- Convert the message into `robot_trajectory::RobotTrajectory`.
- Run TOTG with configured velocity and acceleration scaling.
- Copy the timed result back to the executable message.
- Reject missing/invalid timing or duration above the segment ceiling.
- Execute only after every decision passes.
- Unit-test fraction, retiming, and duration decision logic independently.

**Failure symptoms:** A fraction such as `0.6` is treated as success, a geometric path
is sent without timestamps, scaling is ignored, duration is unchecked, or collisions
are disabled.

**Verification:** Decision tests cover boundary values; a short vertical runtime
segment reports sufficient coverage, valid timing, acceptable duration, and executes.

**Stop gate:** Partial, untimed, over-duration, and failed paths are all rejected, and
one complete retimed path succeeds.

## Step 5 — planning scene

**Status:** Future.

**Purpose:** Give MoveIt the table, trays, and objects that Gazebo does not
automatically add to its collision world.

**Files:** `include/sorting_arm_skills/scene_manager.hpp`,
`src/scene_manager.cpp`.

**Learn:** World collision objects versus attached objects,
`PlanningSceneInterface::applyCollisionObject(s)`, `getObjects`,
`applyAttachedCollisionObject`, `getAttachedObjects`, attachment link, touch links,
and add/remove operations.

**Work checklist:**

- Build static table and compound tray geometry through the pure helpers.
- Apply static and dynamic objects synchronously.
- Verify requested IDs and geometry by querying the scene.
- Keep the configured dynamic-ID set separate from unrelated scene objects.
- Attach the existing object to `tcp` using configured touch links.
- Verify attached presence and world absence.
- Detach and reinsert at the requested center pose.
- Verify world presence and attached absence.
- Return typed failures; do not use sleeps as synchronization.

**Failure symptoms:** RViz shows no collision geometry, objects remain both world and
attached, stale objects accumulate, unrelated objects are removed, or timing sleeps
hide failed updates.

**Verification:** RViz and scene queries agree on table, trays, and test objects;
attach/detach transitions satisfy the exact presence/absence invariants.

**Stop gate:** Every scene mutation is synchronously verified.

## Step 6 — one-shot demonstration

**Status:** Future.

**Purpose:** Expose the reusable APIs for manual proof before actions hide them behind
another layer.

**Files:** `src/motion_demo_main.cpp`, then `config/skills.yaml`.

**Learn:** Node creation, parameter-file loading, choosing one requested operation,
logging typed results, and process exit status.

**Work checklist:**

- Register one small executable only at this step.
- Select one named, joint, pose, Cartesian, or scene operation by parameters.
- Require explicit targets; do not embed a magic demonstration pose.
- Print selected frames/groups and the complete typed result.
- Exit non-zero on failure.
- Keep it one-shot; do not add action behavior or sequence policy.

**Failure symptoms:** The demo runs multiple operations implicitly, hard-codes scene
values, swallows failures, or duplicates MotionCommander logic.

**Verification:** Each reusable API can be invoked independently with a documented
command and produces inspectable output.

**Stop gate:** The demo is a thin parameter-driven adapter, not a second
implementation.

## Step 7 — reusable-skills gate

**Status:** Future.

**Purpose:** Prevent gripper/actions/sequences from being built on an unproven motion
and planning-scene layer.

**Files:** Relevant unit tests, configuration, and recorded commands only; no new
feature files.

**Learn:** How to interpret MoveIt logs, controller results, RViz collision display,
and `colcon test-result --verbose`.

**Work checklist:**

- Run a clean package build and all registered unit tests.
- Execute named, six-joint, TCP pose, and Cartesian demonstrations.
- Confirm Cartesian timing and duration evidence.
- Add and query the complete planning scene.
- Demonstrate one deliberate collision rejection.
- Demonstrate representative invalid-input and planning failures.
- Retain the commands and results, including failures, as acceptance evidence.

**Failure symptoms:** Any API is only transitively tested, RViz disagrees with scene
queries, an unsafe target executes, or only success paths have evidence.

**Verification:** The complete named/joint/pose/Cartesian/scene/collision/error
evidence set is recorded.

**Stop gate:** The reusable layer satisfies every verification item above and is
accepted for use by higher-level packages.

## Step 8 — continuous integration

**Status:** Future; start only after Step 7 passes locally.

**Purpose:** Prove package metadata, dependencies, build, and tests on a clean machine
rather than through stale local artifacts.

**Files:** `.github/workflows/ci.yml`.

**Learn:** Workflow, event, job, runner, step, action, shell exit status,
`push`/`pull_request`, Docker image builds, `colcon build`, `colcon test`, verbose
test results, workflow logs, and required checks.

**Work checklist:**

- Write one understandable workflow using the project development image.
- Build the workspace from a clean checkout.
- Run tests and always expose verbose results on failure.
- Explain every workflow line.
- Open the pull request and require the remote workflow to pass.
- Configure a required check only when the repository owner chooses to.

**Failure symptoms:** CI succeeds through cached local outputs, a test failure is
masked, logs omit `colcon test-result`, or the workflow contains unexplained release,
secret, matrix, or cache machinery.

**Verification:** One remote `push` or pull-request run builds and tests successfully;
a deliberately failing branch proves the check turns red and points to the failing
step.

**Stop gate:** The remote result is green and every workflow step is documented and
reviewable.

## Step 9 — shared interfaces

**Status:** Future; do not create the package before Step 7.

**Purpose:** Give the skill server and executive stable typed contracts without making
either depend on the other’s implementation.

**Files:** A future `sorting_arm_interfaces` package containing
`DetectedObject.msg`, result/phase definitions, `Pick.action`, `Place.action`,
`Home.action`, `DetectObjects.srv`, and `SyncObjects.srv`.

**Learn:** ROS 2 message/service/action syntax, bounded goals versus long-running
actions, feedback, cancellation, package interface generation, and dependency
direction.

**Work checklist:**

- Translate proven internal concepts into minimal external types.
- Define object ID, opaque label, stamped center pose, and primitive geometry.
- Preserve typed result category, native code, failed phase, object state, and detail.
- Keep Pick/Place/Home long-running and cancellable.
- Keep DetectObjects/SyncObjects request-response bounded.
- Avoid embedding BehaviorTree policy or MoveIt C++ types in messages.
- Generate and inspect the interfaces before consumers are added.

**Failure symptoms:** Actions expose internal classes, results collapse back to a
boolean, object-center semantics are ambiguous, or interfaces depend on skills.

**Verification:** The interfaces package builds alone and `ros2 interface show`
matches the documented contract.

**Stop gate:** Both future consumers can depend on interfaces while interfaces depends
on neither.

## Step 10 — gripper and deterministic sequences

**Status:** Future.

**Purpose:** Add measured gripper semantics and testable Pick, Place, and Home policy
without involving action-server concurrency.

**Files:** `gripper_commander.hpp/.cpp`, `adapters.hpp`,
`skill_sequences.hpp/.cpp`, `test/test_skill_sequences.cpp`.

**Learn:** ROS 2 action-client lifecycle, `GripperCommand`, injected interfaces/fakes,
deterministic state machines, segment-boundary cancellation, and truthful scene
transitions.

**Work checklist:**

- Implement open/close commands with timeout and native result preservation.
- Interpret grasp success from Step 0’s measured `stalled`/`reached_goal` behavior.
- Define narrow motion, scene, and gripper adapter interfaces.
- Implement Pick: validate → open → pre-grasp → descend → close → attach → retreat.
- Implement Place: validate attached → pre-place → descend → open → detach/reinsert
  → retreat.
- Implement Home through named motion.
- Check cancellation between physical segments.
- Test exact phase order and each injected failure with fakes.
- Register only the gripper/sequence sources and tests reached by this step.

**Failure symptoms:** Timeout is treated as contact, attachment occurs before verified
grasp, detachment occurs before opening, fakes cannot force each failure, or sequence
policy lives in ROS action callbacks.

**Verification:** Deterministic tests prove order, early exit, result propagation,
scene invariants, and cancellation boundaries.

**Stop gate:** Pick, Place, and Home sequences are fully testable without live MoveIt
or controllers, and gripper interpretation matches runtime evidence.

## Step 11 — action server

**Status:** Future.

**Purpose:** Expose long-running, cancellable Pick, Place, and Home operations with
feedback and one clear manipulation owner.

**Files:** `skill_server_node.hpp/.cpp`, `skill_server_main.cpp`.

**Learn:** `rclcpp_action` goal/cancel/accepted callbacks, feedback/result lifecycle,
callback responsiveness, owned worker lifetime, and mutual exclusion.

**Work checklist:**

- Create one node owning motion, scene, gripper, and sequence objects.
- Create all three action servers from the shared interfaces.
- Reject concurrent manipulation goals.
- Run blocking sequence work in one owned joinable worker.
- Publish phase feedback at segment boundaries.
- Map internal results to external results without losing native details.
- Keep cancel callbacks responsive and report cancellation truthfully.
- Shut down without detached work or use-after-free.

**Failure symptoms:** Detached threads outlive the node, callbacks block the executor,
two goals move the same arm, cancellation claims more than the controller did, or
action callbacks duplicate sequence logic.

**Verification:** CLI goals show feedback and typed results; concurrent-goal rejection,
cancellation, shutdown, Pick attachment, Place detachment, and Home all behave as
documented.

**Stop gate:** One exclusive server owns manipulation and each action works
independently before an executive exists.

## Step 12 — executive

**Status:** Future.

**Purpose:** Add task order and label-to-slot routing outside the manipulation layer.

**Files:** A future `sorting_arm_executive` package with fixed-source configuration
and parser, `DetectObjects` provider, BehaviorTree nodes/tree, executive node/main,
and pure allocation/order tests.

**Learn:** BehaviorTree Sequence semantics, blackboard data, asynchronous ROS
service/action leaves, stable object IDs, deterministic slot allocation, and
fail-fast orchestration.

**Work checklist:**

- Load fixed YAML objects behind `DetectObjects`.
- Call `SyncObjects` with the snapshot.
- Allocate ordered, distinct slots by opaque label.
- For each object, call Pick then Place.
- Call Home after all objects succeed.
- Stop the first milestone on the first typed failure.
- Reject unknown labels, exhausted slots, and a second static cycle until reset.
- Keep motion/scene/gripper details out of tree nodes.
- Test tree order and slot decisions with fakes.
- Add bringup integration only after independent executive tests pass.

**Failure symptoms:** Labels change motion mechanics, slots are reused, blocking leaves
freeze the executor, failures are ignored, or the executive manipulates MoveIt
directly.

**Verification:** Fake tests prove
Detect → Sync → four Pick/Place pairs → Home and all failure stops; runtime evidence
shows four objects in distinct matching slots, no attachment, and the arm at home.

**Stop gate:** The fixed-source cycle passes and its second-trigger guard is proven.

## Step 13 — perception

**Status:** Future.

**Purpose:** Replace fixed object input with real detections without changing
manipulation or task contracts.

**Files:** A future `sorting_arm_perception` package and only the bringup/configuration
changes needed to select its `DetectObjects` provider.

**Learn:** The selected sensor/model pipeline, frame transforms into `world`,
timestamp handling, stable IDs, uncertainty, and filtering. Choose these only when
the perception approach is explicitly scoped.

**Work checklist:**

- Produce the existing `DetectedObject` contract.
- Transform center poses into `world`.
- Provide stable IDs and opaque labels.
- Validate primitive geometry and reject stale/invalid detections.
- Replace the provider through launch/configuration, not downstream rewrites.
- Compare fixed-source and perception-driven snapshots.
- Add perception-specific tests and runtime evidence.

**Failure symptoms:** Skills gain camera dependencies, the executive learns
model-specific labels, frames are ambiguous, IDs change mid-cycle, or downstream
interfaces must change.

**Verification:** The same Sync/Pick/Place/Home path runs with the perception provider
and no downstream source changes.

**Stop gate:** Perception is a contract-compatible provider, not a new application
architecture.

## Gate summary

| After step | Evidence required before advancing |
|---|---|
| 0 | Controllers, frames, measured home, and gripper semantics |
| 1–3 | Typed results, validated configuration, and pure geometry tests |
| 4A–4D | Independent named, joint, pose, and safe retimed Cartesian motion |
| 5–7 | Verified scene transitions, collision rejection, and complete reusable-layer evidence |
| 8 | Understandable remote clean build/test |
| 9–11 | Stable interfaces, deterministic sequences, and independent actions |
| 12 | Complete fixed-source sorting cycle |
| 13 | Contract-compatible real perception |
