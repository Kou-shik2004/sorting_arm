# Architecture

This document defines the target architecture and ownership boundaries. The
[implementation roadmap](../plan/roadmap.md) controls when each component is
introduced and records the current project stage.

## Target application data flow

```text
Docker Compose / application bringup
        │ one automatic cycle after readiness
        ▼
BehaviorTree executive ◄──── DetectObjects ──── fixed YAML or wrist perception
        │
        │ SyncObjects / Pick / Place / Home
        ▼
manipulation skill server
        │
        ├── MoveGroupInterface
        ├── PlanningSceneInterface
        └── GripperCommand
        │
        ▼
MoveIt + ros2_control + Gazebo
```

Today only the established description, MoveIt, and bringup packages plus the minimal
skills package are active.

## Ownership boundaries

### Robot configuration

`sorting_arm_description` owns physical and kinematic description: links, joints,
meshes, transforms, and hardware interfaces. `sorting_arm_moveit` owns semantic and
planning configuration: groups, named states, kinematics, joint limits, planning
pipelines, and controllers. Application code reads those contracts; it does not
redefine them.

### Reusable manipulation

`sorting_arm_skills` will own:

- validated, typed results;
- ROS-parameter-backed manipulation configuration;
- pure pose and collision-object construction;
- named, joint, pose, and Cartesian arm motion;
- planning-scene world/attachment transitions;
- gripper commands and grasp-result interpretation;
- deterministic Pick, Place, and Home sequences; and
- one exclusive skill-server node exposing the three manipulation actions.

It will not choose object order or label-to-slot policy.

### Shared contracts

`sorting_arm_interfaces` will be created only after the reusable skills gate. It will
contain stable data types and long-running action contracts without depending on
skill implementation. Pick, Place, and Home cross the executive-to-skills boundary.
This avoids inventing an external API before the internal behavior is proven.

### Task policy

`sorting_arm_executive` will start exactly one cycle after controllers, MoveIt, the
skill server, and the selected object provider report ready. It will request an
object snapshot, synchronize it with MoveIt, allocate destination slots, and call
Pick, Place, and Home in order. It will stop on the first typed failure in the first
milestone. Motion planning, gripper mechanics, and attachment details do not belong
here.

BehaviorTree.CPP is used at this layer because later recovery policy must be visible,
composable, and independently testable through sequences, fallbacks, conditions,
retries, and asynchronous action leaves. It is not used to hide arm-motion or
planning-scene mechanics. The first milestone remains deliberately fail-fast until
the happy path and failure evidence are trustworthy; recovery behavior is added only
with an explicit policy and tests.

### Perception

`sorting_arm_perception` will eventually provide `DetectObjects`. The fixed YAML
provider and real perception must return the same contract so everything downstream
remains unchanged.

The planned real provider uses the wrist camera. It owns image processing, camera
calibration assumptions, timestamp validation, transforms into `world`, and stable
object identity. Reference perception logic may accelerate implementation, but its
frames, intrinsics, object model, and error behavior must be verified against this
cell before adoption.

## Recovery contract

Recovery is explicit task policy, not an automatic property of BehaviorTree.CPP or
ROS actions. The baseline tree first proves fail-fast sorting. The recovery milestone
then adds only these bounded branches:

- retry a temporary `DetectObjects` failure within a configured budget;
- after a missed grasp with the object still in the world, retreat, obtain a fresh
  snapshot, synchronize the scene, and retry Pick once;
- when Place fails and the object remains attached, try another unused slot for the
  same opaque label, then a configured safe-drop pose;
- after recovery leaves no attached object, make one best-effort Home request before
  returning the original failure; and
- stop immediately on invalid input, controller unavailability, violated scene
  invariants, or any failure whose reported object state is unknown.

Recovery decisions use the typed category, failed phase, and final object state from
the child action. No retry may be selected from message text alone. Exhausting a
retry, using the safe-drop pose, or leaving any object unsorted returns a partial
failure; recovery must never relabel an incomplete job as success.

The tree organizes these branches through Sequence, Fallback, condition, and bounded
retry nodes. It does not erase physical state: world, attached, placed, safe-dropped,
and unknown remain explicit states that determine which branch is legal.

Every recovery branch requires deterministic fake-based evidence and one controlled
runtime fault where practical. A tuned scene that never fails naturally is not
evidence that recovery works.

## Frame and pose contract

- World and object-center poses use planning frame `world`.
- Arm pose goals command link `tcp`.
- Object poses represent geometric centers, not TCP poses.
- Pure helpers derive grasp, pre-grasp, placement, and retreat poses from object
  geometry and configured clearances.
- Every received `PoseStamped` frame is validated rather than silently reinterpreted.

## Motion contract

Named, exactly-six-joint, and TCP-pose operations follow one explicit lifecycle:

```text
validate input
  → configure and set target
  → setStartStateToCurrentState()
  → plan()
  → inspect planning result
  → execute(the returned plan)
  → inspect execution result
```

The implementation will never call `move()` after already calling `plan()`, because
that would plan a second trajectory and discard the one that was checked.

Short descents and retreats use collision-checked Cartesian interpolation. A returned
geometric path is not yet an executable trajectory: the coverage fraction must meet
the configured threshold, TOTG must add valid timing, the final duration must remain
within its limit, and only then may it execute.

## Planning-scene contract

Gazebo’s physical objects do not automatically exist in MoveIt’s collision world.
`SceneManager` will synchronously:

- apply table, compound tray, and dynamic object collision geometry;
- query world objects to verify each change;
- attach the existing requested object to `tcp` with configured touch links;
- verify attached presence and world absence;
- detach and reinsert an object at its placed center pose; and
- verify world presence and attached absence.

No settling sleep substitutes for a state query.

## Gripper contract

The Robotiq controller will be commanded through `control_msgs/action/GripperCommand`.
Runtime measurements must establish what an empty close and a cube-contact close
report. The intended grasp rule is based on the action result:

- `reached_goal=true` at the fully closed position means no object was captured;
- cube contact should stop the fingers early and report `stalled=true`;
- timeout alone is not evidence of contact.

Those assumptions stay provisional until Step 0 records evidence from this workspace.

## Concurrency contract

The future skill node will own one arm `MoveGroupInterface`, one
`PlanningSceneInterface`, one gripper action client, and the three skill action
servers. It will accept only one manipulation goal at a time. One owned worker will
perform blocking physical segments while the executor remains able to process goal,
feedback, and cancellation callbacks.

Cancellation is observed between truthful physical segments. The design will not
claim that an already-running controller command was preempted until its result
actually returns.

Only one BehaviorTree cycle runs per simulator reset. Its asynchronous leaves are the
clients of Pick, Place, and Home. Halting the tree or shutting down the executive
requests cancellation of the active child goal and waits for its truthful terminal
result before choosing cleanup.

## Error propagation

Each layer adds context without destroying lower-level evidence:

```text
MoveIt/controller/scene result
        ↓
internal typed skill result
        ↓
Pick / Place / Home action result
        ↓
BehaviorTree recovery decision
        ↓
executive completion report and final scene state
```

At minimum, failures distinguish invalid input, planning failure, execution failure,
incomplete Cartesian coverage, retiming failure, duration violation, gripper failure,
missed grasp, scene update failure, cancellation, and internal failure.
