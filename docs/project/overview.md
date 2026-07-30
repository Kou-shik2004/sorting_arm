# Project overview

## Goal

`sorting_arm` will be a simulated sorting cell built around a UR5e, a Robotiq
2F-85, Gazebo Harmonic, `ros2_control`, MoveIt 2, and ROS 2 Jazzy.

The completed pre-perception milestone will:

1. read a fixed YAML snapshot of objects;
2. mirror the table, trays, and objects into MoveIt’s planning scene;
3. select a destination slot from each object’s label;
4. pick and place each object through reusable manipulation skills; and
5. return the arm to a measured SRDF `home` state.

Real perception will later replace only the fixed object provider. It must not force
the manipulation layer or executive to be redesigned. The planned real provider uses
the wrist camera and returns detections through the same contract.

The deployed application will start one autonomous sorting cycle after its runtime
dependencies report ready. Its BehaviorTree will call the reusable Pick, Place, and
Home actions and apply bounded recovery policy from their typed results. A successful
fixed scene may never enter a recovery branch, so controlled failures must prove
those branches independently.

## Final package map

| Package | Why it exists | Status now |
|---|---|---|
| `sorting_arm_description` | Owns robot geometry, joints, frames, meshes, and `ros2_control` hardware descriptions. | Existing |
| `sorting_arm_moveit` | Owns SRDF, planning groups, kinematics, planners, controllers, and MoveIt launch configuration. | Existing |
| `sorting_arm_bringup` | Starts Gazebo, controllers, MoveIt, RViz, and later application nodes. | Existing |
| `sorting_arm_skills` | Owns reusable motion, planning-scene, gripper, Pick, Place, and Home behavior. | Experimental MoveIt scaffolding |
| `sorting_arm_interfaces` | Defines typed messages, services, and actions shared across packages. | Future, after the skills gate |
| `sorting_arm_executive` | Starts one cycle and decides detection, synchronization, sorting order, slot allocation, recovery, and skill order. | Future |
| `sorting_arm_perception` | Replaces the fixed object source while keeping the same detection contract. | Future |

The skills package defines how one manipulation operation is performed safely. The
executive selects the next operation, object, and destination. Keeping those
responsibilities separate prevents task policy from leaking into motion code.

## Current scope

`sorting_arm_skills` currently builds `commander`, an early `MoveGroupInterface`
scaffold that is not yet an accepted manipulation capability.

The package does not yet provide stable reusable motion, planning-scene management,
gripper commands, deterministic skill sequences, action servers, or unit tests.
Interfaces, the executive, perception, and application bringup integration remain
planned work governed by the [implementation roadmap](../plan/roadmap.md). A
dedicated CI image and per-change workflow provide the local clean-build foundation;
remote workflow evidence and independent development-image validation are pending.
CI matures with the application rather than waiting for all features.

## Project navigation

- [Architecture](architecture.md) owns package boundaries and runtime contracts.
- [Architectural decisions](decisions.md) records accepted choices and reasons.
- [Development workflow](development-workflow.md) defines feature slices, review,
  understanding, and verification.
- [Context management](context-management.md) defines which information remains
  canonical, temporary, or on demand.
- [Implementation roadmap](../plan/roadmap.md) owns ordered gates and live status.
- [Continuous integration](../plan/continuous-integration.md) defines the three
  validation levels.
- [Deployment experience](../plan/deployment-experience.md) defines the final
  one-command demonstration and runtime-image contract.

## Success criteria

The pre-perception path is complete only when evidence shows:

- the three controllers are active and the gripper result semantics are measured;
- the planning frame is `world` and the commanded link is `tcp`;
- named, six-joint, pose, and retimed Cartesian motion work independently;
- the planning scene contains the intended static and dynamic objects;
- collision checking rejects a deliberately unsafe goal;
- CLI Pick attaches exactly the requested object;
- CLI Place returns that object to the world with no remaining attachment;
- four objects reach distinct label-matched slots;
- the arm returns to the measured `home`; and
- local and remote clean builds/tests pass at the appropriate gates.

The bounded recovery path is complete only when evidence also shows:

- a temporary detection failure is retried within a fixed budget;
- a missed grasp causes retreat, redetection, scene synchronization, and one bounded
  Pick retry;
- a failed primary placement while holding an object tries an alternate matching
  slot and then a configured safe-drop pose;
- non-recoverable controller and scene-invariant failures stop without a blind retry;
- halting the executive or shutting down reaches the active child action and reports
  the resulting object state truthfully; and
- exhausted recovery returns a typed partial-failure result rather than claiming a
  successful sort.

No document may claim a gate passed without recorded command output or an observable
runtime result.

## Explicit non-goals

The project does not currently require MoveIt Task Constructor, MoveItCpp, custom
planner plugins, OMPL internals, custom IK, Jacobian derivations, legacy MoveIt
`pick()`/`place()`, advanced constraint planning, or release automation. Pilz remains
deferred unless measured Cartesian behavior creates a reason to add it.
