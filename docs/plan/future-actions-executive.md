# Future interfaces, actions, and executive

## Status

This document records the target design for `sorting_arm_interfaces`,
`sorting_arm_executive`, the manipulation action server, and bringup integration.
Implementation begins at the corresponding gates in the
[roadmap](roadmap.md).

## Why interfaces wait

The internal skills layer must first reveal which result categories, phases, pose
semantics, and scene invariants are actually needed. Creating public ROS interfaces
before that proof would freeze guesses into contracts used by multiple packages.

Once Step 7 passes, `sorting_arm_interfaces` becomes a small dependency shared by the
skill server and executive:

```text
sorting_arm_interfaces
      ▲             ▲
      │             │
sorting_arm_skills  sorting_arm_executive
```

The interfaces package must never depend on either consumer.

## Planned object contract

`DetectedObject` will carry:

- stable object ID;
- opaque classification label;
- center `PoseStamped` in `world`; and
- primitive type and dimensions.

“Center pose” must appear in field comments/documentation. A grasp TCP pose is derived
inside skills and is never supplied by perception or the executive.

## Planned result contract

External skill results will preserve:

- success/failure category;
- lower-level MoveIt/controller code when one exists;
- failed physical phase;
- expected object state, such as world or attached;
- concise detail.

The exact message layout is chosen in Step 9 from the proven internal type. Message
definitions remain independent of C++ implementation types.

## Planned service contracts

### `DetectObjects`

Returns a bounded snapshot of detected/configured objects. The initial provider reads
fixed YAML. Future perception implements the same service.

### `SyncObjects`

Accepts the current object snapshot and reconciles only dynamic object IDs owned by
the application. Static table/tray setup remains a skill-server scene concern. The
response returns a typed scene result rather than a count alone.

Services are appropriate here because each operation is bounded and has one response.

## Planned action contracts

Actions are the ROS boundary between independently owned responsibilities. The
executive's asynchronous BehaviorTree leaves send Pick, Place, and Home goals to the
skill server. The action layer does not create recovery by itself. It supplies the
goal lifecycle, typed terminal state, and cancellation path that the tree's policy
needs.

### `Pick`

Goal: requested object identity and any stable data needed to validate the current
scene object. Feedback: current physical phase. Result: typed outcome and final object
state.

Success means the object is attached to `tcp`, absent from the world scene, and the
arm has retreated. Closing the gripper by itself is not success.

### `Place`

Goal: attached object ID and destination object-center pose. Feedback: current
physical phase. Result: typed outcome and final object state.

Success means the gripper opened, the object is detached, the world scene contains it
at the destination, no attachment remains, and the arm has retreated.

### `Home`

Goal: normally empty or a named-state override only if later evidence justifies it.
Feedback: homing phase. Result: typed motion outcome.

Success means the measured SRDF `home` plan executed, not merely that planning
succeeded.

The sorting application has no top-level action. Bringup starts one cycle after
explicit readiness, and the executive remains idle after success or failure so the
final Gazebo, RViz, tree, and log state can be inspected. Another repository may wrap
the application later, but that interface is outside this project.

## Planned exclusive server

One `sorting_arm_skills` node will own:

- one arm `MoveGroupInterface`;
- one `PlanningSceneInterface`;
- one `GripperCommand` client;
- Pick, Place, and Home action servers;
- reusable sequence objects; and
- one joinable worker.

Goal callbacks validate basic shape and reject a goal if manipulation is already
owned. Accepted work runs outside executor callbacks. Feedback is published at
segment boundaries. Internal typed results are mapped one-to-one into action results.

The first milestone does not promise mid-controller-command preemption. A cancel
request is remembered, the active physical segment returns its truthful result, and
the sequence stops before starting the next segment.

## Planned executive package

The future package will contain at least:

```text
sorting_arm_executive/
├── CMakeLists.txt
├── package.xml
├── behavior_trees/
│   ├── sorting_cycle.xml
│   └── sorting_recovery.xml
├── config/
│   └── sorting.yaml
├── include/sorting_arm_executive/
│   ├── target_config.hpp
│   ├── static_object_source.hpp
│   ├── bt_nodes.hpp
│   └── executive_node.hpp
├── src/
│   ├── target_config.cpp
│   ├── static_object_source.cpp
│   ├── static_object_source_main.cpp
│   ├── bt_nodes.cpp
│   ├── executive_node.cpp
│   └── executive_main.cpp
└── test/
    ├── test_target_config.cpp
    ├── test_tree_order.cpp
    └── test_recovery_policy.cpp
```

Names may change when Step 12 starts. The responsibilities may not drift across the
skills/executive boundary.

## Fixed-source configuration

The first provider will read object IDs, opaque labels, center poses, primitive
dimensions, and deterministic object order from YAML. The executive also reads
ordered destination slots grouped by label.

Validation rejects:

- duplicate or empty object IDs;
- unsupported or empty labels;
- wrong frames;
- malformed dimensions/poses;
- duplicate destination slots; and
- too few slots for any label in the snapshot.

## Baseline tree

```text
Sequence
├── DetectObjects
├── SyncObjects
├── ForEach detected object, in snapshot order
│   └── Sequence
│       ├── SelectPlaceSlot
│       ├── Pick
│       └── Place
└── Home
```

The action/service leaves are asynchronous from the BehaviorTree’s perspective. The
tree’s first milestone is fail-fast: the first failed leaf stops the cycle and
surfaces its typed result.

## Recovery tree

Recovery is added after the baseline and perception contracts have independent
evidence:

```text
SortingCycle
└── Sequence
    ├── RetryWithinBudget(DetectObjects)
    ├── SyncObjects
    ├── ForEach detected object
    │   └── Sequence
    │       ├── SelectPlaceSlot
    │       ├── PickWithRecovery
    │       │   └── Fallback
    │       │       ├── Pick
    │       │       └── Sequence
    │       │           ├── IsRecoverableWorldState
    │       │           ├── RedetectAndSync
    │       │           └── RetryPickOnce
    │       └── PlaceWithRecovery
    │           └── Fallback
    │               ├── PlaceSelectedSlot
    │               ├── PlaceAlternateMatchingSlot
    │               └── PlaceSafeDrop
    └── Home
```

A Pick retry is legal only when the result says the object remains in the world and
the skill has retreated. A Place fallback is legal only when the result says the
object remains attached. Invalid input, controller unavailability, scene-invariant
failure, or unknown object state bypass retry and stop the task.

When recovery exhausts, a cleanup subtree attempts Home only if no object remains
attached. If an object is attached, it attempts the configured safe drop first.
Cleanup never replaces the causal failure, and a safe drop remains a partial task
failure.

Skip-and-continue and resuming a partial task are not part of this bounded policy.

## Slot allocation

Slot allocation is pure policy:

1. Take the object’s opaque label.
2. Look up the ordered slot list for that label.
3. Take the next unused slot.
4. Fail for unknown labels or exhausted slots.
5. Never allow two objects to use the same slot.

No joint target, planner name, gripper position, touch link, or collision operation
appears in this code.

## One-cycle guard

The fixed YAML snapshot describes a simulator starting condition. After a successful
or partially executed cycle, calling it again without resetting Gazebo would make the
snapshot false. The initial executive therefore accepts one cycle per reset and
rejects a second trigger with a specific explanation.

Perception can later remove or redesign that guard because it observes current state.

## Testing order

1. Parse and validate fixed objects/slots with pure tests.
2. Test slot allocation, unknown label, and exhaustion.
3. Tick the tree with fake Detect/Sync/Pick/Place/Home leaves.
4. Prove exact success order.
5. Inject failure at every leaf and prove later leaves do not run.
6. Run each real action independently from the CLI.
7. Run the executive against the fixed provider.
8. Only then add bringup wiring and the full cycle command.
9. Inject each recoverable result with fake leaves and prove the exact fallback order.
10. Inject terminal failures and prove no retry occurs.
11. Prove tree halt and executive shutdown reach the active child and preserve its
    final object state.
12. Demonstrate controlled runtime failures: an unavailable detection, a displaced
    object or missed grasp, and a blocked primary placement slot.

## Acceptance evidence

The fixed-source milestone passes only when:

- four objects are synchronized;
- calls occur in Detect → Sync → four Pick/Place pairs → Home order;
- each object occupies a distinct slot matching its label;
- no attached object remains;
- the arm reaches measured `home`;
- the first typed failure stops the tree; and
- the second static trigger is rejected until reset.

The supervised recovery milestone passes only when:

- a temporary detection failure succeeds within its retry budget;
- a missed grasp performs redetection and one Pick retry;
- a blocked selected slot uses another unused matching slot;
- an attached object that cannot reach a matching slot reaches the safe-drop pose and
  causes a partial-failure result;
- terminal controller, scene, and unknown-state failures are never retried;
- halting the tree or shutting down the executive cancels the active child goal and
  reports the resulting object state; and
- every exhausted path preserves the original typed cause and never reports success.
