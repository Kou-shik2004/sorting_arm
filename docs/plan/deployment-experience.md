# Deployment experience

## Status

This document defines the final user-visible contract. The current Compose service
and Dockerfile remain development infrastructure, while the CI image remains
build/test infrastructure; neither implements this runtime experience.

## User outcome

A reviewer starts from a clean checkout and follows one documented command:

```bash
docker compose --profile demo up --build
```

The exact profile and image names may be settled during the deployment gate, but the
experience may not require entering the container or coordinating several terminals.

The demo launches the complete sorting cell, waits for explicit readiness, runs one
autonomous sorting cycle, and leaves the final scene open for inspection. This
repository does not expose a top-level task action. A consuming project may wrap the
application later without adding that unused interface here.

## Runtime image contract

The runtime image is separate from both the development and CI images. A multi-stage
build may compile the workspace in a builder stage, but the delivered stage contains
only the installed workspace, required ROS 2 runtime dependencies, Gazebo/MoveIt
assets, and the application entrypoint.

The runtime service excludes:

- compilers, debuggers, editor servers, and development shells;
- Claude, Codex, Node, Bun, and other agent tooling;
- source bind mounts and persistent development volumes;
- build and test artifacts not required at runtime; and
- a `sleep infinity` command that requires a later `docker exec`.

Gazebo, MoveIt, RViz, and their graphics dependencies are inherently substantial.
The target is a materially smaller and cleaner image than the development image, not
an unrealistically tiny robotics image.

## Compose profiles

The final Compose application provides two paths:

- `demo` starts the graphical Gazebo and RViz experience plus the complete
  application;
- `headless` starts a bounded server-side scenario, captures evidence, returns an
  exit status, and shuts down cleanly.

Supported display forwarding, GPU acceleration, and the CPU-compatible path are
documented beside the final commands. The runtime service does not inherit the
development container's source, agent configuration, or editor volumes.

## Bringup lifecycle

The application bringup owns this dependency order:

```text
Gazebo world
  → robot description and controllers
  → MoveIt
  → manipulation skill server
  → selected object provider
  → BehaviorTree executive
  → one automatic sorting cycle
```

Process startup alone is not readiness. The executive waits for active controllers,
MoveIt readiness, the Pick/Place/Home action servers, and the selected
`DetectObjects` provider. Fixed sleeps may not decide when sorting begins.

The executive runs once per simulator reset. After success, partial failure, or
terminal failure, it remains idle and does not silently restart the scene.

## Visible normal cycle

The graphical demonstration makes these states observable:

1. Gazebo shows the UR5e, Robotiq gripper, table, boxes, and label-matched trays.
2. The camera view shows detected boxes and labels.
3. RViz shows the synchronized planning scene and collision-aware trajectories.
4. The tree status and structured logs show the active object, Pick/Place/Home
   operation, physical phase, and completed-object count.
5. Each successful Pick attaches exactly one object to `tcp`.
6. Each successful Place detaches that object into a distinct matching slot.
7. After the last object, Home executes and the executive reports completion.

The exact tree visualizer is chosen when the executive is implemented. At minimum,
the same state must be visible through structured logs; a graphical tree monitor is
preferred for the final demo.

## Recovery presentation

The normal profile must be deterministic and may never enter recovery naturally.
Recovery is therefore demonstrated through one controlled scenario, not by adding
random failure.

Suitable demonstrations include:

- displacing one box after its first observation so Pick retreats, redetects,
  synchronizes, and retries; or
- blocking the selected tray slot so Place chooses another unused matching slot.

The display and logs must make the original failure, selected recovery branch, and
final object state visible. If bounded recovery exhausts, the application reports
partial or terminal failure and never prints success merely because cleanup ran.

## Completion contract

A successful normal cycle leaves:

- every detected object in a distinct slot matching its opaque label;
- no attached collision object;
- the MoveIt world consistent with the visible Gazebo scene;
- the arm at the measured SRDF `home`;
- the BehaviorTree in success;
- one concise completion summary in the logs; and
- Gazebo and RViz open until the user stops Compose.

The headless profile checks equivalent machine-observable facts, enforces an overall
timeout, collects logs, and exits cleanly.

## Acceptance evidence

The deployment experience is complete only when:

- a clean machine can build the runtime image from documented prerequisites;
- one Compose command starts the graphical application without manual container
  entry;
- explicit readiness gates the one automatic cycle;
- the normal scene sorts all objects and remains inspectable;
- the controlled recovery scenario follows its documented branch;
- the headless profile terminates within its timeout and retains useful evidence;
- the runtime image contains no development-only mounts, tools, or startup command;
  and
- the documented shutdown leaves no orphaned simulator or ROS processes.
