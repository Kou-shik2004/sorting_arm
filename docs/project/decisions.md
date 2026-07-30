# Architectural decisions

## Current decisions

| ID | Decision | Reason |
|---|---|---|
| D1 | Keep description, MoveIt config, bringup, interfaces, skills, and executive as separate packages. | Generated robot configuration and authored application logic evolve at different rates. |
| D2 | Use `MoveGroupInterface` for arm planning and execution. | The task needs named, joint, pose, and short Cartesian operations without a second planning abstraction. |
| D3 | Use `GripperCommand` directly for the gripper. | Its result exposes `stalled` and `reached_goal`, which are part of grasp verification. |
| D4 | Use explicit ROS 2 Pick, Place, and Home actions. | Asynchronous BehaviorTree leaves need a process boundary to the exclusive manipulation owner, including phase feedback, cancellation, typed failures, and final object state. |
| D5 | Use synchronous planning-scene apply calls and immediate state queries. | The next motion must observe the exact attachment/world state without timing guesses. |
| D6 | Use one arm MoveGroup and reject concurrent skill goals. | Competing plans and executions would make ownership and cancellation ambiguous. |
| D7 | Keep fixed objects and slots in YAML behind `DetectObjects`. | Perception can later replace the provider without changing the executive or skills. |
| D8 | Use BehaviorTree.CPP only at the orchestration layer. | Task order and later recovery policy benefit from composable, testable control flow while motion primitives and scene transitions remain reusable. |
| D9 | Stop the first milestone on the first typed failure. | Recovery must not be implied before its policies and tests exist. |
| D10 | Use the normal single-threaded ROS executor plus an owned C++20 worker. | Callbacks remain responsive while blocking manipulation work has explicit lifetime. |
| D11 | Do not use MoveIt Task Constructor, MoveItCpp, legacy `pick()/place()`, or a generic motion action. | They either duplicate the required contract or move task policy into the wrong layer. |
| D12 | Keep Gazebo physics out of per-push CI. | Pure logic and mock-hardware integration are deterministic; physics remains manual/nightly. |
| D13 | Grow CI through a dedicated per-change CI image, independent development-image validation, and manual/scheduled simulation smoke. | The CI image gives every source change one declared clean-build environment without coupling ordinary feedback to agent tooling, development volumes, or flaky physics. |
| D14 | Keep wrist-camera perception behind the same `DetectObjects` contract as the fixed source. | Calibration, vision, and transform concerns can evolve without changing manipulation skills or task policy. |
| D15 | Add bounded recovery only after the fail-fast cycle is proven. | Recovery must branch from measured failure categories and truthful object state, not hide defects behind arbitrary retries. |
| D16 | Start exactly one autonomous sorting cycle after explicit runtime readiness, then remain idle for inspection. | This repository is the complete sorting application; a top-level task protocol belongs only in a future consuming project with a demonstrated caller. |
| D17 | Use a Conventional Commit type with a simple human-written subject and a detailed explanatory body. | The prefix makes history scannable and tool-friendly, while the body preserves the author’s reasoning, verification, and limitations for future readers. |

## Deferred decisions

Skip-and-continue after exhausted recovery, resuming a partially completed task,
continuous observability, and Pilz adoption remain deferred. Exact retry budgets and
the safe-drop pose require configuration and runtime evidence at the recovery gate.
