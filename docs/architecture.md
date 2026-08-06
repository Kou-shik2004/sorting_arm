# Architecture

Seven packages, each with one job. This page is the map; the package READMEs cover
how to run each piece, and `docs/rca/` covers what broke and why.

## Packages

| Package | Owns |
|---|---|
| `sorting_arm_description` | The UR5e and Robotiq 2F-85 URDF/xacro, meshes, and physical parameters |
| `sorting_arm_moveit` | MoveIt configuration: planning groups, named states, kinematics, controllers |
| `sorting_arm_bringup` | Launch files and controller config that wire everything together |
| `sorting_arm_interfaces` | Shared messages, services, and actions (`Pick`, `Place`, `Home`, `DetectObjects`, `SyncObjects`) |
| `sorting_arm_skills` | Manipulation: `skill_server_node` hosts Pick, Place, and Home over MoveIt and `GripperCommand` |
| `sorting_arm_perception` | One-shot RGB-D cube detection, serving `DetectObjects` |
| `sorting_arm_executive` | The BehaviorTree that runs one detect-pick-place-home cycle end to end |

`sorting_arm_description` and `sorting_arm_moveit` don't depend on anything else in
the workspace. `sorting_arm_interfaces` depends on nothing but message packages, so
that `sorting_arm_skills` and the executive can share a contract without depending on
each other. `sorting_arm_bringup` holds no logic of its own, only launch orchestration.

## The cycle

```text
executive ──DetectObjects──> perception (camera) or a fixed-pose provider
    │
    ├─ SyncObjects   (push detections into the MoveIt planning scene)
    ├─ Pick           (per object)
    ├─ Place           (per object)
    └─ Home            (once, at the end)
```

`skill_server_node` is the one process behind Pick, Place, and Home. There's no
separate binary per skill; the executive calls all three actions on the same node.
Each action drives `MoveGroupInterface` for arm motion and `GripperCommand` directly
for the gripper. MoveIt planning is never used for the gripper joint itself.

Detection is swappable: `sorting_arm_perception`'s `camera_object_provider` and
`sorting_arm_executive`'s `fixed_object_provider` both implement `DetectObjects`, and
the executive doesn't know which one it's talking to. `app.launch.xml`'s
`use_camera_provider` argument picks between them.

The executive runs exactly one cycle per process start, not a loop, and it stops on
the first failure rather than retrying. That's deliberate while the happy path is
still the thing being proven.
