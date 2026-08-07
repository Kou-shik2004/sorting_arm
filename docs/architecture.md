# Architecture

Seven packages, each with one job. This page is the map; the package READMEs cover how
to run each piece, and `docs/rca/` covers what broke and why.

## Packages

| Package | Owns |
|---|---|
| `sorting_arm_description` | The UR5e and Robotiq 2F-85 URDF/xacro, meshes, and physical parameters |
| `sorting_arm_moveit` | MoveIt configuration: planning groups, named states, kinematics, controllers |
| `sorting_arm_bringup` | Launch files and controller config that wire everything together |
| `sorting_arm_interfaces` | Shared messages, services, and actions (`Pick`, `Place`, `Home`, `DetectObjects`, `SyncObjects`) |
| `sorting_arm_skills` | Manipulation: `skill_server_node` hosts Pick, Place, and Home over MoveIt and `GripperCommand` |
| `sorting_arm_perception` | One-shot RGB-D cube detection, serving `DetectObjects` |
| `sorting_arm_executive` | The BehaviorTree that runs the sorting cycle end to end |

`sorting_arm_description` and `sorting_arm_moveit` don't depend on anything else in
the workspace. `sorting_arm_interfaces` depends on nothing but message packages, so
that `sorting_arm_skills` and the executive can share a contract without depending on
each other. `sorting_arm_bringup` holds no logic of its own, only launch
orchestration.

## The cycle

The tree in `src/sorting_arm_executive/behavior_trees/sorting_cycle.xml` goes home
once to take the first look at the table, then repeats a detect, sync, pick, place,
home sequence once per object rather than detecting everything up front:

```text
Home (observation pose)
repeat, once per object:
    detect one cube
    plan which tray slot it goes to
    sync every known object into the MoveIt planning scene
    Pick
    Place
    commit the slot as filled
    Home
```

It re-detects before every pick rather than working from a single snapshot of all
four cubes, so a cube that shifted while an earlier one was being placed is still
found where it actually is.

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

## Frames

The planning frame is `world`. Pose goals command the `tcp` link, and every object
position the executive works with is that object's geometric centre, not a corner or
a grasp point.

## Where the numbers live

`src/sorting_arm_executive/config/sorting.yaml` sets `cycle_object_count` (4) and
`destination_slots`, four tray positions labelled `red, red, blue, blue`. Tray and
table geometry comes from `src/sorting_arm_bringup/worlds/sorting_cell.sdf`, and the
perception search volume in `sorting_arm_perception/config/perception.yaml` is
measured from that same world file.

## Interfaces

| Type | Kind | Carries |
|---|---|---|
| `Pick` | Action | Job describing which object to grasp |
| `Place` | Action | Job describing where to put it down |
| `Home` | Action | Return to the observation pose |
| `DetectObjects` | Service | One detection pass, camera or fixed |
| `SyncObjects` | Service | Push known objects into the planning scene |
| `DetectedObject` | Message | Label and world-frame centre for one object |
| `SkillResult` | Message | `ok`, `native_code`, `phase`, `message`, carried through every layer of a skill call |
