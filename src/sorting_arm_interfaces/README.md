# sorting_arm_interfaces

Shared messages, services, and actions for the sorting cell. No executables, no
launch files, just the generated interface code everything else depends on.

## What's in here

| Type | Name | Used for |
|---|---|---|
| Action | `Pick`, `Place`, `Home` | The three manipulation goals `skill_server_node` serves. |
| Service | `DetectObjects` | What perception (or a fixed-pose provider) answers with the objects currently visible. |
| Service | `SyncObjects` | Pushing a detection snapshot into the MoveIt planning scene. |
| Message | `DetectedObject`, `SkillResult` | The object shape `DetectObjects` returns, and the typed result (`ok`, `native_code`, `phase`, `message`) every skill action reports. |

## Who depends on it

`sorting_arm_skills` and `sorting_arm_executive` both depend on this package and not
on each other, so the manipulation and task-policy layers only ever share a
contract, never implementation. `sorting_arm_perception` depends on it for
`DetectObjects` and `DetectedObject`.
