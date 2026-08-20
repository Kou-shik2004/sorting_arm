# sorting_arm_executive

The BehaviorTree that runs one detect-sort-home cycle, plus a fixed-pose object
provider for testing the cycle without a camera in the loop.

## Running it

This is what `app.launch.xml` brings up as the top of the stack; it isn't meant
to run standalone, since it needs `skill_server_node` and a `DetectObjects`
provider already up.

```bash
ros2 launch sorting_arm_bringup app.launch.xml use_camera_provider:=true
```

Set `use_camera_provider:=false` to swap in `fixed_object_provider`, which answers
`DetectObjects` from `config/sorting.yaml`'s `fixed_objects` list instead of the
camera. Useful for testing the cycle itself without depending on perception.

The tree runs exactly once per process start. Restart the launch to run another
cycle.

## Parameters you might change

| Key (in `config/sorting.yaml`) | Controls |
|---|---|
| `cycle_object_count` | How many objects the cycle expects to detect and sort. |
| `destination_slots` | Where each labelled object goes: one centre pose per slot, matched to objects by label. |
| `fixed_objects` | The object poses `fixed_object_provider` reports. Only read when `use_camera_provider:=false`. |
| `*_timeout_s` | How long the tree waits on each step (readiness, detection, sync, an action, a cancel) before treating it as failed. |

`behavior_trees/sorting_cycle.xml` is the tree itself, detect, sync, then Sort
per object, then Home.
