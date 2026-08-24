# sorting_arm_executive

The BehaviorTree that runs one re-observe-detect-sort-home cycle over however many
cubes were spawned this run.

## Running it

This is what `app.launch.xml` brings up as the top of the stack; it isn't meant
to run standalone, since it needs `skill_server_node` and the camera `DetectObjects`
provider (`sorting_arm_perception`) already up.

```bash
ros2 launch sorting_arm_bringup app.launch.xml object_count:=4
```

`object_count` (2–8) is a single `app.launch` argument shared with the cube spawner: it
sets both how many cubes are spawned and how many the cycle sorts. Valid range is
1 up to the tray capacity (8). Each launch spawns a fresh random arrangement, so
restart the launch to run another cycle.

## Parameters you might change

`config/executive.yaml` holds only tunables — the timeouts:

| Key (in `config/executive.yaml`) | Controls |
|---|---|
| `*_timeout_s` | How long the tree waits on each step (readiness, detection, sync, an action, a cancel) before treating it as failed. |

Everything that is a fixture, not a knob, is hardcoded rather than parameterised:
the two tray placement grids live as constants in `bt_nodes.cpp` (they mirror the
tray poses in `sorting_cell.sdf`), and the red-cube→red-tray / blue-cube→blue-tray
mapping is fixed in code. `object_count` comes from the launch argument, not this file.

`behavior_trees/sorting_cycle.xml` is the tree itself: each cycle re-detects the
remaining cubes, plans one placement, syncs the scene, Sorts that cube, commits it,
then Homes.
