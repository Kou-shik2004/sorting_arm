# sorting_arm_skills

Sort (one native MoveIt Task Constructor pick-and-place task) and Home, plus the
motion and scene primitives they're built from. One node, two action servers.

## Running it

Normally you don't start this by hand, `sorting_arm_bringup`'s `skills.launch.xml`
or `app.launch.xml` brings up `skill_server_node` alongside Gazebo. With
`skills.launch.xml` running, Home needs no setup:

```bash
ros2 action send_goal /home sorting_arm_interfaces/action/Home {}
```

Sort acts on an object already pushed into the planning scene by `/sync_objects`,
so driving it by hand means calling that service first with a `DetectedObject` for
whatever you want to sort. The easier path is the scripted four-cube demo, which
does the sync for you:

```bash
ros2 run sorting_arm_skills sequence_demo --ros-args --params-file \
  $(ros2 pkg prefix sorting_arm_skills)/share/sorting_arm_skills/config/sequence_demo.yaml
```

## Parameters you might change

| File | Controls |
|---|---|
| `config/skills.yaml` | Planning timeouts and scaling, Cartesian step size, MTC solution count, grasp geometry (offset, yaw delta, approach/retreat heights), and the table/tray collision geometry the MTC task checks against. The tray and table geometry is measured from `sorting_arm_bringup/worlds/sorting_cell.sdf`; if the world changes, these need re-measuring. |
| `config/sequence_demo.yaml` | The four-cube catalogue `sequence_demo` drives: object poses, sizes, and destinations. |
