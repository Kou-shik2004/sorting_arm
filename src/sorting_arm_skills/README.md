# sorting_arm_skills

Pick, Place, and Home, plus the motion and scene primitives they're built from.
One node, three action servers.

## Running it

Normally you don't start this by hand, `sorting_arm_bringup`'s `skills.launch.xml`
or `app.launch.xml` brings up `skill_server_node` alongside Gazebo. With
`skills.launch.xml` running, Home needs no setup:

```bash
ros2 action send_goal /home sorting_arm_interfaces/action/Home {}
```

Pick and Place both act on an object already pushed into the planning scene by
`/sync_objects`, so driving them by hand means calling that service first with a
`DetectedObject` for whatever you want to pick. The easier path is the scripted
four-cube demo, which does the sync for you:

```bash
ros2 run sorting_arm_skills sequence_demo --ros-args --params-file \
  $(ros2 pkg prefix sorting_arm_skills)/share/sorting_arm_skills/config/sequence_demo.yaml
```

## Other executables

`motion_demo` is a one-shot proof of a single motion primitive, useful when
`skill_server_node`'s combined behavior makes it hard to tell which piece is at
fault:

```bash
ros2 run sorting_arm_skills motion_demo --ros-args -p operation:=named -p target_name:=home
ros2 run sorting_arm_skills motion_demo --ros-args -p operation:=pose -p x:=0.4 -p y:=0.1 -p z:=0.5 \
  -p qx:=1.0 -p qy:=0.0 -p qz:=0.0 -p qw:=0.0
```

`operation` is one of `named`, `joint`, `pose`, `cartesian`, or `apply_scene`; each
takes its own set of targets.

## Parameters you might change

| File | Controls |
|---|---|
| `config/skills.yaml` | Planning timeouts and scaling, Cartesian step size, gripper timing, pre-grasp candidate limits, and the table/tray collision geometry Pick and Place check against. The tray and table geometry is measured from `sorting_arm_bringup/worlds/sorting_cell.sdf`; if the world changes, these need re-measuring. |
| `config/sequence_demo.yaml` | The four-cube catalogue `sequence_demo` drives: object poses, sizes, and destinations. |
