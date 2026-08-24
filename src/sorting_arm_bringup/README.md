# sorting_arm_bringup

Every launch file for the sorting cell lives here. This package wires the other
packages together and owns the small simulation-startup helper that spawns the
cubes. The launch path follows the same stages as the project: URDF first, then
mock-hardware MoveIt, Gazebo, and finally the full application.

## Launch files

| File | Brings up | Notes |
|---|---|---|
| `display.launch.xml` | URDF + RViz, `joint_state_publisher_gui` | No MoveIt, no Gazebo. Fastest way to look at the robot and jog its joints by hand. |
| `moveit.launch.xml` | Mock hardware, `ros2_control`, `move_group`, RViz | MoveIt planning without Gazebo. The step before simulation. |
| `sim.launch.xml` | Gazebo, the ROS-Gazebo bridge, controllers, `move_group`, RViz | `gui:=false` runs Gazebo headless. `rviz_config` picks the RViz layout. |
| `skills.launch.xml` | `sim.launch.xml` + `skill_server_node` | The full manipulation stack, no executive and no perception. Drive it by hand with `ros2 action send_goal`, or run `sorting_arm_skills`' `sequence_demo` against it. |
| `perception.launch.xml` | `camera_object_provider` | Just the detector. `show_viewer` and `viewer_scale` control the live detection window. |
| `camera_validation.launch.xml` | `sim.launch.xml` + `perception.launch.xml` | Gazebo, RViz, and live detection together, so you can jog the arm with MoveIt while watching what the camera sees. |
| `app.launch.xml` | Everything | The full application: Gazebo, `skill_server_node`, perception, the cube spawner, and the executive. Spawns `object_count` random cubes and runs one sort cycle on startup. |

```bash
ros2 launch sorting_arm_bringup app.launch.xml object_count:=4
```

is the one-command demo. `object_count` (2–8) sets how many cubes `cube_spawner`
drops at random spaced positions and how many the executive sorts; each launch is a
fresh random arrangement.

## Parameters you might change

| File | Controls |
|---|---|
| `config/ros2_controllers.yaml` | Which `ros2_control` controllers are spawned, and their gains. |
| `config/gz_bridge.yaml` | Which topics cross the ROS-Gazebo bridge. |
| `worlds/sorting_cell.sdf` | The fixed table and trays in simulation. Skill and executive code measures their geometry from this file, so a change here needs matching changes there. |
| `src/cube_spawner.cpp` | The one-shot C++ node that spawns the run's random cubes via the gz create service, used only by `app.launch.xml`. |
| `rviz/*.rviz` | The RViz layouts each launch file opens by default. |
