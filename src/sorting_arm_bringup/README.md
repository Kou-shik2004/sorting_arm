# sorting_arm_bringup

Every launch file for the sorting cell lives here. Nothing else in this package
runs on its own; it exists to wire the other packages together, the same staged
path the project itself was built along: URDF first, then mock-hardware MoveIt,
then Gazebo, then the full application.

## Launch files

| File | Brings up | Notes |
|---|---|---|
| `display.launch.xml` | URDF + RViz, `joint_state_publisher_gui` | No MoveIt, no Gazebo. Fastest way to look at the robot and jog its joints by hand. |
| `moveit.launch.xml` | Mock hardware, `ros2_control`, `move_group`, RViz | MoveIt planning without Gazebo. The step before simulation. |
| `sim.launch.xml` | Gazebo, the ROS-Gazebo bridge, controllers, `move_group`, RViz | `gui:=false` runs Gazebo headless. `rviz_config` picks the RViz layout. |
| `skills.launch.xml` | `sim.launch.xml` + `skill_server_node` | The full manipulation stack, no executive and no perception. Drive it by hand with `ros2 action send_goal`, or run `sorting_arm_skills`' `sequence_demo` against it. |
| `perception.launch.xml` | `camera_object_provider` | Just the detector. `show_viewer` and `viewer_scale` control the live detection window. |
| `camera_validation.launch.xml` | `sim.launch.xml` + `perception.launch.xml` | Gazebo, RViz, and live detection together, so you can jog the arm with MoveIt while watching what the camera sees. |
| `app.launch.xml` | Everything | The full application: Gazebo, `skill_server_node`, perception (or the fixed-pose provider), and the executive. Runs one sort cycle on startup. |

```bash
ros2 launch sorting_arm_bringup app.launch.xml
```

is the one-command demo. `use_camera_provider:=false` swaps in a fixed-pose object
provider instead of the camera, useful for testing manipulation without perception
in the loop.

## Parameters you might change

| File | Controls |
|---|---|
| `config/ros2_controllers.yaml` | Which `ros2_control` controllers are spawned, and their gains. |
| `config/gz_bridge.yaml` | Which topics cross the ROS-Gazebo bridge. |
| `worlds/sorting_cell.sdf` | The table, trays, and cube starting poses in simulation. Skill and executive config files measure their geometry from this file, so a change here needs matching changes there. |
| `rviz/*.rviz` | The RViz layouts each launch file opens by default. |
