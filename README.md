# sorting_arm

**A simulated UR5e cell that finds cubes by colour with a wrist camera and sorts them into trays.**

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![MoveIt](https://img.shields.io/badge/MoveIt-2-blueviolet)
![License](https://img.shields.io/badge/license-MIT-green)

Four cubes sit on a table in front of the arm, two red and two blue. The arm looks at
the table, finds one cube by its colour, picks it up, drops it in the tray for that
colour, returns to its observation pose, and looks again. It repeats that until all
four cubes are sorted.

The arm is a UR5e with a Robotiq 2F-85 two-finger gripper, simulated in Gazebo.
**MoveIt 2** plans and executes every arm motion; the gripper is driven directly, not
through MoveIt. A **BehaviorTree** decides what happens next and in what order, and
hands the actual work off to the motion and detection layers underneath it.

Everything runs in a Docker container that this repository builds for you. Docker is
the only thing you install on the host. There is no ROS installation to set up and no
dependency to hunt down by hand.

## Table of contents

- [What one cycle does](#what-one-cycle-does)
- [Before you start](#before-you-start)
- [Quick start](#quick-start)
- [The scripts](#the-scripts)
- [Working inside the container](#working-inside-the-container)
- [Launch files](#launch-files)
- [Running individual nodes](#running-individual-nodes)
- [The packages](#the-packages)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)
- [Credits](#credits)
- [License](#license)

## What one cycle does

For each cube, in order:

1. **Detect**: the wrist camera takes one RGB-D snapshot and the cube's colour and
   world position come back from it.
2. **Plan**: the colour picks a destination tray slot.
3. **Sync**: the table, trays, and every known cube are pushed into the MoveIt
   planning scene, so the planner treats them as obstacles.
4. **Pick**, then **place**, then commit that the slot is filled.
5. **Home**: return to the observation pose and start again.

> [!NOTE]
> Detection runs once per cube, not once for the whole run. The arm looks again after
> every placement rather than trying to remember four positions from a single glance.
> The BehaviorTree runs exactly one cycle per process start and stops on the first
> failure instead of retrying. Restart the launch to run another cycle.

## Before you start

| Requirement | Why |
|---|---|
| Docker Engine + the compose plugin | Builds and runs the container everything else lives in |
| Linux desktop or WSL2 | The host platform the scripts target |
| `x11-xserver-utils` *(usually)* | Gazebo and RViz open as windows on your host; the scripts call `xhost +local:` for you if `xhost` is present. Some WSLg setups display windows without it |
| NVIDIA container runtime *(optional)* | Detected automatically; force it with `--gpu` or `--no-gpu` on any script |

## Quick start

```bash
git clone https://github.com/Kou-shik2004/sorting_arm.git
cd sorting_arm
./scripts/demo
```

> [!TIP]
> The first run is slow: it builds the Docker image and the ROS workspace before
> anything launches. Later runs skip straight to the launch.

`./scripts/demo` starts the container if it isn't already up, builds the ROS workspace
with `colcon build --symlink-install` if it hasn't been built yet, then launches the
full application. Later runs skip straight to the launch. Gazebo and RViz windows
should open, and the arm should start sorting once the controllers come up. `Ctrl-C`
stops the demo and leaves the container running; run `./scripts/demo` again to start
another cycle.

## The scripts

```bash
./scripts/build   # build the image, start nothing
./scripts/start   # bring the container up, detached, idle
./scripts/enter   # open a shell in it
./scripts/demo    # build the workspace if needed, then run the full demo
```

| Script | Does | Doesn't |
|---|---|---|
| `build` | Builds the base image | Read `--dev` or `--gpu`. It never brings anything up |
| `start` | Brings the container up detached | Launch anything inside it |
| `enter` | Opens a shell, starting the container first if it's down, sourced through `.docker/entrypoint.sh` | Build the workspace for you, though it warns if `install/setup.bash` is missing |
| `demo` | Starts the container if needed, builds the workspace if it hasn't been built, runs `app.launch.xml` | Rebuild the workspace on every run (only the first) |

`start`, `enter`, and `demo` all accept `--dev`, which adds named volumes for VS Code
Server and bash history so they survive a container rebuild, and `--gpu` / `--no-gpu`,
which force the NVIDIA compose fragment on or off. `build` reads neither. It only
builds the image. The container itself is named `sorting_arm`, and this repository is
bind-mounted into it at `/sorting_arm_ws`.

## Working inside the container

Once you're in with `./scripts/enter`, three shell functions from `.docker/bashrc.sh`
cover the usual loop:

```bash
cbuild      # colcon build --symlink-install
ctest_all   # colcon test, then colcon test-result --verbose
cdeps       # rosdep install --from-paths src --ignore-src -r -y
```

## Launch files

`sorting_arm_bringup` holds every entry point, staged from a bare URDF up to the full
application. Run each with:

```bash
ros2 launch sorting_arm_bringup <file> [arg:=value ...]
```

| File | Brings up | Arguments (default) |
|---|---|---|
| `display.launch.xml` | URDF + RViz only, no MoveIt, no Gazebo | none |
| `moveit.launch.xml` | Mock hardware + MoveIt planning, no Gazebo | none |
| `sim.launch.xml` | Gazebo + MoveIt | `gui` (`true`), `rviz_config` (`rviz/final_moveit.rviz`) |
| `skills.launch.xml` | Gazebo + the skill server, no executive | `skills_config` (skills package's `config/skills.yaml`) |
| `perception.launch.xml` | Just the detector node, attaching to a camera already publishing (Gazebo isn't started here) | `show_viewer` (`true`), `viewer_scale` (`1.0`) |
| `camera_validation.launch.xml` | Gazebo + MoveIt + the detector, for jogging the arm while watching detection | none |
| `app.launch.xml` | Everything (the one `./scripts/demo` runs) | `skills_config`, `executive_config`, `use_camera_provider` (`true`), `show_viewer` (`true`) |

`display.launch.xml` is good for checking the model itself: a
`joint_state_publisher_gui` window lets you drag joints and watch the robot move in
RViz. `moveit.launch.xml` plans and previews motion without waiting for a physics
simulation to start. `skills.launch.xml` lets you call `Pick`, `Place`, or `Home` by
hand and watch each one happen in isolation.

On `app.launch.xml`, `use_camera_provider:=false` swaps the wrist camera for
`fixed_object_provider`, which reads hardcoded cube positions from
`src/sorting_arm_executive/config/sorting.yaml` and answers the same detection
service, useful for testing pick and place without perception in the loop.
`show_viewer` is passed straight through to `perception.launch.xml` and only does
anything when the camera provider is running.

```bash
ros2 launch sorting_arm_bringup sim.launch.xml gui:=false
ros2 launch sorting_arm_bringup camera_validation.launch.xml
ros2 launch sorting_arm_bringup app.launch.xml use_camera_provider:=false
ros2 launch sorting_arm_bringup app.launch.xml show_viewer:=false
```

## Running individual nodes

| Command | What it is |
|---|---|
| `ros2 run sorting_arm_skills skill_server_node` | Hosts the `Pick`, `Place`, and `Home` actions |
| `ros2 run sorting_arm_skills motion_demo` | Runs one motion; `operation` is `named`, `joint`, `pose`, `cartesian`, or `apply_scene` |
| `ros2 run sorting_arm_skills sequence_demo` | Runs a scripted pick-and-place sequence |
| `ros2 run sorting_arm_executive executive_node` | Runs the BehaviorTree |
| `ros2 run sorting_arm_executive fixed_object_provider` | Answers `DetectObjects` from fixed poses |
| `ros2 run sorting_arm_perception camera_object_provider` | Answers `DetectObjects` from the wrist camera |

These normally come up through the launch files above, but you can also talk to a
running skill server or detector directly:

```bash
ros2 service call /detect_objects sorting_arm_interfaces/srv/DetectObjects "{expected_count: 4}"
ros2 action send_goal /home sorting_arm_interfaces/action/Home {}
```

## The packages

| Package | Owns |
|---|---|
| [`sorting_arm_description`](src/sorting_arm_description/README.md) | The UR5e and Robotiq 2F-85 URDF/xacro, meshes, and physical parameters |
| [`sorting_arm_moveit`](src/sorting_arm_moveit/README.md) | MoveIt configuration: planning groups, named states, kinematics, controllers |
| [`sorting_arm_bringup`](src/sorting_arm_bringup/README.md) | Launch files and controller config that wire everything together |
| [`sorting_arm_interfaces`](src/sorting_arm_interfaces/README.md) | Shared messages, services, and actions |
| [`sorting_arm_skills`](src/sorting_arm_skills/README.md) | Manipulation: one node hosting `Pick`, `Place`, and `Home` over MoveIt and `GripperCommand` |
| [`sorting_arm_perception`](src/sorting_arm_perception/README.md) | One-shot RGB-D cube detection, serving `DetectObjects` |
| [`sorting_arm_executive`](src/sorting_arm_executive/README.md) | The BehaviorTree that runs the sorting cycle end to end |

See [docs/architecture.md](docs/architecture.md) for how these fit together.

## Troubleshooting

| Symptom | Try |
|---|---|
| No Gazebo or RViz window opens | Install `x11-xserver-utils` on the host so the scripts can call `xhost +local:` for you |
| `docker: permission denied` | `sudo usermod -aG docker $USER`, then log out and back in |
| A controller stays inactive | Inside the container, `ros2 control list_controllers`: `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` should all read `active` |
| Detection finds nothing | Launch `perception.launch.xml show_viewer:=true` and watch `/perception/debug_image`; the colour bands and search volume are in `src/sorting_arm_perception/config/perception.yaml` |
| The URDF won't parse | `xacro src/sorting_arm_description/urdf/sorting_arm.urdf.xacro \| check_urdf -` |
| Nothing has been built yet | `cdeps` then `cbuild` inside the container |
| A script refuses to use your GPU | Requesting a GPU that isn't there is a hard failure by design; pass `--no-gpu` |

## Documentation

| Doc | Covers |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Package boundaries, the cycle, frames, interfaces |
| [docs/container.md](docs/container.md) | The image, the compose fragments, adding persistent state |
| [docs/README.md](docs/README.md) | Index of everything below |
| [docs/skills/](docs/skills/) | `sorting_arm_skills` notes, one file per source file |
| [docs/perception/](docs/perception/) | The one-shot RGB-D provider and its viewer |
| [docs/executive/](docs/executive/) | The BehaviorTree, blackboard, ROS leaves, runtime limits |
| [docs/container/](docs/container/) | Dockerfile, compose, devcontainer, deployment |
| [docs/project/](docs/project/) | Overview, architecture, recorded decisions, workflow |
| [docs/rca/](docs/rca/) | Write-ups of real failures and how they were fixed |

## Credits

The UR5e description comes from ROS-Industrial's `ur_description`, and the gripper
description from `robotiq_description`, both BSD-3-Clause. Both were copied into
`sorting_arm_description` rather than depended on. See [NOTICE](NOTICE) for the exact
file list and license text. The project is built directly on MoveIt 2, `ros2_control`,
Gazebo, and BehaviorTree.CPP rather than reimplementing any of them.

## License

This project's own code is MIT. See [LICENSE](LICENSE). The vendored UR5e and
Robotiq 2F-85 files keep their original BSD-3-Clause license. See [NOTICE](NOTICE).
