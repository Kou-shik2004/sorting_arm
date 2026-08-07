# Simulated Colour-Sorting Robot Arm

**A simulated UR5e cell that finds cubes by colour with a wrist camera and sorts them into trays.**

![ROS 2](https://img.shields.io/badge/ROS_2-Jazzy-22314E?logo=ros&logoColor=white)
![Gazebo](https://img.shields.io/badge/Gazebo-Harmonic-orange)
![MoveIt](https://img.shields.io/badge/MoveIt-2-blueviolet)
![License](https://img.shields.io/badge/license-MIT-green)



https://github.com/user-attachments/assets/712d5a2e-bca9-482f-95df-b3a75069dd2b

*Partway through, I clear every cube off the table by hand and set them back down
somewhere else. The arm still finds and sorts them, because it looks again before
each pick instead of working from one snapshot taken at the start.*

> [!NOTE]
> **Status:** the sorting cycle above is complete and runs end to end - this isn't a
> preview. The repository is still under active development: recovery behaviour in
> the BehaviorTree (the reason it's a BehaviorTree and not a script), more
> documentation, and code and naming cleanup are all in progress. Expect this repo to
> keep changing.

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
- [Running with or without a GPU](#running-with-or-without-a-gpu)
- [The scripts](#the-scripts)
- [Building the workspace](#building-the-workspace)
- [Launch files](#launch-files)
- [If the simulation is lagging](#if-the-simulation-is-lagging)
- [Running individual nodes](#running-individual-nodes)
- [The packages](#the-packages)
- [Troubleshooting](#troubleshooting)
- [Documentation](#documentation)
- [Developing in VS Code](#developing-in-vs-code)
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

Everything below runs inside the container this repo builds. On the host, you need:

**Docker Engine and the Compose plugin, required.** If `docker compose version`
already works, skip this. Otherwise, on Ubuntu or Debian:

```bash
curl -fsSL https://get.docker.com | sh
```

That installs Docker Engine and the Compose plugin together. See
[Docker's own install guide](https://docs.docker.com/engine/install/) if you'd rather
not pipe a script into `sh`, or you're on a distribution it doesn't cover.

**Your user in the `docker` group, required on Linux.** Without it, every Docker
command needs `sudo`:

```bash
sudo usermod -aG docker $USER
```

Log out and back in for this to take effect; `newgrp docker` only fixes the shell you
run it in, not new terminals. If you skip this step, `./scripts/*` detects the
permission error and falls back to `sudo docker` on its own, with a reminder printed
each time.

**An X server, usually already there.** Gazebo and RViz open as windows on your host.
On most Linux desktops nothing extra is needed; if windows fail to open, install
`x11-xserver-utils` so the scripts can call `xhost +local:` for you. Some WSLg setups
display windows with no extra configuration at all.

**Platform: Linux desktop or WSL2.** That's what the scripts target. There's no macOS
or native Windows support, and none planned.

**A GPU, optional.** NVIDIA, Intel or AMD integrated, or none - the scripts detect
what your host has and pick the fastest rendering path automatically. See
[Running with or without a GPU](#running-with-or-without-a-gpu) for what each path
needs, and [If the simulation is lagging](#if-the-simulation-is-lagging) if you don't
have one.

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

## Running with or without a GPU

`./scripts/start`, `enter`, and `demo` detect what your host has and pick the fastest
rendering path automatically. All three work; they only differ in speed.

**NVIDIA GPU.** Auto-detected through the NVIDIA Container Toolkit. Force it with
`--gpu`:

```bash
./scripts/start --gpu
```

This needs three separate things working together, and it's worth checking them as
one unit rather than guessing which is missing: the GPU itself, the host's NVIDIA
driver, and the NVIDIA Container Toolkit. One command proves all three at once:

```bash
docker run --rm --gpus all ubuntu nvidia-smi
```

If that fails, fix the [NVIDIA Container Toolkit install
guide](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/latest/install-guide.html)
before troubleshooting the scripts - they can't do anything the toolkit itself
doesn't support.

**Intel or AMD integrated graphics.** `/dev/dri` is passed through automatically
whenever it exists and no NVIDIA runtime was found; nothing to configure. Slower than
a discrete GPU, considerably faster than software rendering.

**Neither, or `--no-gpu`.** Gazebo renders through Mesa's `llvmpipe` on the CPU. It
runs anywhere, including a VM with no GPU passthrough, and it's the slowest of the
three:

```bash
./scripts/start --no-gpu
```

`--no-gpu` also doubles as a "give me the plainest possible container" switch if a
GPU you do have is the thing causing trouble.

## The scripts

```bash
./scripts/build   # build the image, start nothing
./scripts/start   # bring the container up, detached, idle
./scripts/enter   # open a shell in it
./scripts/demo    # build the workspace if needed, then run the full demo
./scripts/stop    # tear the container down
```

| Script | Does | Doesn't |
|---|---|---|
| `build` | Builds the base image | Read `--dev` or `--gpu`. It never brings anything up |
| `start` | Brings the container up detached | Launch anything inside it |
| `enter` | Opens a shell, starting the container first if it's down, sourced through `.docker/entrypoint.sh` | Build the workspace for you, though it warns if `install/setup.bash` is missing |
| `demo` | Starts the container if needed, builds the workspace if it hasn't been built, runs `app.launch.xml` | Rebuild the workspace on every run (only the first) |
| `stop` | Tears the container down | Touch the `--dev` named volumes; they survive for next time |

`start`, `enter`, `demo`, and `stop` all accept `--dev`, which adds named volumes for
VS Code Server and bash history so they survive a container rebuild, and `--gpu` /
`--no-gpu`, which force the NVIDIA compose fragment on or off. `build` reads neither.
It only builds the image. The container itself is named `sorting_arm`, and this
repository is bind-mounted into it at `/sorting_arm_ws`.

Every script is a thin wrapper around `docker compose` or `docker exec`. If you'd
rather not use them, here's what each one runs:

| Script | Runs |
|---|---|
| `build` | `docker compose -f docker-compose.yaml build` |
| `start` | `docker compose -f docker-compose.yaml [-f .docker/compose.gpu.yaml \| -f .docker/compose.dri.yaml] [-f .docker/compose.dev.yaml] up -d --build` |
| `enter` | Brings the container up first if it's down (same `up -d --build` as `start`), then `docker exec -it sorting_arm /entrypoint.sh bash` |
| `demo` | Brings the container up first if it's down, builds the workspace first if `install/setup.bash` is missing (`docker exec sorting_arm /entrypoint.sh colcon build --symlink-install`), then `docker exec -it sorting_arm /entrypoint.sh ros2 launch sorting_arm_bringup app.launch.xml` |
| `stop` | `docker compose -f docker-compose.yaml [...] down` |

The `-f` fragments in brackets are picked automatically, and at most one of the first
pair is ever added: `.docker/compose.gpu.yaml` when the NVIDIA runtime is detected
(or `--gpu` forces it), else `.docker/compose.dri.yaml` when `/dev/dri` exists (every
Intel/AMD laptop), then `.docker/compose.dev.yaml` whenever `--dev` is passed on top
of either. `start`, `enter`, and `demo` also run `xhost +local:` before bringing the
container up, so RViz and Gazebo windows are allowed through; `build` and `stop`
don't touch X11 at all, and a raw `docker compose up` skips it too - you'll need to
run it yourself.

If your user can't reach the Docker daemon without `sudo`, the scripts detect that
and fall back to running every `docker`/`docker compose` command above as
`sudo docker`, printing a reminder to fix it for good with
`sudo usermod -aG docker $USER` (then log out and back in).

## Building the workspace

Once you're in with `./scripts/enter`, the workspace is a regular colcon workspace:

```bash
colcon build --symlink-install
rosdep install --from-paths src --ignore-src -r -y   # only if a dependency is missing
```

`./scripts/demo` only ever builds the workspace once, the first time it finds no
`install/setup.bash`; it never rebuilds on a later run even if you've edited source.
After changing code, run `colcon build --symlink-install` yourself from inside
`./scripts/enter` before the next `./scripts/demo`. If a build fails partway through
but still produces `install/setup.bash`, `demo` will keep skipping straight to the
launch on every following run - build again to get a real error instead.

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
| `sim.launch.xml` | Gazebo + MoveIt | `gui` (`true`), `rviz_config` (`rviz/final_moveit.rviz`), `camera_width` (`1280`), `camera_height` (`960`), `camera_update_rate` (`30`) |
| `skills.launch.xml` | Gazebo + the skill server, no executive | `skills_config` (skills package's `config/skills.yaml`) |
| `perception.launch.xml` | Just the detector node, attaching to a camera already publishing (Gazebo isn't started here) | `show_viewer` (`true`), `viewer_scale` (`1.0`) |
| `camera_validation.launch.xml` | Gazebo + MoveIt + the detector, for jogging the arm while watching detection | none |
| `app.launch.xml` | Everything (the one `./scripts/demo` runs) | `skills_config`, `executive_config`, `use_camera_provider` (`true`), `show_viewer` (`true`), `camera_update_rate` (`30`), `viewer_scale` (`1.0`) |

`display.launch.xml` is good for checking the model itself: a
`joint_state_publisher_gui` window lets you drag joints and watch the robot move in
RViz. `moveit.launch.xml` plans and previews motion without waiting for a physics
simulation to start. `skills.launch.xml` lets you call `Pick`, `Place`, or `Home` by
hand and watch each one happen in isolation.

On `app.launch.xml`, `use_camera_provider:=false` swaps the wrist camera for
`fixed_object_provider`, which reads hardcoded cube positions from
`src/sorting_arm_executive/config/sorting.yaml` and answers the same detection
service, useful for testing pick and place without perception in the loop.
`show_viewer` and `viewer_scale` are passed straight through to
`perception.launch.xml` and only do anything when the camera provider is running.
`gui` is not one of `app.launch.xml`'s arguments - the full app always runs the
Gazebo GUI. Only `sim.launch.xml` can go headless.

```bash
ros2 launch sorting_arm_bringup sim.launch.xml gui:=false
ros2 launch sorting_arm_bringup camera_validation.launch.xml
ros2 launch sorting_arm_bringup app.launch.xml use_camera_provider:=false
ros2 launch sorting_arm_bringup app.launch.xml show_viewer:=false
```

## If the simulation is lagging

**If you have a GPU and it isn't being used, that's the real fix.** Check
[Running with or without a GPU](#running-with-or-without-a-gpu) before touching any
of the settings below - software rendering is the slowest path by a wide margin, and
no launch argument closes that gap.

Cheapest fix that doesn't touch rendering at all, cutting the OpenCV detection
window:

```bash
ros2 launch sorting_arm_bringup app.launch.xml show_viewer:=false
```

`viewer_scale:=0.5` shrinks that OpenCV window further, but only the window -
detection still runs on the full-resolution frame, so this helps only if the window
redraw itself is what's slow.

```bash
ros2 launch sorting_arm_bringup app.launch.xml viewer_scale:=0.5
```

**Camera resolution is not a free dial, and lowering it will break detection on the
full app.** The wrist camera renders at 1280x960 at 30 Hz by default, and that's the
most expensive part of every frame - but `camera_width` and `camera_height` are not
`app.launch.xml` arguments, and that's deliberate.
`src/sorting_arm_perception/config/perception.yaml`'s `minimum_contour_area` (400 px)
and `minimum_top_face_area` (480 px) are pixel counts calibrated at exactly that
resolution and don't scale down on their own. Halve the resolution and a cube's
contour area drops to roughly a quarter of what it was; the thresholds don't move, so
every cube gets rejected as too small. The detector checks the incoming camera stream
against its tuned resolution and fails `DetectObjects` naming both, instead of
silently finding nothing.

`sim.launch.xml` still takes `camera_width`, `camera_height`, and
`camera_update_rate` directly, since it starts no detector and nothing there depends
on the tuned pixel thresholds:

```bash
ros2 launch sorting_arm_bringup sim.launch.xml \
    camera_width:=640 camera_height:=480 camera_update_rate:=10
```

This is also safe on `app.launch.xml use_camera_provider:=false`, since the fixed-pose
provider never touches the pixel detector either. If you want a lower resolution on
the full app anyway, `tuned_image_width`, `tuned_image_height`,
`minimum_contour_area`, and `minimum_top_face_area` in `perception.yaml` would need to
scale together by the same area ratio, for example a quarter each at 640x480. That
combination hasn't been tested here - this cell is tuned for one camera
configuration, not built to generalise across resolutions, so treat it as a starting
point, not a supported setting.

The full app always runs the Gazebo GUI - `gui` isn't one of its arguments. To drop
the 3D view and keep only physics and RViz, run `sim.launch.xml` directly instead of
the full app:

```bash
ros2 launch sorting_arm_bringup sim.launch.xml gui:=false
```

These are xacro arguments on `sorting_arm.sim.urdf.xacro`, threaded through
`sim.launch.xml`; the description still parses at any resolution you pass, and the
defaults above are unchanged unless you override them.

## Running individual nodes

| Command | What it is |
|---|---|
| `ros2 run sorting_arm_skills skill_server_node` | Hosts the `Pick`, `Place`, and `Home` actions, and the `SyncObjects` service |
| `ros2 run sorting_arm_skills motion_demo` | Runs one motion; `operation` is `named`, `joint`, `pose`, `cartesian`, or `apply_scene` |
| `ros2 run sorting_arm_skills sequence_demo` | Runs a scripted pick-and-place sequence |
| `ros2 run sorting_arm_executive executive_node` | Runs the BehaviorTree |
| `ros2 run sorting_arm_executive fixed_object_provider` | Answers `DetectObjects` from fixed poses |
| `ros2 run sorting_arm_perception camera_object_provider` | Answers `DetectObjects` from the wrist camera |

These normally come up through the launch files above, but you can also talk to a
running skill server or detector directly:

```bash
ros2 service call /detect_objects sorting_arm_interfaces/srv/DetectObjects "{expected_count: 4}"
ros2 service call /sync_objects sorting_arm_interfaces/srv/SyncObjects "{objects: []}"
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
| `docker: permission denied` | The scripts already fall back to `sudo docker` for you when this happens; fix it for good with `sudo usermod -aG docker $USER`, then log out and back in |
| A controller stays inactive | Inside the container, `ros2 control list_controllers`: `joint_state_broadcaster`, `arm_controller`, and `gripper_controller` should all read `active` |
| Detection finds nothing | Launch `perception.launch.xml show_viewer:=true` and watch `/perception/debug_image`; the colour bands and search volume are in `src/sorting_arm_perception/config/perception.yaml` |
| The URDF won't parse | `xacro src/sorting_arm_description/urdf/sorting_arm.urdf.xacro \| check_urdf -` |
| Nothing has been built yet | `rosdep install --from-paths src --ignore-src -r -y`, then `colcon build --symlink-install`, inside the container |
| A script refuses to use your GPU | `--gpu` requests the NVIDIA device reservation explicitly; if the hardware or runtime isn't there, compose fails to create the container rather than quietly falling back. Drop `--gpu` to auto-detect instead, or pass `--no-gpu` |
| Gazebo is unbearably slow | See [If the simulation is lagging](#if-the-simulation-is-lagging) |
| The container is still running after you closed the demo | That's by design - `Ctrl-C` only stops the launch, so `enter` and `demo` are instant next time. `./scripts/stop` tears the container down |

## Documentation

| Doc | Covers |
|---|---|
| [docs/architecture.md](docs/architecture.md) | Package boundaries, the cycle, frames, interfaces |
| [.docker/README.md](.docker/README.md) | What's in `.docker/`, and why there are two images |
| [docs/container/](docs/container/) | Tutorial, day-to-day commands, devcontainer setup, Dockerfile and compose reference |
| [docs/ci.md](docs/ci.md) | Why CI exists, what each stage proves, running the same checks locally |
| [docs/rca/](docs/rca/) | Write-ups of failures that took real investigation to track down |

Deeper implementation notes for `sorting_arm_skills`, `sorting_arm_perception`, and
`sorting_arm_executive` exist for project development but aren't part of this public
set yet.

## Developing in VS Code

Everything above works from a terminal with no editor involved. If you want an IDE
attached to the same container, install the [Dev Containers
extension](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers),
open this repository, and run **Dev Containers: Reopen in Container**.

> [!IMPORTANT]
> Let VS Code create the container - don't run `./scripts/start` first and then
> **Attach to Running Container**. Attach connects to whatever is already there
> without reading `devcontainer.json` at all: no extensions install, no settings
> apply, and you land as the wrong user in the wrong directory. It doesn't error, it
> just quietly isn't the environment `devcontainer.json` describes.

Once VS Code has created the container this way, `./scripts/enter` or a plain
`docker exec -it sorting_arm ...` from any other terminal reaches that same
container. Only the reverse order, starting it yourself first, causes the problem
above.

`devcontainer.json` can't probe the host for a GPU the way the scripts do, so it
doesn't request one by default. Add the fragment yourself if you want one:

```jsonc
"dockerComposeFile": [
    "../docker-compose.yaml",
    "../.docker/compose.dev.yaml",
    "../.docker/compose.gpu.yaml"      // or compose.dri.yaml for integrated graphics
]
```

Closing the VS Code window leaves the container running
(`"shutdownAction": "none"` in `devcontainer.json`); `./scripts/stop` is what
actually stops it.

## Credits

The UR5e description and the Robotiq 2F-85 gripper description were copied into
`sorting_arm_description` and renamed to this project's namespace, not depended on as
external packages. Both keep their original licence.

- **UR5e:** from ROS-Industrial's `Universal_Robots_ROS2_Description` project,
  `ur_description`, BSD-3-Clause.
- **Robotiq 2F-85:** from `robotiq_description`, BSD-3-Clause.

See [NOTICE](NOTICE) for the exact file list, the preserved upstream author credit in
`ur_macro.xacro`, and the full licence text. The project is built directly on
MoveIt 2, `ros2_control`, Gazebo, and BehaviorTree.CPP rather than reimplementing any
of them.

## License

This project's own code is MIT. See [LICENSE](LICENSE). The vendored UR5e and
Robotiq 2F-85 files keep their original BSD-3-Clause license. See [NOTICE](NOTICE).
