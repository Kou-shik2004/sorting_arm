# `docker-compose.yaml` and the opt-in fragments

The Dockerfile describes what the image contains. This file, plus two optional
fragments, describes how the container runs.

## The base file

```yaml
services:
  dev:
    build:
      context: .
      dockerfile: .docker/Dockerfile
      args:
        USERNAME: rosdev
        USER_UID: 1000
        USER_GID: 1000
    image: sorting_arm:jazzy
    container_name: sorting_arm
    command: sleep infinity
    network_mode: host
    ipc: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      - .:/sorting_arm_ws:rw
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
```

The service is named `dev` in every compose file in this repository, including the
private one under `.private/` - only the image name and container name change.

**`command: sleep infinity`** keeps the container alive with nothing running in it.
It's a machine you enter, not a process you launch, so it has to outlive any
particular command.

**`network_mode: host`** removes network namespacing. DDS discovery works without
multicast configuration, and host tools can see container topics. The trade: container
ports are host ports, so anything listening inside can collide with host software, and
there's no port remapping under host networking.

**`ipc: host`** shares the host IPC namespace. Qt-based tools (RViz, the Gazebo GUI)
use MIT-SHM shared memory, which doesn't work cleanly across the default IPC
isolation. `QT_X11_NO_MITSHM=1` disables that transport as well - it's set in
addition to `ipc: host`, not instead of it; either alone has proven insufficient.

**Display.** A container has no path to the host display unless one is given: the X
socket is bind-mounted and `DISPLAY` is passed through. `xhost +local:` may still be
required on the host - the scripts run it for you (see `_lib.sh`'s `allow_x11`); a
raw `docker compose up` doesn't.

**No named volumes in the base file.** Enter the container, build, source, launch -
that's the whole offering. The `--dev` fragment below adds volumes for anyone who
wants a container that survives a rebuild.

## `.docker/compose.gpu.yaml` - NVIDIA

```yaml
services:
  dev:
    environment:
      - NVIDIA_DRIVER_CAPABILITIES=all
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: all
              capabilities: [gpu]
```

The device reservation plus `NVIDIA_DRIVER_CAPABILITIES=all` are what make hardware
rendering available, and they require the NVIDIA Container Toolkit on the host.
Reserving a device that isn't there is a hard failure at container creation, not a
silent fallback - that's exactly why this lives in an opt-in file instead of the base
one. `scripts/start`, `enter`, and `demo` add it automatically when
`docker info --format '{{json .Runtimes}}'` reports an `nvidia` runtime; `--gpu` forces
it on regardless, `--no-gpu` forces it off.

## `.docker/compose.dri.yaml` - Intel/AMD integrated graphics

```yaml
services:
  dev:
    devices:
      - /dev/dri:/dev/dri
```

The middle option, for a host with no NVIDIA runtime but a real GPU behind
`/dev/dri`. The scripts add this fragment automatically when `/dev/dri` exists and no
NVIDIA runtime was found. Slower than a discrete GPU reservation, considerably faster
than the alternative below.

## No GPU at all

With neither fragment applied - `--no-gpu`, or simply no GPU node on the host -
Gazebo still runs. `sorting_cell.sdf` pins Gazebo's sensors system to `ogre2`, so the
camera plugin needs a real GL context whether or not a window is ever shown. Mesa's
`llvmpipe`, already present in `osrf/ros:jazzy-desktop-full`, provides that context in
software, at the cost of running physics and rendering on the CPU. It works
everywhere, including inside a VM with no GPU passthrough at all, and it's the
slowest of the three paths - see the README's [If the simulation is
lagging](../../README.md#if-the-simulation-is-lagging) for what to turn down.

## `.docker/compose.dev.yaml` - persistent volumes

```yaml
services:
  dev:
    volumes:
      - sorting_arm_bash_history:/commandhistory
      - sorting_arm_vscode_server:/home/rosdev/.vscode-server

volumes:
  sorting_arm_bash_history:
    name: sorting_arm_bash_history
  sorting_arm_vscode_server:
    name: sorting_arm_vscode_server
```

Added by `--dev` on any script, and by default in `.devcontainer/devcontainer.json`.
Without it, both directories live in the container's writable layer, which is
discarded on `down`, on `--force-recreate`, and on every image rebuild - so shell
history disappears and VS Code Server re-downloads itself every time. With it, both
survive.

**Never mount a volume over a home directory.** It hides everything the image placed
there. Shell history is persisted by putting it on its own volume at
`/commandhistory` and pointing `HISTFILE` there (see `.docker/bashrc.sh`), rather than
mounting over `~`.

## `-f` order, and why it's fixed

`scripts/_lib.sh`'s `compose_args()` always builds the flag list in the same order:
base file, then GPU or DRI (never both - the GPU runtime wins if both are somehow
present), then dev:

```bash
docker compose -f docker-compose.yaml \
               [-f .docker/compose.gpu.yaml | -f .docker/compose.dri.yaml] \
               [-f .docker/compose.dev.yaml] \
               up -d --build
```

Compose merges `-f` files left to right, later files overriding or extending earlier
ones. Reversing the order wouldn't currently break anything (the fragments touch
disjoint keys), but keeping one fixed order means every script and every doc quotes
the same command.

## `.dockerignore`

```
# Allowlist - the build context holds only what the Dockerfile COPYs.

*
!.docker/entrypoint.sh
!.docker/packages/base.txt
!.docker/packages/ros.txt
!.docker/packages/extra.txt
!.docker/underlay.repos
```

An allowlist: exclude everything, then re-admit exactly what a `COPY` needs. The
build context stays small and, more importantly, cache-stable - nothing outside the
allowlist can invalidate a layer. Adding a `COPY` and adding its `!` line here is one
edit; miss the second half and the build fails with
`failed to compute cache key: "<path>": not found`, which reads like a typo and
isn't - the file simply never reached the daemon.

`bashrc.sh` isn't in the list. That's not an omission - it's sourced straight off the
bind mount instead of `COPY`'d in, so it never needs to reach the build context at
all.

## `.docker/entrypoint.sh`

```bash
source /opt/ros/jazzy/setup.bash
source /opt/underlay/install/setup.bash
[ -f /sorting_arm_ws/install/setup.bash ] && source /sorting_arm_ws/install/setup.bash
exec "$@"
```

This does not run for every container process. `docker exec` bypasses `ENTRYPOINT`
entirely - it starts a new process in an existing container and never passes through
it. What the entrypoint actually wraps is PID 1 (the compose `command: sleep
infinity`) and anything launched with `docker run`.

That's why `bashrc.sh` repeats the same three `source` lines: the two scripts cover
different entry paths, and neither covers both on its own. The workspace overlay is
sourced conditionally because it doesn't exist until the first `cbuild`.

**To run a ROS command through `docker exec` without an interactive shell, call the
entrypoint explicitly:**

```bash
docker exec sorting_arm /entrypoint.sh ros2 pkg prefix behaviortree_ros2
```

A bare `docker exec sorting_arm bash -lc 'ros2 ...'` won't work - a login shell is
still non-interactive, so `bashrc.sh` hits its `case $-` guard and returns before
sourcing anything. `scripts/enter` and `scripts/demo` both go through the entrypoint
for exactly this reason.

## `.docker/bashrc.sh`

Interactive-shell setup only; it returns immediately for anything non-interactive.
Beyond the same three `source` lines as the entrypoint, it defines `cbuild`,
`ctest_all`, and `cdeps` (see the README's [Working inside the
container](../../README.md#working-inside-the-container)), points `COLCON_HOME` at
`.docker/colcon` so colcon config survives a container recreate, and redirects
`HISTFILE` to `/commandhistory` so history survives one too, when the `--dev` volume
is mounted.

## `.docker/colcon/defaults.yaml`

```yaml
build:
  cmake-args:
    - -DPython3_EXECUTABLE=/usr/bin/python3
    - -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
```

Pins the Python CMake uses to configure each package, so a versioned interpreter
installed later on the `PATH` (by you, inside the container) can't get probed ahead
of the system one and fail `ament_cmake` configuration with a missing `catkin_pkg`.
`CMAKE_EXPORT_COMPILE_COMMANDS` emits `compile_commands.json` for `clang-tidy` and for
editor tooling that reads a compilation database.
