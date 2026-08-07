# `.docker`

Everything the project needs to build and run a container lives here: the
development image you work in, and a separate, smaller image that only CI builds.
Nothing in this folder is meant to be read top to bottom. Use the table below to find
the one file you actually need, then follow the link at the bottom for the reasoning
behind it.

## File map

| File | What it does |
|---|---|
| [`Dockerfile`](./Dockerfile) | Builds the development image: `osrf/ros:jazzy-desktop-full` plus this workspace's toolchain. What `./scripts/build` builds. |
| [`bashrc.sh`](./bashrc.sh) | Interactive-shell setup sourced from inside the container: ROS environment, command history, prompt. No build or test aliases. |
| [`entrypoint.sh`](./entrypoint.sh) | Sources the ROS and workspace overlays before handing off to whatever command the container was started or `exec`'d with. |
| [`underlay.repos`](./underlay.repos) | The one source dependency not packaged for apt, BehaviorTree.ROS2, built from source into `/opt/underlay` at image build time. |
| [`packages/base.txt`](./packages/base.txt), [`ros.txt`](./packages/ros.txt), [`extra.txt`](./packages/extra.txt) | The apt package lists the Dockerfile installs, split so a new package in `extra.txt` doesn't invalidate the expensive layers above it. |
| [`colcon/defaults.yaml`](./colcon/defaults.yaml) | colcon build defaults: pins the Python interpreter `cmake` probes for and turns on `compile_commands.json` for clang-tidy. |
| [`compose.gpu.yaml`](./compose.gpu.yaml) | Opt-in NVIDIA GPU passthrough, added automatically by the scripts when the NVIDIA container runtime is present. |
| [`compose.dri.yaml`](./compose.dri.yaml) | Opt-in Intel/AMD integrated graphics passthrough, added automatically when `/dev/dri` exists and no NVIDIA runtime was found. |
| [`compose.dev.yaml`](./compose.dev.yaml) | Opt-in named volumes for VS Code Server and bash history, so both survive a container rebuild. |

`docker-compose.yaml` itself lives at the repo root, not here, because it's the entry
point Docker Compose looks for by convention.

## Two images, two jobs

**The image built from `.docker/Dockerfile`** is what you develop in: a
bind-mounted workspace, the full toolchain, GUI forwarding for RViz and Gazebo. This
is the one `./scripts/build`, `start`, `enter`, and `demo` all use, described in the
root [README](../README.md).

**The image built from `.docker/runtime/Dockerfile`** is smaller, headless, and only
`.github/workflows/ci.yml` builds it. It compiles the workspace from a clean Ubuntu
runner, strips every build tool back out, and proves the result runs as a non-root
user with none of the trimmed dependencies missing. You never build this one by hand;
see [`docs/ci.md`](../docs/ci.md) for what each of its stages checks and why.

Nobody using the project needs to touch `.docker/runtime/`. It exists so CI can prove
the dev image's shortcuts, the bind mount, the pre-baked toolchain, aren't hiding a
dependency the manifests don't declare.

## `.docker/runtime/` file map

| File | What it does |
|---|---|
| [`runtime/Dockerfile`](./runtime/Dockerfile) | Multi-stage build: compiles and tests the workspace clean, then assembles a headless runtime image with the toolchain removed. |
| [`runtime/entrypoint.sh`](./runtime/entrypoint.sh) | The runtime image's equivalent of the dev `entrypoint.sh`, minus the underlay source that only the dev image builds. |
| [`runtime/resolve-dependencies.sh`](./runtime/resolve-dependencies.sh) | Turns each package's `exec` rosdep keys into a concrete apt package list, so the runtime stage installs only what's actually needed to run, not to build. |
| [`runtime/validate-image.sh`](./runtime/validate-image.sh) | Asserts the built image: non-root user, no compiler or `colcon` left behind, no GUI package, the right entrypoint and default command. CI runs this after the image builds. |
| [`runtime/packages/gui-skip-keys.txt`](./runtime/packages/gui-skip-keys.txt) | The rosdep keys the headless image is allowed to skip: RViz, MoveIt visualization, the setup assistant. |
| [`runtime/smoke/test_headless.py`](./runtime/smoke/test_headless.py) | A `launch_test` that brings the headless sim up for real and checks the clock, joint states, controllers, and MoveIt action server, bounded at 180 seconds. |

## When a rebuild is actually needed

Source under `src/` never needs a Docker rebuild, only `colcon build`. A rebuild of
the development image is for changes to `.docker/packages/*.txt`,
`.docker/underlay.repos`, or the Dockerfile itself. The full table, with the reasoning
behind each row, is in
[`docs/container/daily-use.md`](../docs/container/daily-use.md#when-a-rebuild-is-actually-needed).

## Where to go next

- [`docs/container/`](../docs/container/): the tutorial, the day-to-day commands, the
  Dockerfile explained layer by layer, and the devcontainer setup.
- [`docs/ci.md`](../docs/ci.md): why `.docker/runtime/` exists and what each CI stage
  proves.
