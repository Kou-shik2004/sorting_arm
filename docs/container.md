# Container

One image, built from `.docker/Dockerfile`: ROS 2 Jazzy desktop, the workspace
toolchain, and nothing else. `docker-compose.yaml` at the repo root runs it with your
repo bind-mounted at `/sorting_arm_ws` and the host's X11 socket forwarded, so RViz
and Gazebo windows show up on your desktop.

The image is built from `osrf/ros:jazzy-desktop-full`, with the workspace's own
toolchain layered on from `.docker/packages/{base,ros,extra}.txt`, plus
BehaviorTree.ROS2, which isn't packaged for apt and gets built from source into
`/opt/underlay` from `.docker/underlay.repos`. The workspace itself is not baked into
the image; it's bind-mounted from the host and built with `cbuild` once you're inside.

The base compose file ships no named volumes. Enter the container, build, source,
launch, that's the whole offering. Two optional fragments add more:

| File | Adds | Enabled by |
|---|---|---|
| `.docker/compose.gpu.yaml` | NVIDIA GPU passthrough | `scripts/start` and `scripts/demo`, automatically, when they find the NVIDIA container runtime. Force it with `--gpu` or `--no-gpu`. |
| `.docker/compose.dev.yaml` | Named volumes for VS Code Server and bash history, so they survive a container rebuild | `--dev` on any script |

Reserving a GPU device you don't have is a hard failure, not a slow fallback, which
is why it's opt-in rather than in the base file.

## Scripts

```bash
./scripts/build   # build the image
./scripts/start   # bring the container up, detached, idle
./scripts/enter   # shell into it, starting it first if it's down
./scripts/demo    # build the workspace if needed, then run the full sort demo
```

All four live under `scripts/`, not the repo root, so `build` doesn't collide with
colcon's own `build/` directory. Each checks for Docker before doing anything and
prints the install link if it's missing, no silent installs, no sudo you didn't ask
for. `enter` and `demo` reuse `.docker/entrypoint.sh` to source the workspace, so a
one-off `docker exec` command gets ROS on its `PATH` the same way an interactive
shell does.

## What gets sourced, and when

`.docker/entrypoint.sh` sources three things in order, each only if it's there:
`/opt/ros/jazzy/setup.bash`, then `/opt/underlay/install/setup.bash`, then
`/sorting_arm_ws/install/setup.bash` if the workspace has been built. Every command
that runs through the entrypoint, an interactive shell from `enter` or a one-shot
`docker exec` from `demo`, gets the same environment as a result.

## The three shell functions

`.docker/bashrc.sh` defines the functions you'll use inside the container:

```bash
cbuild      # colcon build --symlink-install
ctest_all   # colcon test, then colcon test-result --verbose
cdeps       # rosdep install --from-paths src --ignore-src -r -y
```

You only need a full image rebuild when `.docker/packages/*.txt`, `underlay.repos`, or
the Dockerfile itself changes. Changes to workspace source under `src/` just need
`cbuild`.

## A second, smaller image for CI

`.docker/runtime/` builds a separate, smaller image that `.github/workflows/ci.yml`
uses to check the workspace headlessly. It isn't what you develop in; the compose
files at the repo root and the four scripts above are for that.

## Adding your own persistent state

Want an editor's server, a tool's config, a shell history that isn't the one
`compose.dev.yaml` already gives you? Add it to `docker-compose.yaml` yourself, either
as a bind mount:

```yaml
volumes:
  - ~/.some-tool:/home/rosdev/.some-tool:rw
```

or as a named volume, declared under `volumes:` at the bottom of the file the same way
`compose.dev.yaml` does it. Nothing here assumes what tools you want; the base image
only carries what the workspace itself needs to build and run.

## The devcontainer

`.devcontainer/devcontainer.json` points VS Code at the same `docker-compose.yaml` and
carries a small starting set of extensions for this repo's C++, Python, launch XML,
and URDF. Add your own on top; nothing about it is meant to be final.
