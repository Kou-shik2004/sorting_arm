# Container setup

One image, built from `.docker/Dockerfile`: ROS 2 Jazzy desktop, this workspace's own
toolchain, and nothing else. `docker-compose.yaml` at the repo root runs it with the
repository bind-mounted at `/sorting_arm_ws` and the host's X11 socket forwarded, so
RViz and Gazebo windows show up on your desktop. The host provides Docker, an X
server, and (optionally) a GPU driver - nothing else. There is no ROS installation on
the host.

The image is built from `osrf/ros:jazzy-desktop-full`, with this workspace's own
toolchain layered on from `.docker/packages/{base,ros,extra}.txt`, plus
BehaviorTree.ROS2, which isn't packaged for apt and gets built from source into
`/opt/underlay` from `.docker/underlay.repos`. The workspace itself isn't baked into
the image; it's bind-mounted from the host and built with `cbuild` once you're inside.

The base compose file ships no named volumes: enter the container, build, source,
launch, that's the whole offering. Two optional fragments add more automatically, and
a third is available by hand:

| File | Adds | Enabled by |
|---|---|---|
| `.docker/compose.gpu.yaml` | NVIDIA GPU passthrough | `scripts/start`, `enter`, and `demo`, automatically, when they find the NVIDIA container runtime. Force it with `--gpu` or `--no-gpu` |
| `.docker/compose.dri.yaml` | Intel/AMD integrated graphics passthrough | The same scripts, automatically, when `/dev/dri` exists and no NVIDIA runtime was found |
| `.docker/compose.dev.yaml` | Named volumes for VS Code Server and bash history, so they survive a container rebuild | `--dev` on any script, and by default in the devcontainer |

Reserving a GPU device you don't have is a hard failure, not a slow fallback, which is
why both GPU fragments are opt-in rather than in the base file. With neither one, the
container still renders - through Mesa's `llvmpipe` on the CPU - just slower; see the
README's [Running with or without a GPU](../../README.md#running-with-or-without-a-gpu).

## Where to start

| Document | What it covers |
|---|---|
| [tutorial.md](./tutorial.md) | **Start here.** Clone to a sorted cube, one path, no options |
| [daily-use.md](./daily-use.md) | The scripts day to day, when a rebuild is actually needed, troubleshooting |
| [devcontainer.md](./devcontainer.md) | Attaching VS Code to the same container, and the one order-dependent trap |
| [dockerfile.md](./dockerfile.md) | `.docker/Dockerfile`, layer by layer: what each block does and why it sits where it sits |
| [compose.md](./compose.md) | `docker-compose.yaml` and its three opt-in fragments, `.dockerignore`, `entrypoint.sh`, `bashrc.sh`, colcon defaults |
| [changes.md](./changes.md) | The 25 July 2026 review of `.docker/`: what changed and why |
| [container-lessons.md](./container-lessons.md) | One pattern per debugged mistake, container and toolchain |

## The scripts

```bash
./scripts/build   # build the image
./scripts/start   # bring the container up, detached, idle
./scripts/enter   # shell into it, starting it first if it's down
./scripts/demo    # build the workspace if needed, then run the full sort demo
./scripts/stop    # tear the container down
```

Each checks for Docker before doing anything and prints the install link if it's
missing - no silent installs, no sudo you didn't ask for. `enter` and `demo` reuse
`.docker/entrypoint.sh` to source the workspace, so a one-off `docker exec` command
gets ROS on its `PATH` the same way an interactive shell does. See the README's [The
scripts](../../README.md#the-scripts) for the raw `docker compose` / `docker exec`
command behind each one.

## What gets sourced, and when

`.docker/entrypoint.sh` sources three things in order, each only if it's there:
`/opt/ros/jazzy/setup.bash`, then `/opt/underlay/install/setup.bash`, then
`/sorting_arm_ws/install/setup.bash` if the workspace has been built. Every command
that runs through the entrypoint - an interactive shell from `enter`, or a one-shot
`docker exec` from `demo` - gets the same environment as a result. Details, and the
`docker exec` gap this doesn't cover, are in [compose.md](./compose.md#dockerentrypointsh).

## The three shell functions

`.docker/bashrc.sh` defines the functions used inside the container:

```bash
cbuild      # colcon build --symlink-install
ctest_all   # colcon test, then colcon test-result --verbose
cdeps       # rosdep install --from-paths src --ignore-src -r -y
```

A full image rebuild is only needed when `.docker/packages/*.txt`,
`underlay.repos`, or the Dockerfile itself changes. Changes to workspace source under
`src/` just need `cbuild` - see [daily-use.md](./daily-use.md#when-a-rebuild-is-actually-needed)
for the complete table.

## A second, smaller image for CI

`.docker/runtime/` builds a separate, smaller image that `.github/workflows/ci.yml`
uses to check the workspace headlessly. It isn't what you develop in; the compose
files at the repo root and the scripts above are for that.

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
