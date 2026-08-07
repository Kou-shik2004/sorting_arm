# `.docker/Dockerfile`, layer by layer

Read this next to the file itself. Each section below matches one block, in order.

## The rule the whole file obeys

Docker caches every instruction as a layer, and the cache is a chain: invalidate one
layer and everything below it rebuilds, whether or not it actually changed.

So instructions are ordered by how often they change, never by topic:

```
top     never changes        apt config, user creation
        rarely                system packages, ROS packages, the source underlay
bottom  often                 extra tooling, shell config
```

When two layers change at a similar rate, the expensive one goes higher. A cheap edit
should never re-run a slow download.

## Base image

```dockerfile
FROM osrf/ros:jazzy-desktop-full
```

`desktop-full` brings ROS 2 Jazzy, RViz, the Gazebo bridge tooling, and the standard
build tools. `ros-base` would save a couple of gigabytes and cost an afternoon of
adding GUI packages back one at a time.

## Build arguments

```dockerfile
ARG DEBIAN_FRONTEND=noninteractive
ARG USERNAME=rosdev
ARG USER_UID=1000
ARG USER_GID=1000
```

An `ARG`'s value joins the cache key of every instruction from its declaration
downward. These four sit at the top because they're consumed by the layer immediately
below and don't change. `USERNAME` is overridable at build time
(`--build-arg USERNAME=<you>`), which is how `.private/Dockerfile` reuses this same
file's shape under a different account.

## apt configuration, first layer

```dockerfile
RUN rm -f /etc/apt/apt.conf.d/docker-clean \
 && echo 'Binary::apt::APT::Keep-Downloaded-Packages "true";' \
      > /etc/apt/apt.conf.d/keep-cache
```

Ubuntu-derived images ship `docker-clean`, which deletes every `.deb` the instant
`dpkg` installs it. With BuildKit cache mounts in use, that hook fills the cache and
empties it in the same layer, all cost and no benefit. Removing it has to happen
before any cached apt layer, so it's first.

Every apt layer below mounts the cache:
`--mount=type=cache,target=/var/cache/apt,sharing=locked` (plus `/var/lib/apt`).
`sharing=locked` stops parallel builds from corrupting it. And there's no
`rm -rf /var/lib/apt/lists/*` anywhere in this file: those paths are cache mounts, so
nothing in them reaches an image layer, and the line would only empty the cache.

## User creation

```dockerfile
RUN userdel -r ubuntu 2>/dev/null || true \
 && groupadd --gid $USER_GID $USERNAME \
 && useradd --uid $USER_UID --gid $USER_GID -m -s /bin/bash $USERNAME \
 && echo "$USERNAME ALL=(root) NOPASSWD:ALL" > /etc/sudoers.d/$USERNAME \
 && chmod 0440 /etc/sudoers.d/$USERNAME \
 && mkdir -p /commandhistory \
 && chown $USERNAME:$USERNAME /commandhistory
```

Three traps handled in one layer:

- **Ubuntu 24.04 already ships a user at UID 1000.** Without `userdel -r ubuntu`,
  `useradd` fails with "UID already in use".
- **UID 1000 matches the host user** on most single-user Linux machines, so files
  created inside the container on the bind mount stay owned by you on the host.
- **`/commandhistory` is created and chowned here, as root, before the `USER`
  switch.** Docker seeds a fresh named volume from whatever is at that path in the
  image, ownership included. A directory that's root-owned at that moment produces a
  root-owned volume, and a later `chown` can't fix it, because the mount happens after
  the image is built.

## Where the package lists live

Three lists under `.docker/packages/`, each with its own `COPY` immediately above the
`RUN` that reads it:

| File | Installed at | Cost of adding one package |
|---|---|---|
| `base.txt` | Top, before ROS | Everything below re-runs: ROS install, underlay build |
| `ros.txt` | After `base.txt` | The underlay build re-runs |
| `extra.txt` | Last layer in the file | One trivial layer - this is where a new package goes |

A single `COPY .docker/packages/` would put all three lists in one layer, so editing
`extra.txt` would invalidate the base install too. Separate `COPY`s keep the layers
independent.

```dockerfile
COPY .docker/packages/base.txt /tmp/base.txt
RUN --mount=type=cache,target=/var/cache/apt,sharing=locked \
    --mount=type=cache,target=/var/lib/apt,sharing=locked \
    pkgs="$(grep -vE '^[[:space:]]*(#|$)' /tmp/base.txt)" \
 && [ -n "$pkgs" ] \
 && apt-get update && apt-get install -y --no-install-recommends $pkgs
```

`[ -n "$pkgs" ]` is the guard that makes the split safe: an empty or mistyped list
piped straight into `apt-get install` succeeds while installing nothing, and the
failure would surface later as a missing command instead of at the layer that caused
it. `$pkgs` is deliberately unquoted - word splitting is what turns one string into
many install arguments.

## System and ROS packages

`base.txt` (general tools: `clang-format`, `clang-tidy`, `gdb`, `gh`, `python3-rosdep`,
`python3-vcstool`, `ros-dev-tools`, and similar) and `ros.txt` (MoveIt, `ros2_control`,
the Gazebo bridge, BehaviorTree.CPP, `cv_bridge`, the linters) both install with
`apt-get update && apt-get install` in one `RUN`. Splitting update from install across
layers is a real bug, not a style choice: a cached `update` can feed a stale package
index to a later `install`, which fails with "package not found" for packages that
exist upstream.

## The source underlay, `/opt/underlay`

```dockerfile
COPY .docker/underlay.repos /opt/underlay.repos
RUN mkdir -p /opt/underlay/src \
 && vcs import /opt/underlay/src < /opt/underlay.repos \
 && . /opt/ros/jazzy/setup.sh \
 && colcon build --base-paths /opt/underlay/src \
      --build-base /opt/underlay/build \
      --install-base /opt/underlay/install \
      --merge-install \
      --packages-select btcpp_ros2_interfaces behaviortree_ros2 \
 && rm -rf /opt/underlay/build /opt/underlay/log /opt/underlay/src
```

`behaviortree_cpp` has a Jazzy binary in `ros.txt`. `behaviortree_ros2`, the package
providing `RosActionNode`, doesn't - only source is published, and upstream supports
"Humble or newer" from the `humble` branch. `git ls-remote` on the repository shows
`humble` as the only release branch, so building it from that branch on Jazzy is
upstream's own instruction, not a workaround.

It's built as an underlay in `/opt`, not vendored into `src/`, so that `src/` holds
only authored code, the package resolves exactly like an apt-installed one
(`ros2 pkg prefix behaviortree_ros2` returns `/opt/underlay/install`), and one
`source /opt/underlay/install/setup.bash` line covers it - and any future source
dependency added to the same repos file - forever.

`.docker/underlay.repos` tracks a branch, not a commit SHA. That's a deliberate trade:
a branch can mean two builds a month apart produce different images from
byte-identical Dockerfile text, but a readable `version: humble` costs nothing to
understand, where a SHA would say nothing to the person reading the file. `vcstool`
(`python3-vcstool`, already in `base.txt`) is the ROS-native tool for this.

## User environment

```dockerfile
USER $USERNAME
WORKDIR /sorting_arm_ws

RUN mkdir -p /home/${USERNAME}/.vscode-server
ENV PATH="/home/${USERNAME}/.local/bin:${PATH}"

ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
ENV AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS=1

RUN rosdep update
```

`.vscode-server` is created here, owned by the user, before `.docker/compose.dev.yaml`
can ever mount its named volume over it - a volume created against a root-owned
directory stays root-owned, and a later `chown` can't reach it once the mount exists.

Both `ENV` lines below it are set in the Dockerfile rather than in `bashrc.sh`, and
that's the point: `docker exec` never runs `ENTRYPOINT`, so a value exported only
there never reaches an exec'd shell or a VS Code terminal. An `ENV` reaches PID 1,
every `docker exec`, and every hook process at once. `RMW_IMPLEMENTATION` picks
Cyclone DDS; `AMENT_CPPCHECK_ALLOW_SLOW_VERSIONS` keeps `colcon test` running the same
cppcheck CI runs, rather than silently skipping it because cppcheck 2.13 fails its own
internal speed check.

`rosdep update` seeds the user's rosdep cache, so `rosdep install` has an index to
resolve against without a first-run fetch.

## Extra tooling: where new packages go

```dockerfile
COPY .docker/packages/extra.txt /tmp/extra.txt
RUN --mount=type=cache,... pkgs="$(grep -vE '^[[:space:]]*(#|$)' /tmp/extra.txt)" \
 && [ -n "$pkgs" ] && sudo apt-get update && sudo apt-get install ... $pkgs
```

`.docker/packages/extra.txt` is where a new package goes. Adding one costs seconds,
because nothing expensive sits below this layer. `base.txt` isn't simply "the general
packages" for exactly this reason: it sits at the top, so a package added there
re-runs the ROS install and the underlay build. If `extra.txt` grows long enough to
matter, promote its contents into `base.txt` or `ros.txt` and leave it holding one
package again.

## Workspace dependencies: resolved at runtime, not a build layer

There's no `rosdep install` layer reading `package.xml` in this Dockerfile.
Dependencies declared by packages under `src/` are resolved at runtime instead:

```bash
rosdep install --from-paths src --ignore-src -r -y   # from /sorting_arm_ws
```

A build-time layer would re-run on every manifest edit, and the bind mount always has
`src/` anyway, so a runtime call costs nothing and can never go stale against a
manifest the image baked in weeks ago.

## Shell configuration, last

```dockerfile
RUN echo '[ -f /sorting_arm_ws/.docker/bashrc.sh ] && source /sorting_arm_ws/.docker/bashrc.sh' \
    >> /home/${USERNAME}/.bashrc

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

The most volatile file in the setup sits last, so editing shell config rebuilds one
trivial layer. `bashrc.sh` itself isn't `COPY`'d into the image at all - this line
sources it straight off the bind mount, so a shell tweak needs a new terminal, not a
rebuild. The `[ -f ]` guard means a container that somehow starts without the
workspace mounted still gets a plain shell instead of an error.
