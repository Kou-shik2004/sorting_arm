# Daily use

## What runs where

| Host | Container |
|---|---|
| Docker, the X server, the GPU driver | Everything else |
| VS Code (the window; the server runs inside) | `colcon` build and test, `ros2` CLI, `gz`, RViz |

## Starting work

Either the scripts or plain Docker commands work; see the README's [scripts
table](../../README.md#the-scripts) for the raw `docker compose` line behind each one.

```bash
./scripts/start          # bring the container up, detached, idle
./scripts/enter           # shell into it
```

or, from VS Code, **Dev Containers: Reopen in Container** - see
[devcontainer.md](./devcontainer.md), including the one order-dependent trap that
costs a rebuild if you hit it.

On the host, once per login, if GUI applications fail to open:

```bash
xhost +local:
```

`./scripts/start` and `enter` already run this for you; it only matters for a raw
`docker compose up`.

## The build loop

```bash
colcon build --symlink-install                            # from /sorting_arm_ws
colcon build --symlink-install --packages-select sorting_arm_skills
colcon test --packages-select sorting_arm_skills
colcon test-result --verbose
```

`--symlink-install` means edits to Python, launch files, and configuration take
effect without rebuilding.

## When a rebuild is actually needed

| Change | Action |
|---|---|
| C++ source, launch, config | `colcon build --symlink-install`. Never a Docker rebuild |
| Shell configuration (`.docker/bashrc.sh`) | Open a new terminal. It's read from the bind mount |
| colcon defaults | Nothing - read from the repo at build time |
| A new apt package | Add it to `.docker/packages/extra.txt`, then `./scripts/build`. Seconds |
| A new ROS dependency of a package | Add it to `package.xml`, then `rosdep install --from-paths src --ignore-src -r -y`. Never a Docker rebuild |
| A new source-only dependency | Add an entry to `.docker/underlay.repos` and its package names to `--packages-select` in the Dockerfile, then `./scripts/build`. Long - it re-runs everything below the underlay |

After a rebuild, VS Code needs **Dev Containers: Rebuild Container** to recreate the
container from the new image.

## Debugging: which side of the boundary

- **Always inside:** `gdb`, `ros2 doctor`, `ros2 topic hz` / `echo`, `rqt_graph`,
  planner logs, `gz topic`. If it involves ROS, it happens in the container.
- **Host only when the container itself is the problem:** compose configuration, GPU
  passthrough, X11 authorisation, volume permissions.
- Simulation weirdness: reproduce headless first (`gz sim -s`, or `gui:=false` on any
  launch file) to separate rendering problems from physics or logic.

## Troubleshooting

| Symptom | Likely cause | Fix |
|---|---|---|
| `cannot connect to X server` | No display path | `xhost +local:` on the host; check `echo $DISPLAY` inside |
| RViz or Gazebo crashes on start | Qt shared memory | Confirm `ipc: host` and `QT_X11_NO_MITSHM=1` are both in effect (they are, in the base compose file) |
| Software rendering, or no GPU | No NVIDIA runtime and no `/dev/dri` found | See the README's [Running with or without a GPU](../../README.md#running-with-or-without-a-gpu) |
| A script refuses to use your GPU | `--gpu` requested the NVIDIA reservation explicitly and it isn't there | Drop `--gpu` to auto-detect, or pass `--no-gpu` |
| Attach fails on a `.vscode-server` path | Container was created by a raw `docker compose up`, not by VS Code | `./scripts/stop`, then **Reopen in Container** |
| Extensions missing, wrong user, wrong folder | "Attach to Running Container" was used instead of "Reopen in Container" | See [devcontainer.md](./devcontainer.md) |
| A `devcontainer.json` setting has no effect | Machine-level settings were seeded at container creation | Edit the persisted file, or delete the `sorting_arm_vscode_server` volume and recreate |
| `failed to compute cache key: "<path>": not found` | A `COPY` without a matching `!` line in `.dockerignore` | Add the line |
| `ros2: command not found` under `docker exec` | `docker exec` bypasses the entrypoint, and `bashrc.sh` returns early for any non-interactive shell | `docker exec sorting_arm /entrypoint.sh <command>` |
| A package silently missing after a build | An apt list file was emptied or its path mistyped | The `[ -n "$pkgs" ]` guard should have failed the build - check the `COPY` path matches the `.dockerignore` allowlist |
| Permission denied writing to a mounted directory | Named volume created root-owned | The directory has to be `mkdir -p` and `chown`'d in the Dockerfile before the `USER` switch, then the volume recreated |

## Host-side inspection

| Question | Command |
|---|---|
| What images exist, how big | `docker images` |
| Is the container running, how was it started | `docker ps -a` |
| What volumes exist | `docker volume ls` |
| What mounts and environment does it actually have | `docker inspect sorting_arm` |
| Which layers rebuilt | Watch `CACHED` versus running lines in the build output |
| Reclaim space from dead build cache | `docker system df`, then `docker builder prune` |
