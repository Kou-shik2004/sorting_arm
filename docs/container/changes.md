# Container review, 25 July 2026 - what changed and why

A line-by-line review of `.docker/` raised a set of design questions. This file
records what came out of it, so the answers don't have to be re-derived. The
layer-by-layer explanation stays in [dockerfile.md](./dockerfile.md); this is the
change log with the reasoning attached.

> [!NOTE]
> This review predates the split between the public `.docker/Dockerfile` and the
> maintainer-only `.private/Dockerfile` (7 Aug 2026). The decisions below shaped both;
> anything specific to the private image's agent tooling has been left out.

> [!NOTE]
> `cbuild`, `ctest_all`, and `cdeps`, mentioned below, were removed from
> `.docker/bashrc.sh` for the public release (8 Aug 2026). The reasoning in this file
> about *why* runtime `rosdep` resolution beats a build-time layer still holds; only
> the shell function wrapping it is gone. Public docs now give the raw `colcon` /
> `rosdep` commands.

## At a glance

| Change | File | Why |
|---|---|---|
| apt lists moved into three files | `.docker/packages/{base,ros,extra}.txt` | The Dockerfile now reads as build process, not as package inventory |
| Underlay renamed and driven by a repos file | `.docker/underlay.repos`, `/opt/underlay` | One `source` line covers every future source dependency, declared in one readable file |
| `RMW_IMPLEMENTATION` became a Dockerfile `ENV` | `Dockerfile`, `entrypoint.sh`, `bashrc.sh` | It was exported in two places; an `ENV` reaches every process instead |
| Baked `bashrc.sh` copy and its fallback deleted | `Dockerfile`, `.dockerignore` | Two copies of one file inside the container, the baked one silently stale |
| Commented-out rosdep block replaced by `cdeps` | `Dockerfile`, `bashrc.sh` | Runtime resolution off the bind mount can't go stale and busts no layer |
| Parser directive on line 1 fixed | `Dockerfile` | A trailing comment made the directive invalid, so BuildKit ignored it |
| Long comments cut | compose, `devcontainer.json`, `.dockerignore`, `Dockerfile` | Every rationale in them is already in these docs |

## Why the apt lists are files now

Three lists under `.docker/packages/`, one `COPY` each, each immediately above the
`RUN` that reads it. Mechanics are in
[dockerfile.md](./dockerfile.md#where-the-package-lists-live); the parts worth
arguing about:

- **The separate `COPY`s are the whole point.** A single `COPY .docker/packages/`
  would put all three lists in one layer, so adding a package to `extra.txt` would
  invalidate the base install. That's the cache chain this split is arranged to
  avoid.
- **`extra.txt` exists because `base.txt` is expensive.** An edit to `base.txt`
  re-runs the ROS install and the underlay build. `extra.txt` is installed by the
  last layer and stays the cheap slot for daily additions.
- **No `xargs`.** `xargs -a list apt-get install` on an empty or mistyped list runs
  `apt-get install` with no arguments, succeeds, and installs nothing - the failure
  then surfaces later as a missing command. The `pkgs="$(...)" && [ -n "$pkgs" ]` form
  fails the build on the spot.

## Why `/opt/underlay` and a `.repos` file

The old form was `git clone -b humble --depth 1` into a package-named path.

**A per-package workspace doesn't scale to two.** Naming the underlay after its first
occupant implies a second path for the next source dependency, and then both
`entrypoint.sh` and `bashrc.sh` need another `source` line. One merged underlay at
`/opt/underlay` with `--merge-install` means those two files never change again -
which is the real win, more than the name.

`vcstool` is already installed and is the ROS-native tool for this. `humble` on Jazzy
isn't a workaround: `git ls-remote` shows it's the only release branch
BehaviorTree.ROS2 publishes, and upstream supports "Humble or newer" from it.

**The version stays a branch, not a commit.** A SHA would make the build
reproducible, and that's a real property to give up; the cost is a line nobody can
read and a manual hash edit for every upgrade.

## Why `RMW_IMPLEMENTATION` moved to an `ENV`

It was `export`ed in both `entrypoint.sh` and `bashrc.sh`. The duplication wasn't
sloppiness, it was necessary, and the reason is the useful part:

**`docker exec` doesn't run the `ENTRYPOINT`.** It runs a new process in an existing
container and never passes through it. So a value set only in the entrypoint never
reaches a VS Code terminal, and a value set only in `bashrc.sh` never reaches a
non-interactive process. One `ENV` covers PID 1, `docker exec`, editor tasks, and hook
processes at once.

Related finding from testing this: `bash -lc` doesn't give a `docker exec` the ROS
environment either. A login shell is still non-interactive, so `bashrc.sh` returns at
its `case $-` guard. Use `docker exec sorting_arm /entrypoint.sh <command>`.

## Why the bashrc fallback is gone

The Dockerfile used to also `COPY` `bashrc.sh` to a baked path, with a loop preferring
the bind mount and falling back to the baked copy. The fallback only mattered for a
container started without the workspace mounted, which doesn't happen through any
supported path here - the scripts and the devcontainer both always mount it. It also
cost a real trap: two copies of the same file inside the container, the baked one
silently stale.

The `[ -f ]` guard on the sourcing line stays, so a shell that somehow starts without
the mount still opens instead of erroring.

## Why there is both an apt list and rosdep

The sharpest question of the review, and the answer is that they have different jobs.

- `ros.txt` pre-bakes MoveIt, the Gazebo bridge, and the rest of what every phase of
  this project needs, so a clean image is ready to build against without a first-run
  dependency fetch.
- `package.xml` manifests are the source of truth for what each package actually
  depends on, read by `cdeps` at runtime.
- A build-time `rosdep install` layer would re-run on every manifest edit, and the
  bind mount always has `src/` anyway, so runtime resolution costs nothing and can't
  drift against a manifest baked in weeks earlier.

## Comments: the rule that came out of this

The generated comments this review flagged were long because they explained -
explanation that was already in these documents. The rule has a volume cap as well as
a style, recorded in `.claude/rules/comments.md`: two lines, and anything longer goes
in `docs/` with a one-line pointer from the code. Never the same rationale in both
places - one of the two rots, and it's always the comment.
