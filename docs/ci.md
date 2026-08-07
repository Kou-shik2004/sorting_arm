# Continuous integration

## Why it exists

The development image runs `ros:jazzy-desktop-full` with this workspace bind-mounted
in and built up over weeks of edits. A green build there proves the build works on
that machine, with whatever apt packages and rosdep state happened to accumulate
along the way. It doesn't prove a stranger's clone builds.

`.github/workflows/ci.yml` builds `.docker/runtime/Dockerfile` instead: a clean Ubuntu
24.04 runner, no bind mount, no history. The only inputs are the source tree and the
`package.xml` manifests. If that build succeeds, the manifests are actually complete,
not just complete enough for a machine that already has everything installed.

## What each stage ensures

| Stage | Target | Proves |
|---|---|---|
| Build and test | `test` (`ci.yml:32`) | A clean `colcon build` from manifest-declared dependencies alone, then `colcon test --return-code-on-test-failure` |
| Runtime dependency check | `runtime-deps-check` (`ci.yml:47`) | `rosdep check` for exec dependencies against the headless package set: nothing the runtime image needs is undeclared |
| Runtime image build | `runtime-headless` (`ci.yml:59`) | The image runs as non-root, carries the current commit's test-pass marker, and contains no compiler, `colcon`, `rosdep`, `sudo`, or GUI package |
| Headless smoke test | `smoke` (`ci.yml:83`) | `sim.launch.xml gui:=false` actually starts: clock, joint states, all three controllers active, MoveIt's `move_action` server up, bounded at 180 seconds |

**Build and test.** `.docker/runtime/Dockerfile`'s `test` stage runs
`colcon build --cmake-args -DBUILD_TESTING=ON`, then `colcon test` with
`--return-code-on-test-failure` so a failing test fails the job instead of quietly
passing with a non-zero exit code buried in the log. Nothing here reads from the
bind-mounted workspace; every dependency comes from `apt-get install` against
`package.xml`.

**Runtime dependency check.** A package can build fine and still be missing an exec
dependency it only needs at runtime, a message package, a Python library, a launch
plugin. `runtime-deps-check` runs `rosdep check --dependency-types exec` against the
same manifests, so a gap here fails before the runtime image is even built.

**Runtime image build.** `runtime-headless` assembles the actual image CI validates:
copies the built `install/` tree in from the test stage, strips the build toolchain,
and runs as the non-root `ubuntu` user.
[`validate-image.sh`](../.docker/runtime/validate-image.sh) then inspects the built
image directly, checking its user, entrypoint, and default command, and running a
throwaway container to confirm no compiler, `colcon`, `rosdep`, `sudo`, or GUI package
survived into it. If one of those creeps back in, this stage catches it before the
smoke test ever runs.

**Headless smoke test.** The other three stages check what's *in* the image.
[`test_headless.py`](../.docker/runtime/smoke/test_headless.py) checks what it *does*:
it launches `sim.launch.xml gui:=false` for real and waits on concrete readiness
conditions, a `/clock` tick, a `JointState` message, all three controllers reporting
`active`, and MoveIt's `move_action` server answering, rather than sleeping a fixed
number of seconds and hoping. The whole run is bounded at 180 seconds by `timeout`, so
a hung launch fails the job instead of hanging the runner.

## The manifest-omission trap

`.docker/runtime/Dockerfile` lists every package's `package.xml` by hand in a `COPY`
instruction, once for the build stage and once for the runtime stage. Immediately
after copying `src/` in, it runs:

```dockerfile
RUN find src -mindepth 2 -maxdepth 2 -name package.xml -printf '%h\n' | xargs -n1 basename | sort > /tmp/src-packages.txt \
 && find /tmp/sorting-arm-src -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort > /tmp/manifest-packages.txt \
 && diff /tmp/src-packages.txt /tmp/manifest-packages.txt
```

A package that exists under `src/` but was never added to the hand-written `COPY`
list fails this `diff` immediately, instead of silently building without its
dependencies resolved and failing somewhere much less obvious later. This project has
been bitten by exactly that gap before, twice, see
[`docs/rca/ci-package-manifest-omission.md`](./rca/ci-package-manifest-omission.md).

> [!IMPORTANT]
> A new package needs its `package.xml` added to **both** `COPY` blocks in
> `.docker/runtime/Dockerfile`, in the same pull request that adds the package. See
> `.claude/rules/docker-ci.md` for the same rule stated for the development image.

## Running the same checks locally

The build and test stage is the one worth reproducing before pushing:

```bash
colcon build --symlink-install
colcon test
colcon test-result --verbose
```

`rosdep check --from-paths src --ignore-src --dependency-types exec` reproduces the
runtime dependency check, run against your own package set instead of the headless
skip-key list.

## Reading a failure

Each job step in the GitHub Actions log names its stage, so the first red step tells
you which contract broke: manifest completeness, a failing test, an undeclared
runtime dependency, a build tool that leaked into the runtime image, or the sim
failing to actually come up. `workflow_dispatch` accepts a `clean_image` input that
disables the GitHub Actions layer cache for that run, useful when a failure might be a
stale cache rather than a real regression.
