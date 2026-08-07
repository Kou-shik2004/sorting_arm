# RCA: Headless CI failed because runtime assumptions were checked one at a time

## Problem

Building the headless runtime image took four failed GitHub Actions runs and several
host smoke attempts, roughly five to six hours across a single day. Each push
corrected the one error visible in the previous run's log without checking the
complete Docker stage graph, base image, runtime user, and smoke-test ownership
boundary as a whole. Every correction revealed a new, different failure one stage
further in.

## Expected Behavior

`.docker/runtime/Dockerfile` builds a headless runtime image straight through: clean
dependency resolution, a non-root runtime user, a readable smoke test, and a bounded
readiness check that proves the simulation stack actually comes up, not just that the
image built.

## Root Cause

The Dockerfile treats each `FROM` as a genuinely separate machine, but every
correction along the way assumed the opposite: that a later stage inherited whatever
the previous stage had already fixed. It didn't, four times over:

1. **`rosdep` missing.** The runtime dependency stage started from
   `ros:jazzy-ros-core-noble` and ran `rosdep`, assuming the tooling available in the
   earlier `build-deps` stage would carry over. It doesn't; they're separate `FROM`
   instructions with nothing shared between them.
2. **`rosdep` installed but never initialized.** Installing `python3-rosdep` supplies
   the executable, not its generated source configuration. The fix addressed the
   missing binary only, not rosdep's full install-initialize-resolve-cleanup
   lifecycle.
3. **Cleanup tried to remove `sudo`.** Once dependency resolution worked,
   `apt-get autoremove` tried to remove `sudo`, which had arrived as a rosdep
   dependency. `autoremove` reads the package manager's global automatic-package
   state; it isn't a safe inverse of one earlier install command, and resolution and
   final image construction shouldn't have shared one filesystem to begin with.
4. **A duplicate user and group.** Moving rosdep into its own disposable stage
   exposed that `ros:jazzy-ros-core-noble` already ships a `ubuntu` user at UID 1000.
   The development image replaces that user, because a bind-mounted workspace needs
   host UID matching; the runtime image has no such requirement and doesn't need to
   repeat that step.
5. **The smoke test existed but was unreachable.** Copied at mode `0444` into a
   directory Docker also created at `0444`, root could see the file but the
   unprivileged runtime user couldn't traverse the directory to reach it.
   `os.path.isfile()` returned false, and `launch_test` reported the file as
   nonexistent. The original inspection had checked the file as root, which wasn't
   the actual runtime contract.
6. **Readiness got mixed up with upstream shutdown.** After the permission fix, the
   smoke test reached every readiness condition, then failed on assertions about
   clean exit behaviour from MoveIt and Gazebo, neither of which this project owns.
   A broad scan for any `ERROR` string also caught an expected, known limitation (no
   3D sensor plugin for octomap updates) and treated it as a failure.

## Fix

The runtime image now separates responsibility by stage instead of assuming
inheritance across any of them:

- `runtime-dependency-resolver` uses rosdep to derive the runtime apt package set
  from the workspace manifests, and nothing else.
- `runtime-deps` installs that generated package set from a fresh runtime base image.
- `runtime-deps-check` verifies the installed dependency closure and is discarded
  afterward; nothing it leaves behind reaches the final image.
- `runtime-headless` inherits no resolver, rosdep, `sudo`, compiler, source tree,
  build tree, or test tooling.
- The base image's own `ubuntu` account is verified and reused rather than
  recreated.

The smoke target creates `/opt/sorting_arm_smoke` explicitly at mode `0755` and
copies the test in as a root-owned, mode `0444` file, so the runtime user can
traverse and read it but not modify it. The smoke test itself now owns only what this
project actually controls: the simulation clock advances, joint states arrive, all
three controllers go active, the MoveIt action server exists, and the required
processes stay live. It rejects unexpected `ERROR`/`FATAL` lines while permitting the
one specific, expected missing-3D-sensor message, and it makes no claim about clean
upstream teardown.

## Verification

Commit `5e870cc6c166cd7de647d71c6284e92b290ce580` triggered GitHub Actions run
[`30588058008`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30588058008),
which passed every owned step: all four packages built in Release mode,
`colcon test-result --verbose` reported 37 tests with zero errors, zero failures, and
zero skipped, runtime dependencies resolved and passed `rosdep check`,
`runtime-headless` built and passed image inspection, the smoke image built, and
`test_runtime_becomes_ready` passed in 8.906 seconds.

The log still records MoveIt exiting with code `-11` and the nested launcher forcing
termination after readiness passed. Green status means the headless stack became
ready within the bounded test; it does not mean MoveIt and Gazebo shut down cleanly,
and this RCA doesn't claim otherwise.

## Prevention

- Treat every Docker `FROM` as a new machine unless it explicitly names an earlier
  stage; nothing carries over by default.
- Audit every remaining command in the file after the first stopped command, before
  pushing the next correction. A shell chained with `&&` stops at the first failure,
  but the commands after it are still there waiting to fail next.
- Inspect files, directories, users, and writable paths as the actual configured
  runtime user, not as root.
- Keep smoke assertions inside this project's own ownership boundary; record known
  upstream limitations as narrow, exact exceptions instead of a broad error-string
  scan, and remove an exception once the underlying limitation is fixed.

## Lessons Learned

- Fixing the first visible error and re-pushing is slower than reading the whole
  file once the first error is understood. Four runs, each fixing exactly one
  problem, cost far more wall-clock time than reading the remaining stages up front
  would have.
- A multi-stage Dockerfile is not one environment with checkpoints; it's several
  independent machines that happen to share a file. Every assumption about what
  carries over between `FROM` instructions has to be verified, not inferred from how
  the previous stage behaved.
- Docker and Buildx aren't available inside the development container, which made
  GitHub Actions the first environment where any of this could actually be tested.
  That raised the cost of each iteration and is itself a reason to read further ahead
  before pushing, not a reason the bug took this long to find.

## References

- Failed runs: `30566668383`, `30568944356`, `30569741208`, `30571828425`.
- Passing run: [`30588058008`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30588058008).
