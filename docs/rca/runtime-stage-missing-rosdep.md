# Headless CI failed because runtime assumptions were checked one at a time

## Overview

The headless runtime work consumed about five to six hours across four failed
GitHub Actions runs and several host smoke attempts.

The task was not blocked by one difficult ROS problem. It repeated because each
push corrected the first visible error without checking the complete Docker stage
graph, exact base image, runtime user, and smoke-test ownership boundary.

The final implementation passed GitHub Actions on 2026-07-30. Run
[`30588058008`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30588058008)
completed the clean build, 37 tests, dependency verification, runtime construction,
image inspection, smoke-image construction, and bounded readiness smoke test.

This closes the headless readiness incident. It does not prove clean MoveIt and
Gazebo shutdown.

## Impact

The clean workspace build and deterministic tests were healthy. The failures were
in the new runtime-image path.

Four pushes were needed before GitHub could construct and validate the image. Host
smoke attempts then found file-permission and test-boundary problems that the
development container could not reproduce.

Time was spent waiting for remote Docker builds when the complete path should have
been inspected after the first failure.

## Failure timeline

### Run 30566668383: `rosdep` was missing

The runtime dependency stage started from `ros:jazzy-ros-core-noble` and attempted
to run `rosdep`.

```text
/bin/bash: line 1: rosdep: command not found
exit code: 127
```

The incorrect assumption was that tools available in `build-deps` would also exist
in `runtime-deps`. They are separate Docker branches with separate `FROM`
instructions.

### Run 30568944356: `rosdep` was installed but not initialized

Installing `python3-rosdep` supplied the executable but not its generated source
configuration.

```text
ERROR: no sources directory exists on the system meaning rosdep has not yet been initialized.
exit code: 1
```

The correction addressed the missing executable only. It did not inspect the
tool's full installation, initialization, resolution, and cleanup lifecycle.

### Run 30569741208: cleanup tried to remove `sudo`

After dependency resolution worked, `apt-get autoremove` attempted to remove
`sudo`, which had arrived through rosdep's package dependencies.

```text
Refusing to remove sudo.
E: Sub-process /usr/bin/dpkg returned an error code (1)
```

`autoremove` uses the package manager's global automatic-package state. It is not a
safe inverse of one earlier install command. Resolution and final runtime
construction should not have shared one filesystem.

### Run 30571828425: the inherited runtime user was recreated

Moving rosdep into a disposable stage exposed the next hidden assumption.

```text
groupadd: GID '1000' already exists
exit code: 4
```

`ros:jazzy-ros-core-noble` already contains the `ubuntu` user and group at ID 1000.
The development image replaces that user because bind-mounted source needs host UID
matching. The deployed runtime image has no such requirement. Copying the
development-account design into the runtime image created a deterministic
collision.

### Host smoke attempts: the test existed but was unreachable

The smoke test was copied with mode `0444` into a destination directory that Docker
also created with mode `0444`. Root could see the file, but the configured
unprivileged user could not traverse the directory.

```text
/usr/bin/ls: cannot access '/opt/sorting_arm_smoke/test_headless.py': Permission denied
```

Python's `os.path.isfile()` returned false because directory search permission was
missing. `launch_test` therefore reported the file as nonexistent.

The original inspection checked the file as root. That was not the runtime
contract.

### Host smoke attempts: readiness was mixed with upstream shutdown

After the permission fix, the smoke test reached every readiness condition. It then
failed because post-shutdown assertions required clean exit behavior from MoveIt
and Gazebo.

MoveIt segfaulted while destroying its executor after `SIGINT`. Gazebo exceeded
the nested launcher's five-second `SIGINT` window and received `SIGTERM`.

Those details are real, but this project does not own MoveIt's executor destructor
or Gazebo's signal latency. The test was supposed to prove that this project's
installed headless stack becomes ready.

The next attempt still rejected MoveIt's expected fixed-pose-stage message:

```text
[ERROR] No 3D sensor plugin(s) defined for octomap updates
```

No 3D occupancy-map updater exists yet. A broad scan for every `ERROR` string made
that expected limitation indistinguishable from a failed runtime.

## Root cause

The first false contract was treating a multi-stage Dockerfile as one inherited
environment.

`build-deps` starts from `ros:jazzy-ros-base-noble`. That image has development
tools and initialized rosdep state. `runtime-deps` started independently from
`ros:jazzy-ros-core-noble`. It inherited none of that state.

The deeper process failure was correcting one stopped command at a time:

```text
missing executable
  -> missing generated configuration
  -> unsafe package cleanup
  -> duplicate user and group
  -> inaccessible smoke-test path
  -> upstream shutdown treated as our contract
  -> expected startup limitation treated as failure
```

The shell used `&&`, so each run stopped at the first false assumption. Later
commands were still visible in the Dockerfile and should have been audited before
the next push.

Docker and Buildx were unavailable inside the development container. That made
GitHub or the Machine ROS Dev host the first exact environment for each correction.
The environment limitation increased feedback time, but it did not require
symptom-by-symptom reasoning.

## Final correction

The runtime image now separates responsibility by stage:

- `runtime-dependency-resolver` uses rosdep to derive apt packages from the four
  `package.xml` manifests.
- `runtime-deps` installs the generated package set from a fresh runtime base.
- `runtime-deps-check` verifies the installed dependency closure and is discarded.
- `runtime-headless` inherits no resolver, rosdep, `sudo`, compiler, source, build
  tree, or test tooling.
- The inherited `ubuntu` account is verified and reused.

The manifests remain the dependency source of truth. The only manual runtime policy
is the reviewed list of six GUI rosdep keys excluded from the headless image.

The smoke target creates `/opt/sorting_arm_smoke` explicitly with mode `0755` and
copies the test as a root-owned mode `0444` file. The runtime user can traverse and
read the path but cannot modify it.

The active smoke test owns these decisions:

- simulation clock advances;
- joint states arrive;
- all three controllers are active;
- the MoveIt action server exists;
- required processes remain live during the check; and
- RViz does not run.

It rejects unexpected startup `ERROR` and `FATAL` lines while permitting only the
exact missing-3D-sensor message during the fixed-pose stage.

It does not assert clean upstream teardown.

## Final evidence

Commit `5e870cc6c166cd7de647d71c6284e92b290ce580` triggered GitHub Actions run
[`30588058008`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30588058008).
Every owned workflow step passed:

- all four packages built in Release mode;
- `colcon test-result --verbose` reported 37 tests, zero errors, zero failures, and
  zero skipped tests;
- runtime dependencies resolved and passed `rosdep check`;
- `runtime-headless` built and passed final-image inspection;
- the smoke image built; and
- `test_runtime_becomes_ready` passed in 8.906 seconds.

The successful log still records MoveIt exit code `-11` and forced termination of
the nested launcher after readiness passed. Green status therefore means the
headless stack became ready within the bounded test. It does not mean MoveIt and
Gazebo shut down cleanly.

## Prevention

- Audit every command after the first stopped command before pushing another
  correction.
- Treat every Docker `FROM` as a new machine unless it explicitly names an earlier
  stage.
- Keep temporary resolvers and verification tools outside the final image's
  ancestor chain.
- Inspect files, directories, users, and writable paths as the configured runtime
  user.
- Keep smoke assertions inside project ownership. Record upstream limitations
  without turning them into unrelated project failures.
- Use exact, narrow exceptions for expected upstream messages. Remove the sensor
  exception when a 3D occupancy-map updater becomes a declared feature.
- Distinguish understood cause, implemented correction, local static evidence, host
  runtime evidence, and complete GitHub evidence.

## Branch workflow conclusion

Remote feature branches still matter because they provide the commit head used by
GitHub Actions and the pull request merged into `main`.

The unsafe part is checking out different branches concurrently inside the same
bind-mounted `/sorting_arm_ws`. A checkout changes the files seen by every container
and session using that directory.

Use one checked-out branch at a time in this workspace. Parallel branch work needs a
sibling Git worktree with its own container and separate `build`, `install`, and
`log` state. Do not place another worktree inside `/sorting_arm_ws`.

## Conclusion

The headless CI readiness gate is complete. Further CI redesign is not justified by
this incident.

Clean shutdown remains a separate, unproven behavior. It should be revisited only
through a bounded contract for behavior this project can control.

Project work returns to roadmap Step 0: runtime prerequisite measurements.
