# Runtime image assumptions were checked one failure at a time

## Overview

The headless runtime image failed in four consecutive GitHub Actions runs. The
first three stopped in `runtime-deps`. The fourth passed dependency verification
and then stopped while creating the runtime user.

The corrected runtime image later reached the smoke target during Machine ROS Dev
host checks. Both the build-time preflight and the smoke container reported that
the copied launch test did not exist.

These were not unrelated package errors. The runtime design was changed one visible
failure at a time without checking the complete stage graph, exact base image, and
final validation rules first.

The final Machine ROS Dev host smoke run returned status `0`. That proves runtime
readiness on the host. It does not yet prove the complete GitHub Actions path, and
it does not prove clean shutdown of MoveIt and Gazebo.

## Impact

The clean workspace build and registered ament tests passed in all four runs.
`runtime-headless` did not finish building in any of them.

The final-image checks, smoke-image build, and bounded Gazebo/MoveIt smoke test were
skipped. None of those runs proved that the new runtime image worked.

The later host work reached and passed the bounded readiness smoke. CI remained
blocked throughout the session because no corrected commit was pushed and no
complete GitHub Actions run executed.

## Evidence

GitHub Actions run `30566668383` reported:

```text
[runtime-deps 6/6] RUN ... rosdep update --rosdistro jazzy ...
/bin/bash: line 1: rosdep: command not found
exit code: 127
```

The first correction installed `python3-rosdep`. Run `30568944356` then reported:

```text
ERROR: no sources directory exists on the system meaning rosdep has not yet been initialized.
exit code: 1
```

The second correction initialized rosdep. Run `30569741208` resolved and installed
all requested runtime dependencies. Cleanup then reported:

```text
The following packages will be REMOVED:
  python3-rosdep-modules sudo
Refusing to remove sudo.
E: Sub-process /usr/bin/dpkg returned an error code (1)
```

`python3-rosdep-modules` has a hard package dependency on `sudo`. Ubuntu's `sudo`
removal script refuses to remove the last privilege-escalation tool when the root
account has no password. That is the normal state inside this container.

The later `Check build summary support` message came from
`docker/build-push-action`. It only reported the build result. It did not cause the
failure.

The next correction moved rosdep into a disposable verification stage. Run
`30571828425` passed that stage and then reported:

```text
groupadd: GID '1000' already exists
exit code: 4
```

The exact `ros:jazzy-ros-core-noble` image used by that run already contains:

```text
ubuntu:x:1000:1000:Ubuntu:/home/ubuntu:/bin/bash
ubuntu:x:1000:
```

The account collision was therefore deterministic. It was not a GitHub runner or
cache problem.

The first Machine ROS Dev host check copied the smoke test, then a build-time
preflight reported:

```text
launch_test: error: Test file '/opt/sorting_arm_smoke/test_headless.py' does not exist
exit code: 1
```

Removing that redundant preflight allowed the smoke image to build, but the
container reported the same error. Inspecting the final image as root showed that
the file was present and regular. Inspecting it as the configured `ubuntu` user
showed the false contract:

```text
dr--r--r-- 2 ubuntu ubuntu ... /opt/sorting_arm_smoke
/usr/bin/ls: cannot access '/opt/sorting_arm_smoke/test_headless.py': Permission denied
```

The file itself was mode `0444`. Its parent directory was also mode `0444`.
Owning or reading a directory does not allow traversal. A process needs the
directory's execute, also called search, permission to access a child path.

`launch_test` checks the path with Python's `os.path.isfile()`. Python returns
`False` when it cannot traverse the parent directory, so `launch_test` described
the inaccessible file as nonexistent.

After the directory mode was corrected, the active smoke test reached every
readiness condition. The clock advanced, joint states arrived, all three
controllers were active, the MoveIt action server was available, and the required
processes were still running.

Jazzy `launch_testing` then shut down the process under test. The test had started
the complete stack as one nested `ros2 launch` process. That launcher forwarded
`SIGINT` to its MoveIt and Gazebo children. MoveIt segfaulted in
`rclcpp::Executor::~Executor()`, and Gazebo exceeded launch's five-second
`SIGINT` window before receiving `SIGTERM`.

Three post-shutdown assertions turned those upstream teardown details into project
failures:

- the nested launcher did not return the expected exit code;
- its shutdown output contained `ERROR`; and
- a Gazebo process still appeared in `/proc` when the assertion ran.

After those assertions were removed, the next run still returned status `1`. The
`NO TESTS RAN` line described the empty optional post-shutdown suite and was not the
failure. The active test had rejected this MoveIt startup line:

```text
[ERROR] No 3D sensor plugin(s) defined for octomap updates
```

This message is expected in the current fixed-pose stage. The project has no depth
or point-cloud occupancy-map updater yet. It does not mean that MoveGroup,
controllers, or the readiness interfaces failed.

## Root cause

The build and runtime branches start from different images:

- `build-deps` starts from `ros:jazzy-ros-base-noble`;
- `runtime-deps` starts from `ros:jazzy-ros-core-noble`.

The official `ros-base` Dockerfile installs `python3-rosdep` and runs `rosdep init`.
The smaller `ros-core` Dockerfile does neither. A new Docker stage inherits only
from the image named by its own `FROM` instruction.

The runtime stage copied logic that depended on the `ros-base` toolchain into a
stage based on `ros-core`. It then tried to keep the final image small by purging
the copied toolchain after use.

That combined two different responsibilities:

1. resolving project declarations into operating-system packages; and
2. producing the filesystem that will run the application.

The cleanup was also broader than the installed tool. `apt-get autoremove` acts on
the package manager's global automatic-package state. It is not a precise inverse
of one earlier `apt-get install` command.

The next design introduced a second source of truth:

- `package.xml` declared the project dependencies;
- `.docker/packages/runtime.txt` repeated the corresponding apt packages.

The disposable `rosdep check` could detect drift after both files changed, but it
could not make the apt list authoritative. The project container context already
records that the deployment image must derive its dependencies from the manifests.

The runtime user had the same problem. The development image deletes the inherited
`ubuntu` account and creates `kratos` so bind-mounted files match the host UID. The
deployment image has no editable source bind mount and nobody logs into it. Copying
the development account creation without its reason caused the GID collision.

Inspection also found that the current final-image validation would fail after the
user fix. The supported Jazzy package closure contains `cmake`, `cppcheck`,
`uncrustify`, and `launch_testing`. These are supplied by ROS package dependencies,
not copied from the builder. Purging them would break the supported apt dependency
state.

The smoke test used this Docker instruction:

```dockerfile
COPY --chown=ubuntu:ubuntu --chmod=0444 \
  .docker/smoke/test_headless.py \
  /opt/sorting_arm_smoke/test_headless.py
```

`/opt/sorting_arm_smoke` did not exist before the copy. Docker created the missing
destination directory with the requested `0444` mode as well as applying that mode
to the file. Changing ownership to `ubuntu` did not add directory search
permission.

The build-time `test -f` and `test -r` checks were misleading. They ran in the
image-construction environment and did not prove that the final container's
unprivileged process could traverse every directory in the path. The runtime
inspection as `ubuntu` exposed the permission denial directly.

The shutdown assertions crossed the project boundary. This project owns whether
the installed stack becomes ready and remains healthy during the smoke check. It
does not own MoveIt's executor destruction or Gazebo's signal-handling latency.
Treating their teardown implementation as a project decision contradicted the test
rule that forbids testing MoveIt, controllers, and physics.

The installed Jazzy `launch_testing` runner executes active tests while the process
under test is running. It then calls `LaunchService.shutdown()` and optionally runs
classes decorated with `post_shutdown_test`. Those optional tests are useful when
the project owns the launched process and its exit contract. Here,
`ExecuteProcess` owned only the nested `ros2 launch` wrapper, not the individual
upstream processes whose teardown produced the failures.

## Why the failure repeated

The runtime work was one long shell command joined with `&&`. The shell stopped at
the first failing command, so later assumptions were not exercised:

```text
missing executable
  -> missing generated configuration
  -> temporary dependency could not be removed
  -> inherited user and group were recreated
  -> supported ROS tools were treated as builder leakage
  -> copied smoke test was unreachable by the runtime user
  -> upstream teardown was treated as a project exit contract
  -> one expected MoveIt message failed a broad error check
```

Each correction handled only the error visible in that run. It did not trace the
complete tool lifecycle before the next push.

The passing workspace checks did not protect this path. The `test` stage inherits
from `build-deps`, while `runtime-deps` is a separate Docker branch. Passing one
target says nothing about the other.

The development container also has rosdep and development packages already
installed. Docker and Buildx are not available inside it. GitHub was therefore the
first environment to build each revised runtime stage.

The same environment mismatch affected the smoke test. Running
`launch_test --show-args` in the development container proved only that the local
Jazzy installation could import the repository copy. It said nothing about the
file mode, parent-directory mode, user, package set, or entrypoint inside the smoke
image.

The smoke-file diagnosis initially checked whether root could see the file. Root
could, so the file looked present. The real contract was whether `ubuntu` could
search every directory component. Checking as that user immediately exposed
`Permission denied` on a directory that had no execute bits.

After readiness worked, one test still combined three different ownership
boundaries:

1. project-owned startup and ROS interface readiness;
2. MoveIt and Gazebo shutdown behavior; and
3. a broad scan of all `ERROR` and `FATAL` text.

That made an upstream destructor failure, Gazebo shutdown latency, and an expected
missing-sensor message indistinguishable from failure of the runtime image.

The RCA was updated after partial corrections while the full runtime build was
still pending. Local inspection was described as a correction even though no
`runtime-headless` image had completed. That made the status sound stronger than
the evidence.

## Structural correction

A disposable `runtime-dependency-resolver` stage now reads the four manifests and
generates the apt package set. The only manual dependency input is the reviewed
list of six GUI rosdep keys excluded from the headless server image.

The resolver fails when:

- an exclusion no longer appears in the manifests;
- a required key is unresolved;
- a key uses an installer other than apt; or
- the generated package set is empty.

`runtime-deps` mounts the generated file read-only from the resolver stage and
installs it from a fresh `runtime-base`. It does not inherit the resolver's
filesystem.

`runtime-deps` no longer contains:

- rosdep;
- rosdep source configuration or cache;
- `sudo`;
- copied project manifests; or
- purge and autoremove cleanup; or
- a hand-maintained apt package list.

A separate `runtime-deps-check` stage inherits from the completed dependency stage.
It installs and initializes rosdep, checks the four project manifests, and is then
discarded. Its rosdep and `sudo` packages cannot become ancestors of
`runtime-headless`.

This makes `package.xml` the runtime dependency source of truth. The separate check
stage still runs `rosdep check` against the installed result.

The final image reuses the inherited `ubuntu` account. It creates only the Gazebo,
ROS log, cache, and XDG runtime directories that the headless process must write.
It does not delete, recreate, or rename a base-image user or group.

The runtime boundary now excludes packages added by this project's build and
verification stages. It checks that compilers, Colcon, rosdep, sudo, source, build
output, and the six project GUI packages are absent. It accepts supported packages
that ROS and Ubuntu pull transitively.

The apt stages also remove Ubuntu's `docker-clean` hook and enable retained package
downloads before using BuildKit apt cache mounts. Without that setup, the hook
empties `/var/cache/apt` after every install.

The smoke target now creates `/opt/sorting_arm_smoke` explicitly as root-owned mode
`0755`. It then copies the test as a root-owned mode `0444` file. The runtime user
can traverse and read the path but cannot modify the directory or test.

The Docker build checks both exact modes after the copy. Its `CMD` remains the real
`launch_test` invocation, so the bounded container run owns test discovery,
startup, assertions, and shutdown.

The active smoke test still rejects startup `ERROR` or `FATAL` output. The three
post-shutdown assertions were removed because they asserted upstream exit behavior
after the project readiness contract had passed. Jazzy `launch_testing` still
requests shutdown after the active test. Docker's init process and container
lifetime reap remaining descendants, while the host and CI timeout bound the whole
operation.

The startup error check permits only MoveIt's exact missing-3D-sensor message while
the fixed-pose provider is active. Every other startup `ERROR` or `FATAL` line still
fails the smoke test.

## Prevention

A delivered Docker stage must contain runtime behavior only. Temporary resolvers,
compilers, and verification tools belong in build or check stages that are not
ancestors of the final image.

Sibling Docker targets need separate evidence. A passing workspace target cannot
be used as evidence for a runtime target.

Before adding or replacing a base-image account, inspect `/etc/passwd`,
`/etc/group`, the home directory, and why the project needs a different identity.
Development bind-mount ownership is not a deployment requirement.

Manifest-driven resolution must remain the only dependency source. A verification
step does not make a duplicated package list safe from drift.

Create a destination directory explicitly before copying a read-only file into a
new path. File read permission and directory search permission are separate
contracts. Changing ownership does not replace either permission.

For a file used by a non-root runtime user, inspect every path component as that
user. A root-only `stat` or a Docker build check cannot prove runtime access.

Keep smoke assertions inside the ownership boundary. Startup errors and readiness
interfaces belong to this runtime image. MoveIt and Gazebo teardown internals do
not. A post-shutdown assertion is appropriate only when this project owns the
process and its exit contract.

An expected upstream error needs a narrow, reviewed match. Do not disable the
whole error check. Remove this exception when a 3D occupancy-map updater becomes a
declared runtime feature.

When a CI step contains several dependent operations, inspect the commands after
the first failure before calling a correction complete. A short-circuited shell
command proves only the path reached so far.

An RCA must distinguish these states:

- the failure mechanism is understood;
- a correction has been implemented;
- deterministic checks passed; and
- the complete GitHub runtime and smoke path passed.

Only the last state closes this CI failure.

## Verification

The four GitHub logs establish the failure chain. Run `30571828425` proves that the
clean workspace and disposable dependency check passed, then records the exact
account collision that stopped runtime construction.

The structural correction and workflow changes are implemented locally. Local
checks confirmed that:

- rosdep derives 22 apt packages from the current manifests after the six GUI
  exclusions;
- every non-excluded runtime key resolves through apt;
- all six exclusions are present in the current manifest dependency set;
- the exact failed base image already owns the `ubuntu` user, group, and
  `/home/ubuntu` at ID 1000;
- the supported package closure explains why CMake, Cppcheck, and Uncrustify are
  present;
- the workflow YAML and four package manifests parse;
- rosdep reports that the current development environment satisfies the same
  headless dependency set; and
- local and remote branch heads both pointed to
  `7c5ca2e7d59da10153bf895e5d66afd5ca383121` during the audit.

The Machine ROS Dev runtime inspection also confirmed that the smoke test was
present but inaccessible because `/opt/sorting_arm_smoke` lacked search
permission. The explicit `0755` directory correction was then observed in the
rebuilt image.

The next host run passed every active readiness assertion. Its failures came only
from the three post-shutdown assertions against upstream teardown. The corrected
smoke-test boundary is implemented locally.

The following run reached the same readiness state but failed the broad startup
error check on MoveIt's expected missing-3D-sensor message. The narrow exception is
implemented locally.

The final Machine ROS Dev host run rebuilt the smoke target, passed the active
readiness test, completed the bounded container run, and returned status `0`. The
nested launcher still needed `SIGTERM` after its five-second `SIGINT` window, but
that happened after the owned readiness and startup-error contracts passed.

Docker and Buildx are unavailable in the development container. The complete
GitHub Actions run is still pending. This RCA must not claim the CI failure closed
until the workspace tests, resolver, dependency check, runtime build, final-image
inspection, and bounded headless smoke test pass together in that workflow.

## Current status and next-session handoff

The original runtime-stage dependency and construction failures have implemented
corrections:

- rosdep resolution happens in a disposable stage;
- the final runtime dependency set comes from the package manifests;
- rosdep verification cannot leak tools into the runtime image;
- the inherited `ubuntu` account replaces duplicate UID/GID creation;
- final-image validation distinguishes project-added tools from supported
  transitive packages;
- `/opt/sorting_arm_smoke` is explicitly mode `0755`;
- `test_headless.py` is root-owned mode `0444`; and
- the smoke test checks runtime readiness and returns status `0` on the host.

The missing-file failure is solved. Its direct cause was the `0444` parent
directory, not a missing Docker build-context file.

The overall CI incident is not closed. The next session must obtain these remaining
results:

1. Build and validate `runtime-headless` from the intended commit.
2. Build and run the smoke image from the same commit.
3. Push the branch through the repository-owner command boundary.
4. Observe one complete GitHub Actions run containing the workspace test,
   dependency check, runtime build, image validation, smoke build, and bounded
   smoke execution.
5. Record the workflow run ID and result here.

Clean shutdown remains unresolved. The passing smoke result no longer fails on
MoveIt and Gazebo teardown because those post-shutdown assertions were removed.
The observed shutdown still contains two facts:

- MoveGroup previously segfaulted in `rclcpp::Executor::~Executor()` after
  `SIGINT`; and
- the nested `ros2 launch` process still exceeded the five-second `SIGINT` window
  and required `SIGTERM`.

This is not evidence of clean shutdown. A later decision must either define the CI
gate as readiness-only and update the governing test documentation, or create a
separate shutdown contract that tests behavior this project can own. Do not restore
the old broad assertions or silently describe status `0` as a clean shutdown.

The working branch was `fix/runtime-dependency` at
`7c5ca2e7d59da10153bf895e5d66afd5ca383121` before these uncommitted changes. The
intended runtime slice touches `.docker/`, `.github/workflows/ci.yml`, and this RCA.
Unrelated `.gitignore`, `.agents/`, and `.docker/smoke/__pycache__/` working-tree
entries must not be included in the runtime commit.

For branch work in this bind-mounted development setup, use one checked-out branch
at a time. Switch branches from the host only after the current work is committed
or otherwise clean. Do not add worktrees inside the ROS workspace unless concurrent
branch work becomes a demonstrated need; each worktree would require separate
build, install, log, and container state.
