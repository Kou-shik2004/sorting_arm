# Runtime dependency resolution was mixed into the delivered image

## Overview

The headless runtime image failed in three consecutive GitHub Actions runs. Each
run stopped in the `runtime-deps` Docker stage before the image could be inspected
or started.

These were not three unrelated failures. They were three parts of one incomplete
stage design. The stage tried to install a development-time dependency resolver,
use it, and remove it again while building the delivered runtime filesystem.

## Impact

The clean workspace build and registered ament tests passed in all three runs.
`runtime-headless` did not finish building.

The final-image checks, smoke-image build, and bounded Gazebo/MoveIt smoke test were
skipped. None of those runs proved that the new runtime image worked.

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

## Why the failure repeated

The runtime work was one long shell command joined with `&&`. The shell stopped at
the first failing command, so later assumptions were not exercised:

```text
missing executable
  -> missing generated configuration
  -> temporary dependency could not be removed
```

Each correction handled only the error visible in that run. It did not trace the
complete tool lifecycle before the next push.

The passing workspace checks did not protect this path. The `test` stage inherits
from `build-deps`, while `runtime-deps` is a separate Docker branch. Passing one
target says nothing about the other.

The development container also has rosdep and development packages already
installed. Docker and Buildx are not available inside it. GitHub was therefore the
first environment to build each revised runtime stage.

The RCA was updated after partial corrections while the full runtime build was
still pending. Local inspection was described as a correction even though no
`runtime-headless` image had completed. That made the status sound stronger than
the evidence.

## Structural correction

The delivered runtime stage now installs a reviewed list of headless apt packages.
The list contains the top-level packages rosdep selected successfully in run
`30569741208`. Apt still owns their transitive system dependencies.

`runtime-deps` no longer contains:

- rosdep;
- rosdep source configuration or cache;
- `sudo`;
- copied project manifests; or
- purge and autoremove cleanup.

A separate `runtime-deps-check` stage inherits from the completed dependency stage.
It installs and initializes rosdep, checks the four project manifests, and is then
discarded. Its rosdep and `sudo` packages cannot become ancestors of
`runtime-headless`.

This keeps `package.xml` as the dependency source of truth. If a future manifest
adds a required runtime dependency without updating the apt list,
`rosdep check` fails in the disposable stage.

## Prevention

A delivered Docker stage must contain runtime behavior only. Temporary resolvers,
compilers, and verification tools belong in build or check stages that are not
ancestors of the final image.

Sibling Docker targets need separate evidence. A passing workspace target cannot
be used as evidence for a runtime target.

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

The three GitHub logs establish the failure chain and show that the selected
runtime apt packages were resolved successfully before cleanup failed.

The structural correction and workflow changes are implemented locally. Local
checks confirmed that:

- the runtime manifest contains the same 20 top-level apt packages recorded in run
  `30569741208`;
- every listed apt package is available to the current Jazzy environment;
- the workflow YAML and four package manifests parse;
- rosdep accepts the new check command and reports satisfied dependencies in the
  development container; and
- `runtime-headless` inherits from the clean dependency stage, not from
  `runtime-deps-check`.

Docker and Buildx are unavailable in the development container. The complete
GitHub Actions run is still pending. The RCA must not claim the runtime image is
fixed until `runtime-deps-check`, final-image inspection, and the bounded headless
smoke test all pass in one run.
