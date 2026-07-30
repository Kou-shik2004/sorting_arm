# Runtime image could not run rosdep

## Overview

The headless runtime image failed to build in GitHub Actions. The failure happened
before runtime inspection and before the Gazebo/MoveIt smoke test.

The `runtime-deps` stage tried to run `rosdep`, but that stage did not contain the
command.

## Impact

The clean workspace build and registered ament tests passed. The workflow then
stopped while building `runtime-headless`.

The runtime image was not created. Runtime inspection and the headless smoke test
did not run.

## Evidence

GitHub Actions run `30566668383` reported:

```text
[runtime-deps 6/6] RUN ... rosdep update --rosdistro jazzy ...
/bin/bash: line 1: rosdep: command not found
exit code: 127
```

Exit code `127` means the shell could not find the requested command.

## Root cause

`python3-rosdep` was installed in the `build-deps` stage. The workspace build and
tests inherited from that stage, so they could use `rosdep`.

`runtime-deps` deliberately starts again from `ros:jazzy-ros-core-noble`. A new
Docker stage inherits only from the image named in its own `FROM` instruction. It
does not inherit packages installed in an unrelated earlier stage.

The runtime stage copied the four `package.xml` files and called `rosdep`, but it
did not install `python3-rosdep` first.

The incorrect assumption was that a command used successfully in one stage would
also exist in a later stage. That is true only when the later stage inherits from
the earlier one.

## Why earlier checks passed

The `workspace-builder` and `test` stages both inherit from `build-deps`.
`build-deps` installs `python3-rosdep`, so dependency resolution, compilation, and
the registered ament tests all passed.

`runtime-deps` is a separate branch of the Docker build graph. It starts from
`ros:jazzy-ros-core-noble`, which does not contain `rosdep`. The first command that
depended on the incorrect assumption was therefore reached only when GitHub built
the `runtime-headless` target.

The development container could not expose this problem. It has rosdep and other
development tools installed, while the purpose of `runtime-deps` is to start from a
smaller base. Docker and Buildx were also unavailable inside the development
container, so the first complete build of this stage happened on GitHub.

## Fix

The `runtime-deps` stage now:

1. installs `python3-rosdep`;
2. verifies that the `rosdep` executable is available;
3. uses it to install only dependencies of type `exec`;
4. purges `python3-rosdep`;
5. removes packages that became unused after the purge;
6. verifies that the `rosdep` executable is no longer available; and
7. removes the root rosdep cache and temporary package manifests.

All of this happens in one Docker layer. The resolved application dependencies
remain, while the dependency-resolution tool does not remain in the delivered
image.

The finished runtime image is checked a second time by the workflow. Runtime-image
inspection fails if a `rosdep` executable is present.

## Prevention

Each Docker stage owns every command it executes. When a stage starts with a new
`FROM`, its base image and the instructions inside that stage are its complete
environment. Commands installed in an unrelated stage are not available.

For temporary build tools in a runtime stage, install, use, and remove the tool in
the same `RUN` instruction. Check availability immediately after installation and
check absence immediately after removal.

The workflow then checks the finished image independently. This gives the boundary
two protections:

- the Dockerfile proves its temporary-tool lifecycle while creating the layer; and
- runtime inspection proves the delivered image does not expose the tool.

Future stages must follow the same rule for any command that is not guaranteed by
their own base image. This catches the error at the stage where the assumption is
made instead of allowing a later container run to reveal it.

## Verification

The GitHub log establishes the original failure mechanism. Local inspection confirms
that `python3-rosdep` is installed and checked before its first use, then removed and
checked again before the layer completes.

The corrected runtime image build, absence check, and full headless smoke test remain
pending the next GitHub Actions run.
