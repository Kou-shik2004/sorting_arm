# CI clean build failed: executive manifest omitted, again

## Overview

GitHub Actions run
[`31041770238`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/31041770238)
failed while building commit `abfdc96` on 2026-08-05. The clean image could not
configure `sorting_arm_executive`.

This is the same failure class as
[`latest-push-ci-contract-breaks.md`](latest-push-ci-contract-breaks.md)'s Failure 1,
five packages later. That fix added the missing manifest; it did not add anything
that would catch the next missing manifest.

## Impact

Colcon aborted before `sorting_arm_executive` finished configuring. Every stage after
`test` in `.docker/runtime/Dockerfile` (the runtime dependency resolution, the
runtime image, the smoke test) never ran.

## Failure: executive manifest omitted from all three rosdep inputs

The decisive error:

```text
CMake Error at CMakeLists.txt:22 (find_package):
  Could not find a package configuration file provided by "behaviortree_cpp"
Failed   <<< sorting_arm_executive [0.91s, exited with code 1]
```

`.docker/runtime/Dockerfile` copies workspace manifests into `/tmp/sorting-arm-src`
by hand, in three separate stages, so rosdep can resolve dependencies without the
whole workspace being present yet. `sorting_arm_executive/package.xml` was in none
of the three lists. `workspace-builder` still built the executive, because it does a
plain `COPY src/ src/`, so the build reached a package whose dependency,
`behaviortree_cpp`, was never installed.

The dev image hides this class of bug entirely: `.docker/packages/ros.txt` already
installs `ros-jazzy-behaviortree-cpp`, so a manifest that never mentions it still
resolves fine inside a dev container.

## Why this happened twice

The first fix treated the missing manifest as the bug and added a line. The actual
contract, that every package under `src/` has a matching manifest COPY in all three
lists, had no enforcement, so it broke again the next time a package was added.

## Correction

- Added `src/sorting_arm_executive/package.xml` to all three manifest lists.
- Added a check directly after `COPY src/ src/` in the `workspace-builder` stage: it
  diffs the package names found under `src/` against the package names present in
  `/tmp/sorting-arm-src` and fails the build if they do not match. A package added to
  `src/` and left out of the manifest lists now fails in seconds with a named cause,
  not minutes later inside an unrelated `find_package` error.

## Verification evidence

A new GitHub Actions run on the pushed commit is the only acceptable evidence.
Docker is not available inside the development container, so the failure could not
be reproduced or fixed locally.
