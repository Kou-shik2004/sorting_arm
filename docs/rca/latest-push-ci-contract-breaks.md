# Latest push broke the clean dependency and lint contracts

## Overview

GitHub Actions run
[`30955821873`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30955821873)
failed while building commit `36fd0f5` on 2026-08-04. The clean image could not
configure the new `sorting_arm_perception` package.

Local validation of the correction then found a second failure from the same push.
`sorting_arm_skills` enabled the workspace's 125-column limit while its existing
sources still contained 73 longer lines.

## Impact

The remote run stopped before compiling `sorting_arm_perception`. Colcon aborted
the remaining dependent work, so deterministic tests and runtime-image checks did
not run.

Without the dependency fix, CI could not build. Without the formatting fix, the
next CI run would build and then fail `sorting_arm_skills` cpplint.

## Failure 1: perception manifest omitted from rosdep input

The decisive GitHub Actions error was:

```text
CMake Error at CMakeLists.txt:22 (find_package):
  Could not find a package configuration file provided by "cv_bridge"
```

The first false contract was that every workspace manifest reached rosdep in the
clean image.

`sorting_arm_perception/package.xml` correctly declares `cv_bridge` and the other
perception dependencies. However, `.docker/runtime/Dockerfile` copied only the five
older package manifests into `/tmp/sorting-arm-src`. Rosdep therefore had no input
that required `cv_bridge`, even though the later workspace copy contained and built
`sorting_arm_perception`.

The development container already carries the perception dependency set from
`.docker/packages/ros.txt`. That existing environment hid the missing clean-image
manifest input.

### Correction

The Dockerfile now copies `sorting_arm_perception/package.xml` into all three
manifest sets:

- `build-deps`, which installs build and test dependencies;
- `runtime-dependency-resolver`, which derives the headless runtime apt set; and
- `runtime-deps-check`, which verifies the installed runtime dependency closure.

Rosdep remains the dependency source of truth. No perception package was added as a
manual apt exception.

## Failure 2: cpplint limit changed before sources were formatted

Commit `36fd0f5` changed `sorting_arm_skills` cpplint from 150 columns to 125. The
workspace-root `.clang-format` already specifies 125 columns, but the package had
not been mechanically reformatted before enabling that check.

The local deterministic suite reported:

```text
Category 'whitespace/line_length' errors found: 73
1 package had test failures: sorting_arm_skills
```

This failure was hidden behind the earlier clean-image build failure because the
remote run never reached the test stage.

### Correction

The workspace-root `clang-format` configuration was applied to the
`sorting_arm_skills` C++ headers and sources. The 125-column check remains active.
No motion behavior or API ownership changed.

## Verification evidence

Local validation on 2026-08-05 produced these results:

- `colcon build --symlink-install`: six packages finished;
- `colcon test` and `colcon test-result --verbose`: 89 tests, zero errors, zero
  failures, and 26 skipped;
- `rosdep check --from-paths src --ignore-src`: all system dependencies satisfied;
- `clang-format --dry-run --Werror`: no formatting errors;
- the Dockerfile contains 18 workspace manifest copies, which is six manifests in
  each of the three rosdep input sets; and
- `git diff --check`: no whitespace errors.

The skipped tests are unchanged registered-linter skips. They are not runtime
evidence.

Docker is unavailable in the development container. A new GitHub Actions run is
still required to prove the clean Buildx stages and later runtime-image checks in
their exact environment.
