# RCA: CI's clean build kept missing a package's manifest

## Problem

`.docker/runtime/Dockerfile` copies every workspace `package.xml` into a rosdep input
directory by hand, in three separate stages, so dependencies can resolve before the
whole workspace is present. Twice, a new package's manifest was never added to that
hand-written list. Both times, the clean CI build reached that package and failed
with a missing dependency it should never have hit:

```text
CMake Error at CMakeLists.txt:22 (find_package):
  Could not find a package configuration file provided by "cv_bridge"
```

```text
CMake Error at CMakeLists.txt:22 (find_package):
  Could not find a package configuration file provided by "behaviortree_cpp"
```

First `sorting_arm_perception` (2026-08-04, run `30955821873`), then
`sorting_arm_executive` (2026-08-05, run `31041770238`), five packages later.

## Expected Behavior

Every package under `src/` has its `package.xml` reflected in all three manifest
lists the runtime Dockerfile reads (`build-deps`, `runtime-dependency-resolver`,
`runtime-deps-check`). Adding a package to the workspace, and nothing else, is enough
for CI's clean build to resolve its dependencies correctly.

## Root Cause

The manifest lists are hand-written `COPY` instructions, one per package, and nothing
checked them against what actually exists under `src/`. The development container
never surfaces this gap at all: `.docker/packages/ros.txt` already installs
`ros-jazzy-cv-bridge` and `ros-jazzy-behaviortree-cpp` directly, so a manifest that
never mentions either dependency still resolves fine inside a dev container. The
clean CI build was the only path that ever exercised the manifest lists honestly.

The first fix treated the missing manifest as the bug and added the missing line.
That corrected the immediate failure but added no enforcement, so the same gap
reopened the next time a package was added, and did.

## Fix

- Added the missing `package.xml` to all three manifest lists both times.
- After the second occurrence, added a check directly following `COPY src/ src/` in
  the `workspace-builder` stage: it diffs the package names actually found under
  `src/` against the package names present in the hand-written manifest set, and
  fails the build immediately if they don't match.

```dockerfile
RUN find src -mindepth 2 -maxdepth 2 -name package.xml -printf '%h\n' | xargs -n1 basename | sort > /tmp/src-packages.txt \
 && find /tmp/sorting-arm-src -mindepth 1 -maxdepth 1 -type d -printf '%f\n' | sort > /tmp/manifest-packages.txt \
 && diff /tmp/src-packages.txt /tmp/manifest-packages.txt
```

A package left out of the manifest lists now fails in seconds with a named cause,
instead of minutes later inside an unrelated `find_package` error.

## Verification

A new GitHub Actions run on the pushed commit was the only acceptable evidence in
both cases; Docker isn't available inside the development container, so neither
failure could be reproduced or fixed locally. Both corrections were confirmed by a
subsequent green run. The `diff` check added after the second occurrence has not
been exercised by a real omission since; it's a structural guarantee, not something
with its own pass/fail history yet.

## Prevention

A new dependency, or a new package, lands in its manifest and the Dockerfile's
manifest lists in the same change. See `.claude/rules/docker-ci.md` for the same rule
stated for the development image, and [`docs/ci.md`](../ci.md#the-manifest-omission-trap)
for what the `diff` check actually catches today.

## Lessons Learned

- A fix that corrects one instance without adding enforcement is a patch, not a
  root-cause fix; the same class of bug will resurface at the next opportunity.
- A convenience the development image provides, apt packages pre-baked ahead of any
  manifest, can hide a contract violation that only a from-scratch build exposes. The
  dev container being green proves nothing about whether the manifests are honest.

## References

- GitHub Actions run [`30955821873`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/30955821873):
  first occurrence, `sorting_arm_perception`.
- GitHub Actions run [`31041770238`](https://github.com/Kou-shik2004/sorting_arm/actions/runs/31041770238):
  second occurrence, `sorting_arm_executive`.
