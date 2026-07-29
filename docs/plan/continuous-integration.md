# Continuous integration plan

## Purpose

CI provides independent evidence that a clean checkout can reconstruct and validate
the project. It begins before the application is complete and grows as deterministic
tests and runtime interfaces become available.

CI does not claim that the simulated robot works merely because the workspace
compiles.

## Level 1 — per-change workspace CI

Run on pushes and pull requests:

- check out a clean workspace;
- build in the declared ROS 2 environment;
- resolve and check package dependencies;
- validate xacro, URDF, launch XML, and package metadata where applicable;
- build with warnings enabled;
- run all registered deterministic tests; and
- always expose useful test results when a job fails.

Before tests exist, this level honestly proves clean dependency resolution,
configuration validation, and compilation. Each feature slice adds its meaningful
tests to the same pipeline.

Full Gazebo physics is excluded from this level.

## Level 2 — development-image validation

Run when the Dockerfile, Compose file, dependency lists, entrypoint, underlay, or
development-container configuration changes, and allow manual runs:

- build the development image from its declared inputs;
- exercise the image entrypoint and non-root development user;
- build the workspace inside the resulting image; and
- retain enough build output to diagnose a failed dependency or layer.

This job may use layer caching, but a documented clean-build path must remain
available. Publishing the development image is optional until a stable tagging and
registry policy is chosen.

Image validation is kept separate from the normal source loop because rebuilding the
complete development image for every C++ change would add latency without new
evidence.

## Level 3 — simulation smoke

Run manually or on a scheduled job once a deterministic headless path exists:

- start the bounded Gazebo, controller, MoveIt, and application stack;
- wait for explicit readiness conditions rather than fixed sleeps;
- exercise a small observable scenario;
- enforce an overall timeout;
- collect relevant logs and test results; and
- shut the stack down cleanly.

Simulation smoke is not a per-push gate. GPU, graphics, physics timing, and process
startup make it slower and less deterministic than unit or mock-hardware tests.
Interactive simulation remains a manual verification source throughout development.

## Growth path

1. Establish Level 1 with a clean build and the validation checks that already
   exist.
2. Add Level 2 when the first workflow is introduced so the development environment
   validates independently.
3. Add pure geometry, configuration, result, and decision tests with their feature
   slices.
4. Add mock-hardware action integration after the skill server exists.
5. Add Level 3 only after the headless readiness and shutdown contracts are proven
   locally.
6. Build and publish a separate runtime image after the application's runtime
   dependency set and launch command are stable.

## Evidence and failure behavior

- No command may mask a failing exit status.
- Test results must be printed even when an earlier test fails.
- Cached success is not accepted as the only clean-build evidence.
- A launch process merely remaining alive is not proof of readiness.
- CI badges and green checks describe only the levels that actually ran.
