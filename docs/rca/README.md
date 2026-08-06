# RCA

Short write-ups of real failures and how they were fixed. Not design notes, not
tutorials, just the record of what went wrong.

## Format

Each entry answers three questions, in this order:

1. **Issue faced.** What broke, with the exact error or symptom.
2. **What we did.** How the cause was found.
3. **How we solved it.** The fix, and what it changes.

Keep it short. A file that turns into a full investigative log has outgrown this
format, and that's fine, some failures need one, but say so instead of forcing a
long story into a three-line shape.

## Entries

| File | Issue |
|---|---|
| [ci-executive-manifest-omitted.md](ci-executive-manifest-omitted.md) | CI's clean build never installed `sorting_arm_executive`'s dependencies |
| [latest-push-ci-contract-breaks.md](latest-push-ci-contract-breaks.md) | Three separate CI breaks from one push: a missing manifest, an unformatted package, a Release-only compiler warning |
| [runtime-stage-missing-rosdep.md](runtime-stage-missing-rosdep.md) | The headless runtime image took four failed CI runs to build, each one exposing a hidden assumption about the base image |
| [gripper-command-vs-moveit-planning.md](gripper-command-vs-moveit-planning.md) | Gripper motion was planned through MoveIt instead of driven directly, which is the wrong abstraction for a one-joint gripper |
| [gripper-controller-configuration.md](gripper-controller-configuration.md) | The generated gripper controller used a trajectory-controller schema while MoveIt expected a `GripperCommand` action |
| [gripper-grasp-instability.md](gripper-grasp-instability.md) | The simulated gripper closed unreliably for most of a multi-session investigation into the physics engine and controller interaction |
