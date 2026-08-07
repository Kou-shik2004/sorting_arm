# RCA

Root cause analyses for this project's genuinely persistent failures: the ones that
took real investigation, produced wrong theories along the way, or came back more
than once. Most bugs get fixed and forgotten; these three didn't, and the reasoning
behind each fix is worth keeping.

This is not the place for a failure that was found and fixed in one pass. If a
problem didn't take real investigation, it doesn't earn a document here.

## Format

Every entry follows the same structure:

```markdown
# RCA: <Issue Name>
## Problem
## Expected Behavior
## Root Cause
## Fix
## Verification
## Prevention
## Lessons Learned
## References
```

**Problem** and **Expected Behavior** separate what actually happened from what
should have happened. **Root Cause** explains the underlying reason, not just the
symptom. **Fix** is exactly what changed. **Verification** is how the fix was
confirmed, with real evidence, not "should work." **Prevention** is what stops the
same class of bug from happening again. **Lessons Learned** is the reusable insight,
the kind of thing worth remembering even outside this specific bug.

## Entries

| File | Issue |
|---|---|
| [gripper-grasp-instability.md](gripper-grasp-instability.md) | The simulated gripper closed unreliably for most of a multi-session investigation into the physics engine and controller interaction |
| [ci-package-manifest-omission.md](ci-package-manifest-omission.md) | CI's clean build kept missing a new package's dependencies, twice, because nothing checked the hand-written manifest list against `src/` |
| [headless-ci-hidden-assumptions.md](headless-ci-hidden-assumptions.md) | Building the headless CI image took four failed runs, each one correcting a different wrong assumption about what carries over between Docker build stages |
