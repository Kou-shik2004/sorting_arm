# Development workflow

## Purpose

Development proceeds through coherent feature slices. A slice is the smallest
reviewable unit that establishes one behavior, contract, or invariant and produces
proportionate evidence. It may touch a header, source file, test, build definition,
configuration, and documentation together.

Files are implementation boundaries, not planning units. Creating isolated files
without a usable contract delays integration; implementing several unrelated
features together makes failures and review ambiguous.

## Slice contract

Before a non-trivial slice is implemented, establish:

- the outcome and why it belongs at the current roadmap gate;
- the owning package and architectural layer;
- inputs, outputs, ownership, and state invariants;
- expected failure and cancellation behavior;
- what is explicitly in and out of scope;
- the files expected to change;
- automated verification; and
- runtime evidence that must be gathered manually.

The contract should be short enough to review quickly. It is not a second design
document. A fully specified mechanical change does not require ceremonial planning.

## Implementation

For each accepted slice:

1. Read the current roadmap step and only the affected architecture and decision
   sections.
2. Inspect the live implementation and settle ROS 2 or MoveIt API behavior from the
   installed version when memory or a reference is insufficient.
3. Open a reference repository only for the matching subproblem. Record which
   assumptions differ before adapting a pattern.
4. Implement the smallest complete behavior, including build wiring and tests that
   belong to that behavior.
5. Keep unrelated cleanup and future abstractions out of the slice.
6. Verify deterministic work locally before requesting runtime evidence.

No temporary sleep, silent fallback, unvalidated hard-coded target, or discarded
native error is accepted as a shortcut.

## Review and understanding

Every completed slice is handed over with:

- its purpose and location in the architecture;
- the important design choices and rejected alternatives;
- the relevant C++, ROS 2, MoveIt, or BehaviorTree APIs;
- a section-by-section code walkthrough;
- the verification performed;
- the exact runtime checks still required; and
- known limitations or deferred behavior.

Line-by-line explanation is used when code is new, subtle, safety-relevant, or
specifically requested. It is not mandatory ceremony for familiar boilerplate.

Understanding is checked by explaining the design in original words, challenging
its assumptions, and correcting misunderstandings. Manually retyping an
implementation is not evidence of understanding.

## Runtime ownership

Compilation, deterministic tests, format checks, and static validation may run
without starting the application. The full Gazebo, MoveIt, and controller stack is
started only by the repository owner unless a specific request delegates that run.

Runtime completion claims require the observed command output or behavior. A clean
build is not evidence that motion, grasping, attachment, or recovery works.

## References

Reference repositories accelerate implementation by providing working patterns,
known failure modes, and negative examples. They do not override this project's
frames, hardware names, interfaces, or architecture.

Research is question-driven:

- derive a new behavior or value from the closest matching source;
- verify the cited implementation rather than relying on a summary;
- compare its assumptions with the local system; and
- stop searching once the current contract is settled.

Pure rearrangement, renaming, or formatting does not require a corpus search.

## Comments and documentation

Code comments explain reasons, invariants, counter-intuitive API behavior, or
hardware-specific constraints. They do not narrate syntax, preserve dead code, or
turn an experiment into a tutorial.

Stable architecture and decisions belong in `docs/project/`. Ordered implementation
and verification gates belong in `docs/plan/`. A reusable, evidence-backed failure
analysis belongs in `docs/rca/`. Temporary task notes are not project truth.

## Completion and Git

A slice is complete only when its contract is implemented, deterministic checks
pass, required runtime evidence is recorded, and the documentation still matches
the code.

The repository does not require a permanent development branch. Work may happen
directly on `main` through small verified commits or on a short-lived branch when
isolation and review are useful.

Commit subjects use `type(optional-scope): plain-language outcome`. Use `feat` for a
new user-visible capability, `fix` for a defect, `ci` for pipeline behavior, `test`
for test-only work, `docs` for documentation-only work, `refactor` for
behavior-preserving code structure, `build` for build/dependency changes, and
`chore` only when no more precise type fits.

Keep the subject simple and human-written. After one blank line, always use the body to
explain why the change was needed, the important behavior or architecture decision,
the evidence gathered, and any remaining limitation.

For example:

```text
ci: add minimal containerized workspace checks

Use the Jazzy ROS base image and install only current build and test dependencies.
Run dependency, formatting, compilation, URDF, launch, and test checks from clean
temporary outputs on every push and pull request.

Verified all four packages locally. Docker-hosted and remote Actions evidence remain
pending.
```
