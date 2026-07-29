# Project context management

## Purpose

Project context must reduce repeated reasoning without forcing every task to load
the entire history. Each fact, decision, plan, and piece of evidence has one owner
and one lifecycle.

More context is useful only when it is current, routed correctly, and relevant to
the active question.

## Context classes

| Class | Location | Lifecycle |
|---|---|---|
| Project definition | `docs/project/overview.md` | Update when scope or status changes |
| Architecture and contracts | `docs/project/architecture.md` | Update with an accepted boundary or invariant |
| Decisions and reasons | `docs/project/decisions.md` | Append or revise when a decision changes |
| Delivery workflow | `docs/project/development-workflow.md` | Update when the review or verification contract changes |
| Ordered work and gates | `docs/plan/roadmap.md` | Update as evidence advances a gate |
| Cross-cutting implementation plans | `docs/plan/` | Keep while the plan remains active |
| Verified failure analysis | `docs/rca/` | Retain while the failure mode remains relevant |
| Reference repositories | `context/ref_repos/` | Keep private and open only for a matching question |
| Working notes | `tasks/` | Replace or delete after the task; never treat as canonical |
| Tool-specific routers and rules | local instruction files | Point to canonical sources; do not duplicate them |

Personal motivation, conversational history, and comparisons with other people or
projects are not project context and must not be persisted.

## Loading order

For a project task, load context in this order:

1. the active roadmap step;
2. the affected architecture contract and recorded decisions;
3. the live source and configuration;
4. a matching RCA, if one exists;
5. the closest reference implementation, only if a question remains.

Do not scan every document or reference repository at session start. A task that is
already fully specified should normally stop at the live source.

## Update rules

- Runtime evidence outranks configuration, plans, summaries, and assumptions.
- Live source outranks a stale description of that source.
- When evidence changes an architectural fact, update its canonical document in the
  same slice.
- When a roadmap gate passes, record the evidence and change its status rather than
  merely declaring progress in conversation.
- When a link or router points to a missing file, correct or remove it immediately.
- When two files state the same rule, choose one owner and replace the duplicate
  with a link.
- Preserve an RCA only when it teaches a reusable failure mechanism.

## Pruning rules

Remove or archive context when it:

- points to a file that no longer exists;
- duplicates a canonical rule;
- describes a superseded architecture or workflow;
- records speculation as though it were measured fact;
- contains a transcript instead of a reusable conclusion; or
- is no longer reachable from a current project question.

Reference repositories are not removed merely because their hardware or ROS version
differs. Their values may not transfer, but their patterns and failure modes may
still answer a future question.

## Tool routing

Tool-specific entry points remain small. They identify the project, load the
development workflow and context policy, and route the current task to the relevant
canonical documents. They must not become independent copies of project truth.
