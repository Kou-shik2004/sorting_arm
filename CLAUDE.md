# CLAUDE.md — sorting_arm

A simulated sorting cell on UR5e + Robotiq 2F-85, in Gazebo Harmonic, built on top of MoveIt 2 and ros2_control. Koushik authors; you assist — review, debug, tutor, plumbing.

**Address me as Koushik in every reply.** If a reply drops the name, this file stopped loading and I need to know immediately.

Ubuntu 24.04 · ROS 2 Jazzy · Gazebo Harmonic · C++20 · MoveIt 2 · ros2_control · BehaviorTree.CPP 4.9. Workspace is `/sorting_arm_ws`; build with `cbuild`, test with `ctest_all`.

## Where the detail lives

`docs/PROJECT.md` — phases and their gates · `docs/architecture.md` — data flow, interfaces, recovery table · `docs/decisions.md` — D1–D11, every architectural choice with its reason. Read them when a task touches them, never automatically.

**`docs/reuse.md` is the reference-repo map, and it is not optional.** Before deriving any configuration, controller, launch or URDF detail, read the relevant `reuse.md` entry and open the file it cites in `context/ref_repos/`. `arm_stack` is the closest match to this robot — our gripper description is its gripper description. A running system in the corpus outranks anything I can reconstruct from installed headers or from memory of similar projects. If a `reuse.md` citation turns out to be wrong, correct `reuse.md`; do not work around it.

**If `docs/` is thin or silent on something, go to `context/plan/`** — the fuller masters live there and `docs/` is distilled from them. `context/` is private and gitignored: read it, never copy it into the repo.

## Scope

- **We build above MoveIt, never below.** Import or generate URDF, moveit_config, planners, IK, controllers, the gz bridge and worlds — never hand-write what a package already provides. Adapt existing worlds; do not author one from scratch.
- **No MoveIt Task Constructor.** The pick and place sequence is ours, in the tree and the skill servers.
- **Refuse out-of-phase work and remind me.** Perception and camera work is P7 only — poses come from sim ground truth until then.

## Code

- Logic lives in plain C++ classes; ROS wrappers stay thin. Hard to test means it is in the wrong layer.
- **Typed result codes, never bool.** Log the MoveIt error-code **enum name**, never "failed".
- Nothing hardcoded — poses and targets arrive externally, tunables are ROS parameters. No JSON payloads anywhere.
- Retries live inside the skill servers; cross-skill branches live in the tree.
- Tests ship in the same PR as the feature. gtest for pure logic. Test decisions, not coverage.
- **XML launch files.** One generated Python MoveIt bring-up is the allowed exception.
- Full C++ style, with the failure cases behind each rule: `.claude/rules/cpp-style.md`, sourced from `context/notes/robotics_cpp_notes.md`.
- **Comments are two lines maximum, in every file type.** Longer goes in `docs/` and the comment becomes a one-line pointer. **Never rewrite a comment I wrote.** `.claude/rules/comments.md`.

## How you work

- **Propose options with trade-offs, then STOP.** Do not write feature code unless I ask and name the file.
- Plan mode for anything non-trivial — 3+ steps or an architectural decision. If it goes sideways: stop and re-plan, never push through.
- **Use subagents liberally** for research, exploration and parallel analysis — one line of enquiry each, so the main context stays clean.
- **Use C++ IntelliSense and `compile_commands.json`** when working in C++ — read symbols and definitions rather than grepping blind. It is the most-authored part of this project.
- **Verification before "done":** show the test output or the log line. Never report complete on "should work".
- **Given a bug, fix it.** Point at the failing test or log and resolve it — no hand-holding, no asking me how.
- **Ask once whether there is a more elegant way** on a non-trivial change. If a fix feels hacky, redo it knowing what you know now. Skip this for obvious one-liners.
- Smallest change that fully solves it, root causes only — no `sleep_for()` settling hacks, no temporary patches.
- When I ask "why": the underlying concept in a few lines plus a doc link. Short.
- Review mode means pointing at problems and asking questions, not rewriting.
- Caveman style is for terminal replies only. Markdown, code, commits and docs are always normal prose.

## Task and lesson discipline

- `tasks/todo.md` is the live plan. Write the plan there first, check items off as they pass, and summarise what changed at each step.
- `tasks/lessons.md` gets a pattern after **every** correction I give you — the rule that would have prevented it, not the incident. Read it at session start.

## Repository

- Plain-English commits, no type prefixes. Feature branch → PR → CI green → squash-merge → tag the phase.
- **No AI attribution anywhere** — not in commits, PRs, comments or docs.
